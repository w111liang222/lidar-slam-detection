import os
import time
from threading import Thread, Event

import cv2
import numpy as np

from hardware.platform_common import is_jetson
from .data_manager_template import DataManagerTemplate
import hardware.gstreamer as driver
from sensor_driver.common_lib import cpp_utils
from util.image_util import get_image_size, undistort_image
from util.common_util import run_cmd
from ..export_interface import register_interface

# workaround for nvv4l2camerasrc (wrong stream data when system bootup)
def warmup_camera(logger, camera):
    if not is_jetson():
        return
    global_flag = '/tmp/warmup_camera_' + camera
    if os.path.exists(global_flag):
        return
    logger.info('start warmup camera %s' % (camera))
    run_cmd(''' gst-launch-1.0 v4l2src device=/dev/video{} num-buffers=1 ! 'video/x-raw' ! videoconvert ! 'video/x-raw,format=I420' ! fakesink'''.format(int(camera)))
    logger.info('finish warmup camera %s' % (camera))
    run_cmd('touch ' + global_flag)

class CameraDataManager(DataManagerTemplate):
    def __init__(self, cfgs, data_cfg, logger=None):
        self.clean_up()
        self.camera = dict()
        self.camera_config = dict()
        self.camera_info = dict()
        self.cap = dict()

        for idx, cfg in enumerate(cfgs.camera):
            self.camera_config[cfg.name] = cfg

        for idx, cfg in enumerate(cfgs.camera):
            w = cfg.output_width  if 'output_width'  in cfg else 640
            h = cfg.output_height if 'output_height' in cfg else 480
            self.camera_info[cfg.name] = {'w' : w, 'h' : h, 'in_w' : -1, 'in_h' : -1, 'valid' : False}
            self.cap[cfg.name] = self._generate_cap_string(idx, cfg, data_cfg.mode)

        self.image_param = self.get_image_param(cfgs.camera)

        super().__init__('Camera', cfgs, data_cfg, logger)
        self.camera_jpeg = CameraJpegDataManager(cfgs, data_cfg, logger)
        self.camera_jpeg.set_parent(self)
        register_interface('camera.get_camera_status', self.get_camera_status)

    def clean_up(self):
        run_cmd("rm -rf /tmp/camera_*")
        run_cmd(''' kill -9 `ps aux | grep "gst-launch-1.0 shmsrc" | grep -v grep | awk '{print $2}'` > /dev/null 2>&1 ''')
        run_cmd(''' kill -9 `ps aux | grep "launch_rtsp_server" | grep -v grep | awk '{print $2}'` > /dev/null 2>&1 ''')

    def get_image_param(self, cfgs):
        image_param = dict()
        for cfg in cfgs:
            w  = cfg.output_width  if 'output_width'  in cfg else 640
            h  = cfg.output_height if 'output_height' in cfg else 480
            fx = cfg.intrinsic_parameters[0]
            fy = cfg.intrinsic_parameters[1]
            cx = cfg.intrinsic_parameters[2]
            cy = cfg.intrinsic_parameters[3]

            ex_param = cfg.extrinsic_parameters
            V2C = cpp_utils.get_transform_from_RPYT(ex_param[0], ex_param[1], ex_param[2],
                                                    ex_param[5], ex_param[4], ex_param[3])

            intrinsic = np.array([[fx, 0, cx, 0],
                                  [0, fy, cy, 0],
                                  [0, 0,   1, 0]], dtype=np.float)
            meta = dict()
            meta['w'] = w
            meta['h'] = h
            meta['intrinsic'] = intrinsic
            meta['V2C'] = V2C
            image_param[cfg.name] = meta
        return image_param

    def _generate_cap_string(self, idx, cfg, mode):
        cap_command = ''

        # input camera parameter
        in_param = ''
        if 'input_width' in cfg and 'input_height' in cfg:
            in_param = in_param + ',width={}'.format(cfg.input_width) + ',height={}'.format(cfg.input_height)
        if 'format' in cfg:
            in_param = in_param + ',format={}'.format(cfg.format)

        # output camera parameter
        out_param = ''
        if 'output_width' in cfg and 'output_height' in cfg:
            out_param = out_param + ',width={}'.format(cfg.output_width) + ',height={}'.format(cfg.output_height)

        # distortion parameter
        distort_param = ''
        if 'undistortion' in cfg and cfg['undistortion'] is True:
            distort_param = driver.get_undistort(out_param)

        flip_method = 0
        if 'flip_method' in cfg:
            flip_method = cfg['flip_method']

        crop = None
        if 'crop' in cfg and 'input_width' in cfg and 'input_height' in cfg:
            crop = cfg['crop']
            crop_param = [cfg.input_width, cfg.input_height]

        if mode == "online":
            # local camera capture
            if cfg.name.isdigit():
                device = '/dev/video{}'.format(int(cfg.name))
                flip_method = driver.get_flip(flip_method) if flip_method != 0 else ""
                crop = driver.get_crop(crop, crop_param) if crop != None else ""
                cap_command = driver.get_local_camera(device, in_param, flip_method, crop, distort_param, out_param)

            # http camera
            if cfg.name.startswith("http://") and cfg.name.find(':', 7) != -1:
                location = cfg.name[0: cfg.name.find(':', 7)] + ':17777/stream?topic=' + cfg.name[cfg.name.find(':', 7) + 1:]
                crop = driver.get_crop(crop, crop_param) if crop != None else ""
                cap_command = driver.get_http_camera(location, in_param, crop, distort_param, out_param)

            # rtsp camera
            if cfg.name.startswith("rtsp://"):
                location = cfg.name
                crop = driver.get_crop(crop, crop_param) if crop != None else ""
                cap_command = driver.get_rtsp_camera(location, flip_method, in_param, crop, distort_param, out_param)

            if cfg.name.startswith("flir:"):
                flip_method = driver.get_flip(flip_method) if flip_method != 0 else ""
                crop = driver.get_crop(crop, crop_param) if crop != None else ""
                cap_command = driver.get_flir_camera(cfg.name[-1], in_param, flip_method, crop, distort_param, out_param)

            # usb camera
            if cfg.name.startswith("usb:"):
                video_num = int(cfg.name[-1])
                if not os.path.exists('/dev/v4l/by-id'): 
                    return {'cap': '', 'dameon': ''}
                usbcam_info = os.popen('ls -l /dev/v4l/by-id', 'r').readlines()
                device_list = []
                for usbcam_info_line in usbcam_info:
                    usbcam_info_line = usbcam_info_line.rstrip('\n')
                    if 'index0' in usbcam_info_line:
                        device_list.append(int(usbcam_info_line[-1]))
                if video_num >= len(device_list):
                    return {'cap': '', 'dameon': ''}
                device = '/dev/video{}'.format(device_list[video_num])
                flip_method = driver.get_flip(flip_method) if flip_method != 0 else ""
                crop = driver.get_crop(crop, crop_param) if crop != None else ""
                cap_command = driver.get_usb_camera(device, in_param, flip_method, crop, distort_param, out_param)

            # custom video capture
            if 'custom_cap' in cfg:
                cap_command = cfg.custom_cap
                return cap_command
        else:
            # local http mjpeg stream
            location = '''http://127.0.0.1:38000/v1/camera/{}'''.format(idx)
            crop = driver.get_crop(crop, crop_param) if crop != None else ""
            cap_command = driver.get_http_camera(location, in_param, crop, distort_param, out_param)

        if 'stream' in cfg and cfg.stream.sink == "udp":
            sink = driver.get_udp_sink(idx)
        elif 'stream' in cfg and cfg.stream.sink == "rtsp":
            sink =  driver.get_rtsp_sink(idx)
        else:
            sink = driver.get_app_sink(idx)

        cap_command = cap_command + sink

        # daemon service
        daemon_command = ''
        if 'stream' in cfg and cfg.stream.sink == "udp":
            daemon_command = driver.get_udp_sink_daemon(idx, out_param, cfg.stream.host, cfg.stream.port)
        elif 'stream' in cfg and cfg.stream.sink == "rtsp":
            audio = ""
            if 'enable_audio' in cfg.stream and cfg.stream.enable_audio:
                audio = driver.get_audio(cfg.stream.audio_name)
            bitrate = cfg.stream.bitrate if 'bitrate' in cfg.stream else 1000000
            daemon_command =  driver.get_rtsp_sink_daemon(idx, out_param, bitrate, audio, cfg.stream.port)

        return dict(cap = cap_command, daemon = daemon_command)

    def send_camera_config(self, cfg):
        CONFIG_FILE = '/tmp/camera_config'
        if os.path.exists(CONFIG_FILE):
            time.sleep(1)

        w = cfg.output_width  if 'output_width'  in cfg else 640
        h = cfg.output_height if 'output_height' in cfg else 480
        fisheye = cfg.get('fisheye', False)
        param = cfg.name + ' ' + str(w) + ' ' + str(h) + ' '
        for k in cfg.intrinsic_parameters:
            param = param + str(k) + ' '
        param = param + str(int(fisheye)) + ' ' + '0'

        with open(CONFIG_FILE, 'w') as f:
            f.write(param)
            os.fsync(f)

    def init(self):
        for idx in range(8):
            warmup_camera(self.logger, str(idx))

        for idx, cfg in enumerate(self.cfg.camera):
            if 'nvivafilter' in self.cap[cfg.name]["cap"] or 'opencvremap' in self.cap[cfg.name]["cap"]:
                self.send_camera_config(cfg)
            self.logger.info('camera %s, try cap: %s' % (cfg.name, self.cap[cfg.name]["cap"]))
            video_cap = cv2.VideoCapture(self.cap[cfg.name]["cap"], cv2.CAP_GSTREAMER)
            if not video_cap.isOpened():
                self.logger.warn('camera %s, can not open' % (cfg.name))
                continue

            self.camera[cfg.name] = video_cap
            if self.cap[cfg.name]["daemon"] != '':
                self.logger.info('camera %s, run service: %s' % (cfg.name, self.cap[cfg.name]["daemon"]))
                run_cmd(self.cap[cfg.name]["daemon"])
            self.logger.info('camera %s open success' % (cfg.name))

    def online_init(self):
        self.init()

    def offline_init(self):
        self.init()

    def loop_run_once(self, sensor, sensor_name):
        valid, frame = sensor.read()
        if not valid:
            self.logger.warn('camera %s, get invalid data' % (sensor_name))
            time.sleep(1.0)

        ret = dict()
        ret[sensor_name] = dict(
            timestamp=int(time.time() * 1000000),
            image=frame,
        )
        return ret, valid

    def start_capture(self):
        super().start_capture()
        self.start_loop(self.camera)
        self.camera_jpeg.start_capture()

    def stop_capture(self):
        self.camera_jpeg.release()
        for name, camera in self.camera.items():
            camera.release()

    def release(self):
        self.camera_jpeg.stop_capture()
        self.stop_loop()
        self.clean_up()

    def get_camera_status(self):
        return self.camera_info

    def update_cam_info(self, image_dict):
        for name, info in self.camera_info.items():
            info['valid'] = False
        for name, img in image_dict.items():
            self.camera_info[name]['valid'] = True
            self.camera_info[name]['in_w'], self.camera_info[name]['in_h'] = get_image_size(img)

    def process_image(self, image_dict, do_distort = None):
        if do_distort == True:
            for name, img in image_dict.items():
                image_dict[name] = undistort_image(image_dict[name], self.camera_config[name])
        return image_dict

    def post_process_data(self, data_dict, **kwargs):
        do_distort = kwargs['do_distort'] if 'do_distort' in kwargs else None
        if self.mode == "offline" and data_dict['image_valid']:
            data_dict['image'] = self.get_loop_data()
            for name, frame in data_dict['image'].items():
                data_dict['image_param'][name].update(self.image_param[name])
                data_dict['image'][name] = frame['image']
            data_dict['image_jpeg'] = self.camera_jpeg.get_loop_data()

            if not bool(data_dict['image']) or not bool(data_dict['image_jpeg']):
                data_dict['image'] = {}
                data_dict['image_jpeg'] = {}
                data_dict['image_valid'] = False

        data_dict['image'] = self.process_image(data_dict['image'], do_distort)
        self.update_cam_info(data_dict['image'])
        return data_dict

    def get_data_online(self, data_dict):
        if not bool(self.camera):
            return {'image_valid' : False, 'image' : dict(), 'image_param' : dict()}

        timeout = 0 if 'frame_start_timestamp' in data_dict else 1.0
        frame_dict = self.get_loop_data(timeout)
        if not bool(frame_dict):
            return {'image_valid' : False, 'image' : dict(), 'image_param' : dict()}

        if 'frame_start_timestamp' not in data_dict:
            data_dict['frame_start_timestamp'] = int(time.time() * 1000000)

        image_param = dict()
        for name, frame in frame_dict.items():
            image_param[name] = self.image_param[name]
            image_param[name]['timestamp'] = frame['timestamp']
            frame_dict[name] = frame['image']

        data = {'image_valid' : True, 'image' : frame_dict, 'image_jpeg' : self.camera_jpeg.get_loop_data(), 'image_param' : image_param}
        return data

    def get_data_offline(self, data_dict):
        return {'image_valid' : False, 'image' : dict(), 'image_param' : dict()}

class CameraJpegDataManager(DataManagerTemplate):
    def __init__(self, cfgs, data_cfg, logger=None):
        self.initial = False
        self.camera = dict()
        self.cap = dict()
        for idx, cfg in enumerate(cfgs.camera):
            self.cap[cfg.name] = self._generate_cap_string(idx, cfg)

        super().__init__('Jpeg', cfgs, data_cfg)

    def _generate_cap_string(self, idx, cfg):
        cap_command = '''shmsrc is-live=true socket-path=/tmp/camera_jpeg_{} do-timestamp=1 ! queue ! image/jpeg,width=1920,height=1080,framerate=(fraction)0 '''.format(idx)
        cap_command = cap_command + "! appsink sync=false"
        return cap_command

    def set_parent(self, parent):
        self.parent = parent

    def init(self):
        for idx, cfg in enumerate(self.cfg.camera):
            if cfg.name not in self.parent.camera:
                continue
            self.logger.info('camera jpeg %s, try cap: %s' % (cfg.name, self.cap[cfg.name]))
            thread_event = Event()
            def start_video_capture():
                video_cap = cv2.VideoCapture(self.cap[cfg.name], cv2.CAP_GSTREAMER)
                if not video_cap.isOpened():
                    self.logger.warn('camera jpeg %s, can not open' % (cfg.name))
                    return

                self.camera[cfg.name] = video_cap
                self.logger.info('camera jpeg %s open success' % (cfg.name))
                thread_event.set()

            thread = Thread(target=start_video_capture, args=(), daemon=True)
            thread.start()
            thread_event.wait(5.0)
            if not thread_event.is_set():
                self.logger.warn('camera jpeg %s, event is not set' % (cfg.name))
                self.parent.camera[cfg.name].release()
            thread.join()

        self.init = True

    def loop_run_once(self, sensor, sensor_name):
        valid, frame = sensor.read()
        ret = dict()
        ret[sensor_name] = frame
        return ret, valid

    def start_capture(self):
        if not self.initial:
            self.init()
        super().start_capture()
        self.start_loop(self.camera)

    def stop_capture(self):
        self.stop_loop()

    def release(self):
        for name, camera in self.camera.items():
            camera.release()
