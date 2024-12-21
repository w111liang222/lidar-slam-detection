import time
import pickle
from pathlib import Path
from multiprocessing import Process, Queue, Event
import cv2
import numpy as np

from util.image_util import get_image_size
from .data_manager_template import DataManagerTemplate
import sensor_driver.common_lib.cpp_utils as util
from ..export_interface import register_interface

class PlayerDataManager(DataManagerTemplate):
    def __init__(self, cfg, system, logger=None):
        self.system = system
        self.data_list = []
        self.start_time, self.stop_time = 0, 0
        self.current_time = 0
        self.current_idx = 0
        self.rate = 1.0
        super().__init__('Player', cfg, logger = logger)

        register_interface('playback.get_player_status', self.get_status)
        register_interface('playback.player_seek', self.seek_to)
        register_interface('playback.set_player_rate', self.set_rate)
        register_interface('playback.set_player_step', self.set_step)

    def offline_init(self):
        self.get_duration()
        self.get_camera_parameter(self.cfg.camera)
        self.last_walltime = time.monotonic()
        self.current_time = self.start_time

        event = Event()
        self.thread = Process(target=self.camera_server_main, args=(event,))
        self.thread.daemon = True
        self.thread.start()
        event.wait()

    def camera_server_main(self, event):
        from flask import Flask
        util.set_thread_priority("playback", 30)
        app = Flask(__name__)
        app.add_url_rule("/v1/camera/<name>", view_func=self.request_camera_data)
        event.set()
        app.run(host='127.0.0.1', port=38000)

    def stream_image(self, name):
        template = np.zeros((self.camera_stream[name]["height"], self.camera_stream[name]["width"], 3), dtype=np.uint8)
        img_zero = cv2.imencode('.jpg', template)[1].tobytes()
        yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + img_zero + b'\r\n')
        while True:
            try:
                img = self.camera_stream[name]["queue"].get(block=True, timeout=5.0)
            except Exception as e:
                img = img_zero
            yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + img + b'\r\n')

    def request_camera_data(self, name):
        from flask import Response, abort
        name = int(name)
        if name not in self.camera_stream:
            return abort(404)
        return Response(self.stream_image(name), mimetype='multipart/x-mixed-replace; boundary=frame')

    def seek_to(self, percent):
        self.current_idx = max(int(len(self.data_list) * percent / 100) - 1, 0)
        return self.data_list[self.current_idx]

    def set_rate(self, rate):
        self.rate = int(rate) if rate > 1.0 else rate

    def set_step(self, step):
        if not self.system.is_running:
            current_idx = min(len(self.data_list) - 1, max(0, self.current_idx + int(step)))
            self.current_idx = current_idx
        return self.data_list[current_idx]

    def get_camera_parameter(self, camera_config):
        self.camera_name_map = dict()
        self.camera_stream = dict()
        if len(self.data_list) <= 0:
            return
        for idx, cfg in enumerate(camera_config):
            self.camera_name_map[cfg.name] = idx

        try:
            data_dict = self.parse_pickle(self.data_list[0])
            for name, image in data_dict['image'].items():
                if name not in self.camera_name_map:
                    continue
                self.camera_stream[self.camera_name_map[name]] = dict()
                width, height = get_image_size(cv2.imdecode(np.frombuffer(image, dtype=np.uint8), cv2.IMREAD_COLOR))
                self.camera_stream[self.camera_name_map[name]]["width"]  = width
                self.camera_stream[self.camera_name_map[name]]["height"] = height
                self.camera_stream[self.camera_name_map[name]]["queue"]  = Queue(maxsize=3)
        except Exception as e:
            self.logger.error(e)

    def get_timestamp(self, idx):
        if idx >= len(self.data_list) or idx < 0:
            return 0
        fpath = self.data_list[idx]
        try:
            data_dict = self.parse_pickle(fpath)
            return data_dict['frame_timestamp_monotonic']
        except Exception as e:
            self.logger.error(e)
            return 0

    def get_duration(self):
        root_path = Path(self.cfg.input.data_path).expanduser().absolute()
        if not root_path.exists():
            self.logger.warn("{} does not exist!".format(str(root_path)))
        self.data_list = list(map(str, root_path.glob('*.pkl')))
        self.data_list.sort()

        self.start_time = 0
        while len(self.data_list) > 0 and self.start_time == 0:
            self.start_time = self.get_timestamp(0) / 1000000.0
            if self.start_time == 0:
                self.data_list = self.data_list[1:]

        self.stop_time = 0
        while len(self.data_list) > 0 and self.stop_time == 0:
            self.stop_time = self.get_timestamp(len(self.data_list) - 1) / 1000000.0
            if self.stop_time == 0:
                self.data_list = self.data_list[0:-1]

        if self.stop_time <= self.start_time:
            self.start_time = 0
            self.stop_time = 0
            self.data_list = []

        duration = self.stop_time - self.start_time
        self.logger.info('player have %d frames, duration %f s' % (len(self.data_list), duration))

    def get_status(self):
        now_time = self.current_time - self.start_time
        left_time = self.stop_time - self.current_time
        percent = self.current_idx / len(self.data_list) * 100 if len(self.data_list) > 0 else 0
        return dict(
            now_time  = "{0:02d}:{1:02d}".format(int(now_time / 60), int(now_time % 60)),
            left_time = "{0:02d}:{1:02d}".format(int(left_time / 60), int(left_time % 60)),
            percent   = percent,
        )

    def parse_pickle(self, file_name):
        f = open(file_name, 'rb', buffering=10*1024*1024)
        data_dict = pickle.loads(f.read())
        if 'frame_timestamp_monotonic' not in data_dict:
            data_dict['frame_timestamp_monotonic'] = data_dict['frame_start_timestamp']

        if 'points_attr' not in data_dict:
            data_dict['points_attr'] = dict()
            for name, data in data_dict['points'].items():
                data_dict['points_attr'][name] = dict(
                    timestamp=data_dict['frame_start_timestamp'],
                    points_attr=np.zeros((data.shape[0], 2), dtype=np.float32)
                )

        for name, data in data_dict['points'].copy().items():
            if 'Ouster-OS1' in name:
                data_dict['points'][name[0] + '-' + 'Ouster-OS1'] = data_dict['points'].pop(name)
                data_dict['points_attr'][name[0] + '-' + 'Ouster-OS1'] = data_dict['points_attr'].pop(name)
            elif 'Ouster-OS2' in name:
                data_dict['points'][name[0] + '-' + 'Ouster-OS2'] = data_dict['points'].pop(name)
                data_dict['points_attr'][name[0] + '-' + 'Ouster-OS2'] = data_dict['points_attr'].pop(name)

        for name, param in data_dict['image_param'].items():
            if 'timestamp' not in param:
                param['timestamp'] = data_dict['frame_start_timestamp'] + 100000

        if 'pose' in data_dict and 'area' not in data_dict['pose']:
            data_dict['pose']['area'] = None

        if data_dict['ins_valid'] and 'imu_data' not in data_dict:
            data_dict['imu_data'] = np.asarray([[data_dict['ins_data']['timestamp'],
                                                 data_dict['ins_data']['gyro_x'],
                                                 data_dict['ins_data']['gyro_y'],
                                                 data_dict['ins_data']['gyro_z'],
                                                 data_dict['ins_data']['acc_x'],
                                                 data_dict['ins_data']['acc_y'],
                                                 data_dict['ins_data']['acc_z']]], dtype=np.float64)

        data_dict['ins_data']['Sensor'] = data_dict['ins_data']['Sensor'] if 'Sensor' in data_dict['ins_data'] else "GNSS"

        if 'motion_valid' not in data_dict:
            data_dict['motion_valid'] = data_dict['ins_valid']

        return data_dict

    def loop_run_once(self):
        if len(self.data_list) == 0:
            return {}

        if self.current_idx >= len(self.data_list):
            self.current_idx = len(self.data_list) - 1
            time.sleep(1e-1)

        fpath = self.data_list[self.current_idx]
        if self.system.is_running:
            self.current_idx = self.current_idx + int(1 if self.rate < 1.0 else self.rate)
            time_offset = 0
        else:
            time_offset = 0.1

        try:
            data_dict = self.parse_pickle(fpath)
            data_dict['image_valid'] = False
            data_dict['lidar_valid'] = bool(data_dict['points'])
            images = data_dict.pop('image', dict())
            for name, image in images.items():
                if name not in self.camera_name_map:
                    continue
                camera_index = self.camera_name_map[name]
                if camera_index not in self.camera_stream:
                    continue
                if self.camera_stream[camera_index]["queue"].full():
                    continue
                self.camera_stream[camera_index]["queue"].put_nowait(image)
                data_dict['image_valid'] = True

            current_time = data_dict['frame_timestamp_monotonic'] / 1000000.0
            sleep_time = (current_time - self.current_time + time_offset) / self.rate - (time.monotonic() - self.last_walltime)
            interval = 0.1 / (self.rate if self.rate < 1.0 else 1.0)
            if sleep_time > 0.01 and sleep_time < (interval + 0.01):
                time.sleep(sleep_time)
            else:
                time.sleep(0.01)
            self.current_time = current_time
            self.last_walltime = time.monotonic()
        except Exception as e:
            self.logger.error(e)
            return {}
        return data_dict

    def start_capture(self):
        pass

    def stop_capture(self):
        pass

    def release(self):
        if self.mode == "offline":
            self.thread.terminate()
            self.thread.join()

    def get_data_online(self, data_dict):
        return {}

    def get_data_offline(self, data_dict):
        start_time = time.monotonic()
        data = self.loop_run_once()
        if bool(data):
            data_dict['frame_timestamp_monotonic'] = data['frame_timestamp_monotonic']
        parse_time = (time.monotonic() - start_time) * 1000
        if parse_time < 1000:
            self.logger.debug('player parse pickle cost: %.1f ms' % (parse_time))
        else:
            self.logger.warn('player parse pickle cost: %.1f ms' % (parse_time))
        return data
