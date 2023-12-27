from hardware.platform_common import MACHINE

FLIP_TEMPLATE = '''\
! videoflip method={} ! video/x-raw'''

CROP_TEMPLATE = '''\
! videocrop top={} bottom={} left={} right={} ! video/x-raw,width={},height={}'''

NV_CAP_TEMPLATE = '''\
v4l2src device={} ! video/x-raw{} {} {} ! videoscale ! video/x-raw{} {} ! videoconvert ! video/x-raw{},format=I420 '''

HTTP_CAP_TEMPLATE = '''\
souphttpsrc timeout=0 location={} ! jpegparse ! jpegdec ! video/x-raw{} {} ! videoscale ! video/x-raw{} {} ! videoconvert ! video/x-raw{},format=I420 '''

RTSP_CAP_TEMPLATE = '''\
rtspsrc location={} latency=0 ! decodebin ! videoflip method={} ! video/x-raw{} {} ! videoscale ! video/x-raw{} {} ! videoconvert ! video/x-raw{},format=I420 '''

FLIR_CAP_TEMPLATE = '''\
flirsrc device={} ! video/x-raw{} {} {} ! videoscale ! video/x-raw{} {} ! videoconvert ! video/x-raw{},format=I420 '''

ALSA_CAP_TEMPLATE = '''\
alsasrc device="plughw:{}" ! audioconvert ! audio/x-raw,rate=8000,channels=1 ! alawenc ! queue ! rtppcmapay pt=97 name=pay1 '''

UNDISTORT_TEMPLATE = '''\
! videoconvert ! video/x-raw{},format=I420 ! opencvremap ! video/x-raw{},format=I420 '''

APP_SINK_TEMPLATE = '''\
! tee name=t
    t. ! queue ! jpegenc ! shmsink socket-path=/tmp/camera_jpeg_{} sync=false
    t. ! queue ! appsink sync=false '''

RTSP_SINK_TEMPLATE = '''\
! tee name=t
    t. ! queue leaky=2 ! shmsink socket-path=/tmp/camera_I420_{} sync=false
    t. ! queue ! jpegenc ! shmsink socket-path=/tmp/camera_jpeg_{} sync=false
    t. ! queue ! appsink sync=false
'''

UDP_SINK_TEMPLATE = '''\
! tee name=t
    t. ! queue leaky=2 ! shmsink socket-path=/tmp/camera_I420_{} sync=false
    t. ! queue ! jpegenc ! shmsink socket-path=/tmp/camera_jpeg_{} sync=false
    t. ! queue ! appsink sync=false
'''

RTSP_SINK_DEAMON_TEMPLATE = '''\
third_party/{}/launch_rtsp_server -p {} -m /cam "shmsrc is-live=true socket-path=/tmp/camera_I420_{} do-timestamp=1 ! video/x-raw{},framerate=(fraction)0,format=I420 ! queue ! x264enc speed-preset=1 threads=8 tune=zerolatency sliced-threads=false bitrate={} ! h264parse ! rtph264pay pt=96 name=pay0 {}" &'''

UDP_SINK_DEAMON_TEMPLATE = '''\
gst-launch-1.0 shmsrc is-live=true socket-path="/tmp/camera_I420_{}" do-timestamp=1 ! 'video/x-raw{},framerate=(fraction)0,format=I420' ! queue ! x264enc speed-preset=1 threads=8 tune=zerolatency sliced-threads=false ! h264parse ! rtph264pay config-interval=1 pt=96 ! queue ! udpsink host="{}" port={} &'''

def get_flip_impl(method):
    return FLIP_TEMPLATE.format(method)

def get_crop_impl(crop, crop_param):
    return CROP_TEMPLATE.format(crop[0], crop[1], crop[2], crop[3], crop_param[0] - crop[2]- crop[3], crop_param[1] - crop[0] - crop[1])

def get_undistort_impl(out_param):
    return UNDISTORT_TEMPLATE.format(out_param, out_param)

def get_audio_impl(name):
    return ALSA_CAP_TEMPLATE.format(name)

def get_local_camera_impl(device, in_param, flip_method, crop, distort_param, out_param):
    return NV_CAP_TEMPLATE.format(device, in_param, flip_method, crop, out_param, distort_param, out_param)

def get_http_camera_impl(location, in_param, crop, distort_param, out_param):
    return HTTP_CAP_TEMPLATE.format(location, in_param, crop, out_param, distort_param, out_param)

def get_rtsp_camera_impl(location, flip_method, in_param, crop, distort_param, out_param):
    return RTSP_CAP_TEMPLATE.format(location, flip_method, in_param, crop, out_param, distort_param, out_param)

def get_flir_camera_impl(device, in_param, flip_method, crop, distort_param, out_param):
    return FLIR_CAP_TEMPLATE.format(device, in_param, flip_method, crop, out_param, distort_param, out_param)

def get_usb_camera_impl(device, in_param, flip_method, crop, distort_param, out_param):
    return NV_CAP_TEMPLATE.format(device, in_param, flip_method, crop, out_param, distort_param, out_param)

def get_app_sink_impl(idx):
    return APP_SINK_TEMPLATE.format(idx)

def get_rtsp_sink_impl(idx):
    return RTSP_SINK_TEMPLATE.format(idx, idx)

def get_udp_sink_impl(idx):
    return UDP_SINK_TEMPLATE.format(idx, idx)

def get_rtsp_sink_daemon_impl(idx, out_param, bitrate, audio, port):
    return RTSP_SINK_DEAMON_TEMPLATE.format(MACHINE, port, idx, out_param, int(bitrate / 1024), audio)

def get_udp_sink_daemon_impl(idx, out_param, dest, port):
    return UDP_SINK_DEAMON_TEMPLATE.format(idx, out_param, dest, port)

