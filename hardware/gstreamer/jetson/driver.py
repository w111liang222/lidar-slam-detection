FLIP_TEMPLATE = '''\
! nvvidconv flip-method={} ! video/x-raw(memory:NVMM)'''

CROP_TEMPLATE = '''\
! nvvidconv top={} bottom={} left={} right={} ! video/x-raw(memory:NVMM),width={},height={}'''

NV_CAP_TEMPLATE = '''\
nvv4l2camerasrc device={} ! video/x-raw(memory:NVMM){} {} {} {} ! nvvidconv ! video/x-raw{},format=I420 '''

USB_CAP_TEMPLATE = '''\
v4l2src device={} ! video/x-raw{} ! videoconvert ! nvvidconv ! video/x-raw(memory:NVMM){} {} {} ! nvvidconv ! video/x-raw{},format=I420 '''

HTTP_CAP_TEMPLATE = '''\
souphttpsrc timeout=0 location={} ! jpegparse ! nvjpegdec ! video/x-raw(memory:NVMM){} {} {} ! nvvidconv ! video/x-raw{},format=I420 '''

RTSP_CAP_TEMPLATE = '''\
rtspsrc location={} latency=0 ! decodebin ! nvvidconv flip-method={} ! video/x-raw(memory:NVMM){} {} {} ! nvvidconv ! video/x-raw{},format=I420 '''

FLIR_CAP_TEMPLATE = '''\
flirsrc device={} ! video/x-raw{} ! nvvidconv ! video/x-raw(memory:NVMM){} {} {} ! nvvidconv ! video/x-raw{},format=I420 '''

ALSA_CAP_TEMPLATE = '''\
alsasrc device="plughw:{}" ! audioconvert ! audio/x-raw,rate=8000,channels=1 ! alawenc ! queue ! rtppcmapay pt=97 name=pay1 '''

NVFILTER_TEMPLATE = '''\
! nvvidconv ! video/x-raw(memory:NVMM){},format=NV12 ! nvivafilter customer-lib-name=./hardware/nvivafilter/lib-gst-custom-opencv_cudaprocess.so cuda-process=true ! video/x-raw(memory:NVMM){},format=RGBA '''

APP_SINK_TEMPLATE = '''\
! tee name=t
    t. ! queue ! nvjpegenc ! shmsink socket-path=/tmp/camera_jpeg_{} sync=false
    t. ! queue ! appsink sync=false '''

RTSP_SINK_TEMPLATE = '''\
! tee name=t
    t. ! queue leaky=2 ! shmsink socket-path=/tmp/camera_I420_{} sync=false
    t. ! queue ! nvjpegenc ! shmsink socket-path=/tmp/camera_jpeg_{} sync=false
    t. ! queue ! appsink sync=false
'''

UDP_SINK_TEMPLATE = '''\
! tee name=t
    t. ! queue leaky=2 ! shmsink socket-path=/tmp/camera_I420_{} sync=false
    t. ! queue ! nvjpegenc ! shmsink socket-path=/tmp/camera_jpeg_{} sync=false
    t. ! queue ! appsink sync=false
'''

RTSP_SINK_DEAMON_TEMPLATE = '''\
third_party/aarch64/launch_rtsp_server -p {} -m /cam "shmsrc is-live=true socket-path=/tmp/camera_I420_{} do-timestamp=1 ! video/x-raw{},framerate=(fraction)0,format=I420 ! nvvidconv ! nvv4l2h264enc maxperf-enable=1 control-rate=1 bitrate={} iframeinterval=30 preset-level=3 insert-sps-pps=1 ! h264parse ! queue ! rtph264pay pt=96 name=pay0 {}" &'''

UDP_SINK_DEAMON_TEMPLATE = '''\
gst-launch-1.0 shmsrc is-live=true socket-path="/tmp/camera_I420_{}" do-timestamp=1 ! 'video/x-raw{},framerate=(fraction)0,format=I420' ! nvvidconv ! nvv4l2h264enc maxperf-enable=1 preset-level=3 insert-sps-pps=1 ! h264parse ! rtph264pay config-interval=1 pt=96 ! queue ! udpsink host="{}" port={} &'''

def get_flip_impl(method):
    return FLIP_TEMPLATE.format(method)

def get_crop_impl(crop, crop_param):
    return CROP_TEMPLATE.format(crop[0], crop_param[1] - crop[1], crop[2], crop_param[0] - crop[3], crop_param[0] - crop[2]- crop[3], crop_param[1] - crop[0] - crop[1])

def get_undistort_impl(out_param):
    return NVFILTER_TEMPLATE.format(out_param, out_param)

def get_audio_impl(name):
    return ALSA_CAP_TEMPLATE.format(name)

def get_local_camera_impl(device, in_param, flip_method, crop, distort_param, out_param):
    return NV_CAP_TEMPLATE.format(device, in_param, flip_method, crop, distort_param, out_param)

def get_http_camera_impl(location, in_param, crop, distort_param, out_param):
    return HTTP_CAP_TEMPLATE.format(location, in_param, crop, distort_param, out_param)

def get_rtsp_camera_impl(location, flip_method, in_param, crop, distort_param, out_param):
    return RTSP_CAP_TEMPLATE.format(location, flip_method, in_param, crop, distort_param, out_param)

def get_flir_camera_impl(device, in_param, flip_method, crop, distort_param, out_param):
    return FLIR_CAP_TEMPLATE.format(device, in_param, flip_method, crop, distort_param, out_param)

def get_usb_camera_impl(device, in_param, flip_method, crop, distort_param, out_param):
    return USB_CAP_TEMPLATE.format(device, in_param, flip_method, crop, distort_param, out_param)

def get_app_sink_impl(idx):
    return APP_SINK_TEMPLATE.format(idx)

def get_rtsp_sink_impl(idx):
    return RTSP_SINK_TEMPLATE.format(idx, idx)

def get_udp_sink_impl(idx):
    return UDP_SINK_TEMPLATE.format(idx, idx)

def get_rtsp_sink_daemon_impl(idx, out_param, bitrate, audio, port):
    return RTSP_SINK_DEAMON_TEMPLATE.format(port, idx, out_param, bitrate, audio)

def get_udp_sink_daemon_impl(idx, out_param, dest, port):
    return UDP_SINK_DEAMON_TEMPLATE.format(idx, out_param, dest, port)

