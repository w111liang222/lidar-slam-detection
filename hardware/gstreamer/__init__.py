from hardware.platform_common import is_jetson

if is_jetson():
    from .jetson.driver import *
else:
    from .base.driver import *

def get_flip(method):
    return get_flip_impl(method)

def get_crop(crop, crop_param):
    return get_crop_impl(crop, crop_param)

def get_undistort(out_param):
    return get_undistort_impl(out_param)

def get_audio(name):
    return get_audio_impl(name)

def get_local_camera(device, in_param, flip_method, crop, distort_param, out_param):
    return get_local_camera_impl(device, in_param, flip_method, crop, distort_param, out_param)

def get_http_camera(location, in_param, crop, distort_param, out_param):
    return get_http_camera_impl(location, in_param, crop, distort_param, out_param)

def get_rtsp_camera(location, flip_method, in_param, crop, distort_param, out_param):
    return get_rtsp_camera_impl(location, flip_method, in_param, crop, distort_param, out_param)

def get_flir_camera(device, in_param, flip_method, crop, distort_param, out_param):
    return get_flir_camera_impl(device, in_param, flip_method, crop, distort_param, out_param)

def get_usb_camera(device, in_param, flip_method, crop, distort_param, out_param):
    return get_usb_camera_impl(device, in_param, flip_method, crop, distort_param, out_param)

def get_app_sink(idx):
    return get_app_sink_impl(idx)

def get_rtsp_sink(idx):
    return get_rtsp_sink_impl(idx)

def get_udp_sink(idx):
    return get_udp_sink_impl(idx)

def get_rtsp_sink_daemon(idx, out_param, bitrate, audio, port):
    return get_rtsp_sink_daemon_impl(idx, out_param, bitrate, audio, port)

def get_udp_sink_daemon(idx, out_param, dest, port):
    return get_udp_sink_daemon_impl(idx, out_param, dest, port)