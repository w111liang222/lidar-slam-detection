import cpp_utils_ext
import threading
import numpy as np

def init_backtrace_handle():
    cpp_utils_ext.init_backtrace_handle()

def set_thread_priority(name, priority):
    threading.current_thread().name = name
    cpp_utils_ext.set_thread_priority(name, priority)

def set_logger_level(level):
    cpp_utils_ext.set_logger_level(level)

def set_message_core(enable):
    cpp_utils_ext.set_message_core(enable)

def publish_message(filename):
    return cpp_utils_ext.publish_message(filename)

def get_projection_forward(lat0, lon0, lat1, lon1):
    return cpp_utils_ext.get_projection_forward(lat0, lon0, lat1, lon1).tolist()

def get_projection_backward(lat0, lon0, x, y):
    return cpp_utils_ext.get_projection_backward(lat0, lon0, x, y).tolist()

def get_transform_from_RPYT(x, y, z, yaw, pitch, roll):
    return np.array(cpp_utils_ext.get_transform_from_RPYT(x, y, z, yaw, pitch, roll)).reshape(4, 4)

def get_RPYT_from_transform(tM):
    return cpp_utils_ext.get_RPYT_from_transform(tM)

def get_transform_from_cfg(x, y, z, roll, pitch, yaw):
    return np.array(cpp_utils_ext.get_transform_from_RPYT(x, y, z, yaw, pitch, roll)).reshape(4, 4)

def get_cfg_from_transform(tM):
    x, y, z, yaw, pitch, roll = cpp_utils_ext.get_RPYT_from_transform(tM)
    return [x, y, z, roll, pitch, yaw]

def get_transform_from_rtk(lat0, lon0, alt0, yaw0, pitch0, roll0,
                           lat1, lon1, alt1, yaw1, pitch1, roll1):
    return np.array(cpp_utils_ext.computeRTKTransform(lat0, lon0, alt0, yaw0, pitch0, roll0,
                                                      lat1, lon1, alt1, yaw1, pitch1, roll1)).reshape(4, 4)

# transform T0 -> T1
def get_relative_transform(lat0, lon0, alt0, yaw0, pitch0, roll0,
                           lat1, lon1, alt1, yaw1, pitch1, roll1,
                           x, y, z, h, p, r):
    return np.array(cpp_utils_ext.getRelativeTransform(lat0, lon0, alt0, yaw0, pitch0, roll0,
                                                       lat1, lon1, alt1, yaw1, pitch1, roll1,
                                                       x, y, z, h, p, r)).reshape(4, 4)

def pointcloud_downsample(points, voxel_size):
    return cpp_utils_ext.pointcloud_downsample(points, voxel_size)

def get_association(det_len, trk_len, matched_indices, threshold, iou_matrix):
    return cpp_utils_ext.get_association(det_len, trk_len, matched_indices, threshold, iou_matrix)

def init_filters():
    cpp_utils_ext.init_filters()

def use_filter(handle, is_static, x):
    cpp_utils_ext.use_filter(handle, is_static, x)

def filter_predict(handle, dt, motion, dh):
    return cpp_utils_ext.filter_predict(handle, dt, motion, dh)

def filter_update(handle, z):
    return cpp_utils_ext.filter_update(handle, z)

def motion_prediction(handle, trajectory):
    cpp_utils_ext.motion_prediction(handle, trajectory)
