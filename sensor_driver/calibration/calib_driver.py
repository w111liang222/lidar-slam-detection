import calib_driver_ext as driver

# Lidar Ins Calibration

def calib_ins_reset(initT):
    driver.calib_ins_reset(initT)

def calib_ins_add_points(points, timestamp):
    driver.calib_ins_set_points(points, timestamp)

def calib_ins_add_odom(odom, timestamp):
    driver.calib_ins_set_odom(odom, timestamp)

def calib_ins_calibrate():
    driver.calib_ins_calibrate()

def calib_ins_get_calibration():
    return driver.calib_ins_get_calibration()

def calib_ins_get_calibration_transform():
    return driver.calib_ins_get_calibration_transform()

# Lidar Imu Calibration

def calib_imu_reset():
    driver.calib_imu_reset()

def calib_imu_add_points(points, timestamp, odom, pose):
    driver.calib_imu_set_points(points, timestamp, odom, pose)

def calib_imu_add_imu(imu_data):
    driver.calib_imu_set_imu(imu_data)

def calib_imu_calibrate():
    return driver.calib_imu_calibrate()

def calib_imu_get_lidar_poses():
    return driver.calib_imu_get_lidar_poses()

def calib_imu_get_imu_poses():
    return driver.calib_imu_get_imu_poses()

def calib_imu_get_lidar_T_R(points):
    return driver.calib_imu_get_lidar_T_R(points)