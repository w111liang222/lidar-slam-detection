import radar_driver_ext as driver
from hardware.can import build_can
from sensor_driver.common_lib.cpp_utils import get_transform_from_RPYT
import numpy as np

class RadarDriver():
    def __init__(self, name, device, baud, token,
                    ex_param = [0, 0, 0, 0, 0, 0],
                    static_ex_param = [0, 0, 0, 0, 0, 0],
                    logger = None):
        self.token = token
        self.name = name
        self.device = device
        self.baud = baud
        self.ex_param = ex_param
        self.static_ex_param = static_ex_param
        self.logger = logger

    def open(self):
        driver.create_radar(self.token, self.name)
        driver.set_external_param(self.token, self.name, self.ex_param[0], self.ex_param[1], self.ex_param[2],
                                  self.ex_param[5], self.ex_param[4], self.ex_param[3])
    def close(self):
        driver.destory_radar(self.token, self.name)

    def start(self):
        self.can_device = build_can(self.baud, self.device, logger=self.logger)
        self.can_device.set_baud(self.baud)
        self.can_device.setup()
        self.can_device.send(id = 512, data = (b'\xff\x19\x00\x00\x08\x9d\x01\x00'), is_extended_id = True) #RadarConfiguration 0x200
        driver.start_capture(self.token, self.name, str(self.device))

    def stop(self):
        driver.stop_capture(self.token, self.name)

    def get_radar_frame_online(self, timeout):
        radar_frame = driver.get_radar_online(self.token, self.name, timeout)
        scan_start_timestamp = driver.get_timestamp(self.token, self.name)
        result = dict()
        result['radar_boxes'] = radar_frame[:, 0:7]
        result['radar_ids'] = radar_frame[:, 7]
        result["radar_types"] = radar_frame[:, 8]
        result['radar_velo'] = radar_frame[:, 9:11]
        return result
    
    def transform(self, data):
        if abs(self.ex_param[0] - self.static_ex_param[0]) < 1e-4 and \
            abs(self.ex_param[1] - self.static_ex_param[1]) < 1e-4 and \
            abs(self.ex_param[2] - self.static_ex_param[2]) < 1e-4 and \
            abs(self.ex_param[3] - self.static_ex_param[3]) < 1e-4 and \
            abs(self.ex_param[4] - self.static_ex_param[4]) < 1e-4 and \
            abs(self.ex_param[5] - self.static_ex_param[5]) < 1e-4:
            return data
        tM = get_transform_from_RPYT(self.ex_param[0], self.ex_param[1], self.ex_param[2],
                                        self.ex_param[5], self.ex_param[4], self.ex_param[3])

        static_tM = get_transform_from_RPYT(self.static_ex_param[0], self.static_ex_param[1], self.static_ex_param[2],
                                            self.static_ex_param[5], self.static_ex_param[4], self.static_ex_param[3])

        tM = np.dot(tM, np.linalg.inv(static_tM))
        xyz_hom = np.hstack((data['radar_boxes'][:, :3], np.ones((data['radar_boxes'].shape[0], 1), dtype=np.float32)))
        data['radar_boxes'][:, 0:3] = np.dot(tM, xyz_hom.T).T[:, 0:3]
        data['radar_velo'] = np.dot(tM[0:2,0:2], data['radar_velo'].T).T
        return data
