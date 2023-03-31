import numpy as np

import sensor_driver.common_lib.cpp_utils as util
from module.detect.detect_template import DetectTemplate
import freespace_detection_wrapper as detection

class freespaceDetection(DetectTemplate):
    def __init__(self, detection_range = [0, 0, 0, 0, 0, 0], resolution = 0.2, logger = None):
        super().__init__(name = 'Freespace', logger = logger)
        self.detection_range = [float(x) for x in detection_range]
        self.resolution = resolution
        detection.set_grid_detection_params(
            self.detection_range[0], self.detection_range[1], self.detection_range[2],
            self.detection_range[3], self.detection_range[4], self.detection_range[5],
            float(self.resolution))

    def enqueue(self, input_dict):
        if not input_dict:
            return

        data_dict = {'lidar_valid': input_dict['lidar_valid'],
                     'points': input_dict['points'],
                     'frame_start_timestamp': input_dict['frame_start_timestamp']}
        self.input_queue.put_nowait(data_dict)

    def _run_thread(self):
        util.init_backtrace_handle()
        super()._run_thread()

    def process(self, input_dict):
        points = np.concatenate(list(input_dict['points'].values()), axis=0) if input_dict['lidar_valid'] else np.zeros((0, 4), dtype=np.float32)
        grid_cells = detection.detection(points, points.shape[0])
        result = {'frame_start_timestamp': input_dict['frame_start_timestamp'], 'freespace': grid_cells}
        return result
