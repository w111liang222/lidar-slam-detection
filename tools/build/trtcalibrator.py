import tensorrt as trt
import pycuda.driver as cuda
import pycuda.autoinit

import os
import numpy as np

dtype_map = {np.float32: trt.DataType.FLOAT,
             np.float16: trt.DataType.HALF,
             np.int8: trt.DataType.INT8}
if hasattr(trt.DataType, 'INT32'):
    dtype_map[np.int32] = trt.DataType.INT32

class DetNetEntropyCalibrator(trt.IInt8EntropyCalibrator2):

    def __init__(self, batch_size, data_file_list, lidar, dataset):
        trt.IInt8EntropyCalibrator2.__init__(self)

        self.cache_file = 'tools/Int8_model.cache'

        self.batch_size = batch_size
        self.data_file_list = data_file_list
        np.random.shuffle(self.data_file_list)
        self.lidar = lidar
        self.dataset = dataset

        self.batch_idx = 0
        self.max_batch_idx = len(self.data_file_list)//self.batch_size

        self.device_inputs = None

    def next_batch(self):
        batch_files = self.data_file_list[self.batch_idx * self.batch_size:(self.batch_idx + 1) * self.batch_size]
        self.batch_idx += 1

        for i, file in enumerate(batch_files):
            points, frame_start_timestamp = self.lidar.get_points_from_file(file)
            data_dict = self.dataset.prepare_data(points)

        print("batch:[{}/{}]".format(self.batch_idx, self.max_batch_idx))
        return data_dict

    def get_batch_size(self):
        return self.batch_size

    def get_batch(self, names, p_str=None):
        if self.batch_idx >= self.max_batch_idx:
            return None

        data_dict = self.next_batch()
        input_tuple = list(data_dict.values())
        input_tuple[1] = input_tuple[1].astype(np.float32)
        input_tuple[2] = input_tuple[2].astype(np.float32)

        if self.device_inputs is None:
            self.device_inputs = []
            for input_item in input_tuple:
                data_size = trt.volume(input_item.shape) * trt.float32.itemsize
                device_input = cuda.mem_alloc(data_size)
                self.device_inputs.append(device_input)

        for input_item, device_input in zip(input_tuple, self.device_inputs):
            input_item = np.ascontiguousarray(input_item)
            cuda.memcpy_htod(device_input, input_item)

        dev_inputs = []
        for device_input in self.device_inputs:
            dev_inputs.append(int(device_input))
        return dev_inputs

    def read_calibration_cache(self):
        if os.path.exists(self.cache_file):
            with open(self.cache_file, "rb") as f:
                return f.read()

    def write_calibration_cache(self, cache):
        with open(self.cache_file, "wb") as f:
            f.write(cache)