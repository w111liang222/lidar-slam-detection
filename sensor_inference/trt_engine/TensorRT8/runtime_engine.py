import tensorrt as trt
from pycuda import driver
from ..tensorrt_engine import Engine

TRT_LOGGER = trt.Logger(trt.Logger.VERBOSE)

from ctypes import cdll, c_char_p
libcudart = cdll.LoadLibrary('libcudart.so')
libcudart.cudaGetErrorString.restype = c_char_p
def cudaSetDevice(device_idx):
    ret = libcudart.cudaSetDevice(device_idx)
    if ret != 0:
        error_string = libcudart.cudaGetErrorString(ret)
        raise RuntimeError("cudaSetDevice: " + error_string)

class RuntimeBackend:
    def __init__(self, trt_path):
        driver.init()
        self.ctx = driver.Device(0).make_context()
        cudaSetDevice(0)
        print('Reading engine from trt model {}'.format(trt_path))
        if not trt.init_libnvinfer_plugins(TRT_LOGGER, ""):
            msg = "Failed to initialize TensorRT's plugin library."
            raise RuntimeError(msg)
        with open(trt_path, "rb") as f, trt.Runtime(TRT_LOGGER) as runtime:
            trt_engine = runtime.deserialize_cuda_engine(f.read())
            self.engine = Engine(trt_engine)

    def run(self, inputs):
        return self.engine.run(inputs)