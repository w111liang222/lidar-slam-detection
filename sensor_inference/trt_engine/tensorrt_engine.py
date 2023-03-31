import tensorrt as trt
import pycuda.driver
import pycuda.gpuarray
import numpy as np
from six import string_types

dtype_map = {trt.DataType.FLOAT: np.float32,
             trt.DataType.HALF:  np.float16,
             trt.DataType.INT8:  np.int8,
             trt.DataType.INT32: np.int32}

class Binding(object):
    def __init__(self, engine, idx_or_name):
        if isinstance(idx_or_name, string_types):
            self.name = idx_or_name
            self.index  = engine.get_binding_index(self.name)
            if self.index == -1:
                raise IndexError("Binding name not found: %s" % self.name)
        else:
            self.index = idx_or_name
            self.name  = engine.get_binding_name(self.index)
            if self.name is None:
                raise IndexError("Binding index out of range: %i" % self.index)
        self.is_input = engine.binding_is_input(self.index)

        dtype = engine.get_binding_dtype(self.index)
        self.dtype = dtype_map[dtype]
        shape = engine.get_binding_shape(self.index)
        for i in range(0, len(shape)):
            if shape[i] == -1:
                shape[i] = 1

        self.shape = tuple(shape)
        self._host_buf   = None
        self._device_buf = None
    @property
    def host_buffer(self):
        if self._host_buf is None:
            self._host_buf = pycuda.driver.pagelocked_empty(self.shape, self.dtype)
        return self._host_buf
    @property
    def device_buffer(self):
        if self._device_buf is None:
            self._device_buf = pycuda.gpuarray.empty(self.shape, self.dtype)
        return self._device_buf
    def get_async(self, stream):
        src = self.device_buffer
        dst = self.host_buffer
        src.get_async(stream, dst)
        return dst

class Engine(object):
    def __init__(self, trt_engine):
        self.engine = trt_engine
        nbinding = self.engine.num_bindings

        bindings = [Binding(self.engine, i)
                    for i in range(nbinding)]
        self.inputs  = [b for b in bindings if     b.is_input]
        self.outputs = [b for b in bindings if not b.is_input]
        self.bindings = bindings

        #for binding in self.inputs + self.outputs:
        #    _ = binding.device_buffer # Force buffer allocation
        #for binding in self.outputs:
        #    _ = binding.host_buffer   # Force buffer allocation
        self.context = self.engine.create_execution_context()
        self.context.active_optimization_profile = 0
        # self.context.profiler = trt.Profiler()
        self.stream = pycuda.driver.Stream()
    def __del__(self):
        if self.engine is not None:
            del self.engine

    def run(self, inputs):
        if len(inputs) < len(self.inputs):
            raise ValueError("Not enough inputs. Expected %i, got %i." %
                             (len(self.inputs), len(inputs)))

        for i, (input_array, input_binding) in enumerate(zip(inputs, self.inputs)):
            self.context.set_binding_shape(i, input_array.shape)
            input_binding._device_buf = pycuda.gpuarray.to_gpu_async(input_array, stream=self.stream)

        self.binding_addrs = [b.device_buffer.ptr for b in self.bindings]
        self.context.execute_async_v2(
            self.binding_addrs, self.stream.handle)

        results = [output.get_async(self.stream)
                   for output in self.outputs]
        self.stream.synchronize()
        return results
