import tensorrt as trt
import six

TRT_LOGGER = trt.Logger(trt.Logger.WARNING)

from ctypes import cdll, c_char_p
libcudart = cdll.LoadLibrary('libcudart.so')
libcudart.cudaGetErrorString.restype = c_char_p
def cudaSetDevice(device_idx):
    ret = libcudart.cudaSetDevice(device_idx)
    if ret != 0:
        error_string = libcudart.cudaGetErrorString(ret)
        raise RuntimeError("cudaSetDevice: " + error_string)

class PrepareBackend:
    def __init__(self, model, max_batch_size=1, build_fp16 = False,
                 max_workspace_size=None, calib = None, trt_path = None, use_DLA = False, **kwargs):
        cudaSetDevice(0)
        print('Reading engine from onnx')
        self.builder = trt.Builder(TRT_LOGGER)
        self.network = self.builder.create_network(flags=1 << (int)(trt.NetworkDefinitionCreationFlag.EXPLICIT_BATCH))
        self.parser = trt.OnnxParser(self.network, TRT_LOGGER)

        if not isinstance(model, six.string_types):
            model_str = model.SerializeToString()
        else:
            model_str = model

        if not trt.init_libnvinfer_plugins(TRT_LOGGER, ""):
            msg = "Failed to initialize TensorRT's plugin library."
            raise RuntimeError(msg)

        if not self.parser.parse(model_str):
            error = self.parser.get_error(0)
            msg = "While parsing node number %i:\n" % error.node()
            msg += ("%s:%i In function %s:\n[%i] %s" %
                    (error.file(), error.line(), error.func(),
                     error.code(), error.desc()))
            raise RuntimeError(msg)
        if max_workspace_size is None:
            max_workspace_size = 1 << 29

        self.builder.max_batch_size = max_batch_size
        # self.builder.max_workspace_size = max_workspace_size

        builder_config = self.builder.create_builder_config()
        builder_config.max_workspace_size = max_workspace_size
        builder_config.set_flag(trt.BuilderFlag.GPU_FALLBACK)

        if 'dynamic_shape' in kwargs.keys():
            profile = self.builder.create_optimization_profile()
            for key, val in kwargs['dynamic_shape'].items():
                profile.set_shape(key, val[0], val[1], val[2])
            builder_config.add_optimization_profile(profile)
            if calib is not None and self.builder.platform_has_fast_int8 is True:
                profile_calib = self.builder.create_optimization_profile()
                for key, val in kwargs['dynamic_shape'].items():
                    profile_calib.set_shape(key, val[1], val[1], val[1])
                builder_config.set_calibration_profile(profile_calib)

        for layer in self.network:
            print(layer.name)

        print(self.network[-1].get_output(0).shape)

        print('platform support FP16', self.builder.platform_has_fast_fp16)
        print('platform support INT8', self.builder.platform_has_fast_int8)
        if calib is not None and self.builder.platform_has_fast_int8 is True:
            self.builder.int8_mode = True
            self.builder.int8_calibrator = calib
            builder_config.set_flag(trt.BuilderFlag.INT8)
            builder_config.int8_calibrator = calib
            print('build engine with INT8')
        if self.builder.platform_has_fast_fp16 is True and build_fp16 is True:
            # self.builder.fp16_mode = True
            builder_config.set_flag(trt.BuilderFlag.TF32)
            builder_config.set_flag(trt.BuilderFlag.FP16)
            print('build engine with FP16')

        if use_DLA is True:
            builder_config.default_device_type = trt.DeviceType.DLA
            builder_config.DLA_core = 0
            for layer in self.network:
                if builder_config.can_run_on_DLA(layer):
                    builder_config.set_device_type(layer, trt.DeviceType.DLA)
                else:
                    builder_config.set_device_type(layer, trt.DeviceType.GPU)

        trt_engine = self.builder.build_engine(self.network,builder_config)

        if trt_engine is None:
            raise RuntimeError("Failed to build TensorRT engine from network")

        trt_engine = self._serialize_deserialize(trt_engine, trt_path)

    def run(self, inputs, **kwargs):
        return self.engine.run(inputs)

    def _serialize_deserialize(self, trt_engine, trt_path):
        self.runtime = trt.Runtime(TRT_LOGGER)
        serialized_engine = trt_engine.serialize()
        with open(trt_path, "wb") as f:
            f.write(serialized_engine)
        del self.parser # Parser no longer needed for ownership of plugins
        trt_engine = self.runtime.deserialize_cuda_engine(
                serialized_engine)
        return trt_engine