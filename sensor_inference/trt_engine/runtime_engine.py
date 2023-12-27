
try:
    import tensorrt
    from .TensorRT8 import runtime_engine
except Exception as e:
    from . import dummy_engine as runtime_engine
    print("WARN: TensorRT engine is unavailable")

RuntimeBackend = runtime_engine.RuntimeBackend