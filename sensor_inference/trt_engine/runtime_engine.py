from hardware.platform_common import JETPACK

if JETPACK == "4.4.1":
    from .TensorRT7_1 import runtime_engine

elif JETPACK == "4.6.1":
    from .TensorRT8_2 import runtime_engine

elif JETPACK == "5.0.2":
    from .TensorRT8_4 import runtime_engine

RuntimeBackend = runtime_engine.RuntimeBackend