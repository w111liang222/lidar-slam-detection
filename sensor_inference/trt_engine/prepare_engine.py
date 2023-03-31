from hardware.platform_common import JETPACK

if JETPACK == "4.4.1":
    from .TensorRT7_1 import prepare_engine

elif JETPACK == "4.6.1":
    from .TensorRT8_2 import prepare_engine

elif JETPACK == "5.0.2":
    from .TensorRT8_4 import prepare_engine

PrepareBackend = prepare_engine.PrepareBackend