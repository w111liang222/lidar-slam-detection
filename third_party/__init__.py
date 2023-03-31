from hardware.platform_common import BOARD_NAME, is_jetson

if is_jetson():
    from .aarch64 import *
elif BOARD_NAME == "IPC":
    from .x86_64 import *
