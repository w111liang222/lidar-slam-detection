from hardware.platform_common import MACHINE

if MACHINE == "aarch64":
    from .aarch64 import *
elif MACHINE == "x86_64":
    from .x86_64 import *
