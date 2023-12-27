from hardware.platform_common import BOARD_NAME
from .base.can_driver import BaseCANDriver

__all__ = {
    'Base': BaseCANDriver,
}

def build_can(baud, device, logger):
    driver = __all__[BOARD_NAME] if BOARD_NAME in __all__ else __all__['Base']
    can_device = driver(logger=logger, baud=baud, device=device)
    return can_device
