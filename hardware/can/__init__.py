from hardware.platform_common import BOARD_NAME
from .xavier_nx.can_driver import NxCANDriver
from .xavier_agx.can_driver import AgxCANDriver
from .agx_orin_ad10.can_driver import AgxOrinAD10CANDriver
from .ipc.can_driver import IPCCANDriver

__all__ = {
    'Xavier-NX':  NxCANDriver,
    'Xavier-AGX': AgxCANDriver,
    'AGX-Orin-32GB': AgxOrinAD10CANDriver,
    'IPC': IPCCANDriver,
}

def build_can(baud, device, logger):
    can_device = __all__[BOARD_NAME](logger=logger, baud=baud, device=device)
    return can_device
