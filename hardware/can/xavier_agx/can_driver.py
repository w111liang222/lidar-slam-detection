import os

from ..can_interface import CANInterface
from util.common_util import run_cmd

_Global_Flag = '/tmp/load_can_success'

def load_can_driver():
    run_cmd('busybox devmem 0x0c303000 32 0x0000C400')
    run_cmd('busybox devmem 0x0c303008 32 0x0000C458')
    run_cmd('busybox devmem 0x0c303010 32 0x0000C400')
    run_cmd('busybox devmem 0x0c303018 32 0x0000C458')
    run_cmd('modprobe can')
    run_cmd('modprobe can_raw')
    run_cmd('modprobe mttcan')
    run_cmd('touch ' + _Global_Flag)

class AgxCANDriver(CANInterface):
    def __init__(self, logger, baud=500000, device='can0'):
        super().__init__(logger=logger, baud=baud, device=device)
        if not os.path.exists(_Global_Flag):
            load_can_driver()
        self.bus = None

    def __del__(self):
        # do not link down the device when deconstruct (need more device manage policy)
        # self.teardown()
        pass

    def get_bus(self):
        import can
        if self.bus is None:
            try:
                self.bus = can.interface.Bus(bustype='socketcan', channel=self.device, baud=self.baud)
            except Exception as e:
                self.logger.error(e)
                self.bus = None
        return self.bus

    def setup(self):
        cmd = 'ip link set up {}'.format(self.device)
        run_cmd(cmd)
        self.logger.info('%s link set up' % (self.device))

    def teardown(self):
        cmd = 'ip link set down {}'.format(self.device)
        run_cmd(cmd)
        self.logger.info('%s link set down' % (self.device))

    def set_baud(self, baud):
        self.teardown()
        self.baud = baud
        cmd = 'ip link set {} type can bitrate {}'.format(self.device, self.baud)
        run_cmd(cmd)
        self.logger.info('%s baud set to %d' % (self.device, baud))
        self.setup()

    def send(self, id, data, is_extended_id = True):
        import can
        self.get_bus()
        msg = can.Message(arbitration_id=id, data=data, is_extended_id=is_extended_id)
        try:
            self.bus.send(msg)
        except Exception as e:
            self.logger.error(e)

    def set_tx_queue_size(self, size):
        cmd = 'echo {} > /sys/class/net/{}/tx_queue_len'.format(size, self.device)
        run_cmd(cmd)

    def recv(self):
        raise NotImplementedError