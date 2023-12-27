from ..can_interface import CANInterface
from util.common_util import run_cmd

class BaseCANDriver(CANInterface):
    def __init__(self, logger, baud=500000, device='can0'):
        super().__init__(logger=logger, baud=baud, device=device)
        self.bus = None

    def __del__(self):
        # do not link down the device when deconstruct (need more device manage policy)
        # self.teardown()
        pass

    def get_bus(self):
        return None

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
        self.logger.error("Base CAN is not implemented")

    def set_tx_queue_size(self, size):
        cmd = 'echo {} > /sys/class/net/{}/tx_queue_len'.format(size, self.device)
        run_cmd(cmd)

    def recv(self):
        raise NotImplementedError