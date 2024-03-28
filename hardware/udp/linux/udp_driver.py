from ..udp_interface import UDPInterface
import socket

class LinuxUdpDriver(UDPInterface):
    def __init__(self, logger, dest_ip, dest_port):
        super().__init__(logger=logger, dest_ip=dest_ip, dest_port=dest_port)
        self.dest = (dest_ip, dest_port)
        self.client = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)

    def __del__(self):
        pass

    def setup(self):
        pass

    def teardown(self):
        pass

    def set_dest(self, dest_ip, dest_port):
        self.dest = (dest_ip, dest_port)

    def send(self, data):
        try:
            self.client.sendto(data, self.dest)
        except Exception as e:
            self.logger.warn(f'{str(e)}, data size: {len(data)}')

    def recv(self):
        raise NotImplementedError