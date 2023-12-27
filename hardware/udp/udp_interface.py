class UDPInterface():
    def __init__(self, logger, dest_ip, dest_port):
        self.logger = logger

    def __del__(self):
        pass

    def setup(self):
        raise NotImplementedError

    def teardown(self):
        raise NotImplementedError

    def set_dest(self, dest_ip, dest_port):
        raise NotImplementedError

    def send(self, id, data):
        raise NotImplementedError

    def recv(self):
        raise NotImplementedError