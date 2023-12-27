class CANInterface():
    def __init__(self, logger, baud=500000, device=None):
        self.logger = logger
        self.baud = baud
        self.device = device

    def __del__(self):
        pass

    def setup(self):
        raise NotImplementedError

    def teardown(self):
        raise NotImplementedError

    def set_baud(self, baud):
        raise NotImplementedError

    def send(self, id, data, is_extended_id = True):
        raise NotImplementedError

    def set_tx_queue_size(self, size):
        pass

    def recv(self):
        raise NotImplementedError