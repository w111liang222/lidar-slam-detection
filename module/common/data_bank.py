from ..export_interface import register_interface

class DataBank():
    def __init__(self, cfg, logger, system):
        self.logger = logger
        self.frame_data = dict()
        register_interface('bank.get_frame_data', self.get_frame_data)

    def set_frame_data(self, data):
        self.frame_data = data

    def get_frame_data(self):
        return self.frame_data
