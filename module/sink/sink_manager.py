from ..manager_template import ManagerTemplate
from ..common.data_bank import DataBank

from .can_sink import CanSink
from .udp_sink import UdpSink
from .frame_sink import FrameSink
from .http_sink import HttpSink
# from .dump_sink import DumpSink

__all__ = {
    'UdpSink': UdpSink,
    'CanSink': CanSink,
    'HttpSink': HttpSink,
    'FrameSink': FrameSink,
    # 'DumpSink': DumpSink,
}

class SinkManager(ManagerTemplate):
    def __init__(self, cfg, logger, system):
        super().__init__('Sink', cfg, logger, system)
        self.data_bank = DataBank(cfg, logger, system)

        self.register = ['UdpSink', 'CanSink', 'HttpSink', 'FrameSink']

    def setup(self, cfg):
        self.cfg = cfg

        self.sink_dict = dict()
        for sink in self.register:
            self.sink_dict[sink] = __all__[sink](cfg, self.logger)

    def release(self):
        for name, sink in self.sink_dict.items():
            sink.stop()

    def start(self):
        pass

    def stop(self):
        pass

    def is_init_done(self):
        return True

    def set_config(self, cfg):
        self.cfg = cfg
        for name, sink in self.sink_dict.items():
            sink.set_config(cfg)

    def try_enqueue(self):
        return True

    def enqueue(self, data_dict, module_name):
        self.data_bank.set_frame_data(data_dict)
        for name, sink in self.sink_dict.items():
            sink.enqueue(data_dict.copy())