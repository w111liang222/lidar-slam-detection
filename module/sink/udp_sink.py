import time
from .sink_template import SinkTemplate
from proto.proto_serialize import serialize_to_string
from hardware.udp import build_udp

class UdpSink(SinkTemplate):
    def __init__(self, cfg, logger):
        super().__init__('UdpSink', cfg, logger)

        dest_ip = cfg.output.protocol.UDP.destination
        dest_port = cfg.output.protocol.UDP.port
        self.udp_device = build_udp('Linux', dest_ip, dest_port, logger)

        if cfg.output.protocol.UDP.use:
            self.start()

    def set_config(self, cfg):
        old_cfg = self.cfg
        if old_cfg.output.protocol.UDP != cfg.output.protocol.UDP:
            if cfg.output.protocol.UDP.use:
                self.set_destination(cfg.output.protocol.UDP.destination, cfg.output.protocol.UDP.port)
                self.start()
            else:
                self.stop()
        self.cfg = cfg

    def set_destination(self, dest_ip, dest_port):
        self.udp_device.set_dest(dest_ip, dest_port)
        if self.is_start.value:
            self.stop()
            self.start()

    def prepare_data(self, data_dict):
        data_dict.pop('imu_data', None)
        data_dict.pop('motion_t', None)
        data_dict.pop('motion_heading', None)
        data_dict.pop('points', None)
        data_dict.pop('points_attr', None)
        data_dict.pop('image', None)
        data_dict.pop('image_jpeg', None)
        data_dict.pop('image_param', None)
        data_dict['lidar_valid'] = False
        data_dict['image_valid'] = False
        return data_dict

    def sink(self, data_dict):
        data = serialize_to_string(data_dict)
        self.logger.debug(f'''pipeline latency: {(time.time() * 1000000 - data_dict['frame_start_timestamp']) / 1000.0:.1f} ms''')
        self.udp_device.send(data)
