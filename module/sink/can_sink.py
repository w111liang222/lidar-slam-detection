import time
import math
from .sink_template import SinkTemplate
from hardware.can import build_can

def quantization_status(num_obstacles, timestamp):
    timestamp = int(timestamp / 1000) % 256
    relative_timestamp = int(time.time() * 1000) % 256

    application_version = 1
    protocol_version = 1
    close_car = 0
    close_car = close_car & 0x01

    can_status = dict()
    can_status['num_obstacles'] = num_obstacles
    can_status['timestamp'] = timestamp
    can_status['relative_timestamp'] = relative_timestamp
    can_status['application_version'] = application_version
    can_status['protocol_version'] = protocol_version
    can_status['close_car'] = close_car
    return can_status

def quantization_obstacle_a(obj_id, obj_x, obj_y, obj_z, vel_x, obj_type, state, valid):
    ids = int(obj_id % 256)
    boxes_x = obj_x
    boxes_x = min(boxes_x, 127.93)
    boxes_x = max(boxes_x, -127.93)
    boxes_x = int(round(boxes_x / 0.0625))
    boxes_y = obj_y
    boxes_y = min(boxes_y, 127.93)
    boxes_y = max(boxes_y, -127.93)
    boxes_y = int(round(boxes_y / 0.0625))
    boxes_z = obj_z
    boxes_z = min(boxes_z, 7.93)
    boxes_z = max(boxes_z, -7.93)
    boxes_z = int(round(boxes_z / 0.0625))
    velo_x = vel_x
    velo_x = min(velo_x, 127.93)
    velo_x = max(velo_x, -127.93)
    velo_x = int(round(velo_x / 0.0625))
    labels = int(obj_type)
    labels = min(labels, 7)
    labels = max(labels, 0)
    obstacle_state = int(state)
    obstacle_state = min(labels, 3)
    obstacle_state = max(labels, 0)
    if bool(valid) is True:
        obstacle_valid = 1
    else:
        obstacle_valid = 2

    boxes_x_lowbyte = boxes_x & 0x00ff
    boxes_x_hightbyte = (boxes_x & 0x0f00) >> 8
    boxes_y_lowbyte = boxes_y & 0x000f
    boxes_y_hightbyte = (boxes_y & 0x0ff0) >> 4
    boxes_xy = (boxes_y_lowbyte << 4) | boxes_x_hightbyte
    boxes_z = boxes_z & 0xff
    velo_x_lowbyte = velo_x & 0x00ff
    velo_x_hightbyte = (velo_x & 0x0f00) >> 8
    labels_byte = labels & 0x07
    labels_velo_x = (labels_byte << 5) | velo_x_hightbyte
    obstacle_valid = obstacle_valid & 0x03
    obstacle_state = obstacle_state & 0x07
    valid_state = (obstacle_valid << 5) | obstacle_state

    can_data_a = dict()
    can_data_a['ids_byte'] = ids.to_bytes(1, 'big', signed=False)
    can_data_a['boxes_x_lowbyte'] = boxes_x_lowbyte.to_bytes(1, 'big', signed=False)
    can_data_a['boxes_xy'] = boxes_xy.to_bytes(1, 'big', signed=False)
    can_data_a['boxes_y_hightbyte'] = boxes_y_hightbyte.to_bytes(1, 'big', signed=False)
    can_data_a['boxes_z_byte'] = boxes_z.to_bytes(1, 'big', signed=False)
    can_data_a['velo_x_lowbyte'] = velo_x_lowbyte.to_bytes(1, 'big', signed=False)
    can_data_a['labels_velo_x'] = labels_velo_x.to_bytes(1, 'big', signed=False)
    can_data_a['valid_state'] = valid_state.to_bytes(1, 'big', signed=False)
    return can_data_a

def quantization_obstacle_b(obj_l, obj_w, obj_h, conf, age):
    boxes_l = obj_l
    boxes_l = min(boxes_l, 30.6)
    boxes_l = max(boxes_l, 0)
    boxes_l = int(round(boxes_l / 0.12))
    boxes_w = obj_w
    boxes_w = min(boxes_w, 12.75)
    boxes_w = max(boxes_w, 0)
    boxes_w= int(round(boxes_w / 0.05))
    boxes_h = obj_h
    boxes_h = min(boxes_h, 12.75)
    boxes_h = max(boxes_h, 0)
    boxes_h = int(round(boxes_h / 0.05))
    obstacle_age = int(age)
    obstacle_age = min(obstacle_age, 255)
    obstacle_age = max(obstacle_age, 0)
    scores = int(conf * 100)
    scores = min(scores, 100)
    scores = max(scores, 0)

    can_data_b = dict()
    can_data_b['boxes_l_byte'] = boxes_l.to_bytes(1, 'big', signed=False)
    can_data_b['boxes_w_byte'] = boxes_w.to_bytes(1, 'big', signed=False)
    can_data_b['boxes_h_byte'] = boxes_h.to_bytes(1, 'big', signed=False)
    can_data_b['obstacle_age'] = obstacle_age.to_bytes(1, 'big', signed=False)
    can_data_b['scores_byte'] = scores.to_bytes(1, 'big', signed=False)
    return can_data_b


def quantization_obstacle_c(obj_hd, angle_rate, accel_x):
    obstalce_angle_rate = angle_rate / math.pi * 180
    obstalce_angle_rate = min(obstalce_angle_rate, 327.67)
    obstalce_angle_rate = max(obstalce_angle_rate, -327.68)
    obstalce_angle_rate = int(round(obstalce_angle_rate / 0.01))
    object_accel_x = accel_x
    object_accel_x = min(object_accel_x, 14.97)
    object_accel_x = max(object_accel_x, -14.97)
    object_accel_x = int(round(object_accel_x / 0.03))
    obstacle_replaced = 0
    obstacle_replaced = min(obstacle_replaced, 1)
    obstacle_replaced = max(obstacle_replaced, 0)
    boxes_hd = obj_hd / math.pi * 180
    if boxes_hd > 180:
        boxes_hd = boxes_hd - 360
    elif boxes_hd < -180:
        boxes_hd = boxes_hd + 360
    boxes_hd = min(boxes_hd, 327.67)
    boxes_hd = max(boxes_hd, -327.68)
    boxes_hd = int(round(boxes_hd / 0.01))

    obstalce_angle_rate_lowbyte = obstalce_angle_rate & 0x00ff
    obstalce_angle_rate_hightbyte = (obstalce_angle_rate & 0xff00) >> 8
    object_accel_x_lowbyte = object_accel_x & 0x00ff
    object_accel_x_hightbyte = (object_accel_x >> 8) & 0x03
    obstacle_replaced = obstacle_replaced & 0x01
    obstacle_replaced_accel_x = (obstacle_replaced << 7) | object_accel_x_hightbyte
    boxes_hd_lowbyte = boxes_hd & 0x00ff
    boxes_hd_hightbyte = (boxes_hd & 0xff00) >> 8

    can_data_c = dict()
    can_data_c['obstalce_angle_rate_lowbyte'] =  obstalce_angle_rate_lowbyte.to_bytes(1, 'big', signed=False)
    can_data_c['obstalce_angle_rate_hightbyte'] =  obstalce_angle_rate_hightbyte.to_bytes(1, 'big', signed=False)
    can_data_c['object_accel_x_lowbyte'] = object_accel_x_lowbyte.to_bytes(1, 'big', signed=False)
    can_data_c['obstacle_replaced_accel_x'] = obstacle_replaced_accel_x.to_bytes(1, 'big', signed=False)
    can_data_c['boxes_hd_lowbyte'] =  boxes_hd_lowbyte.to_bytes(1, 'big', signed=False)
    can_data_c['boxes_hd_hightbyte'] =  boxes_hd_hightbyte.to_bytes(1, 'big', signed=False)
    return can_data_c

class CanSink(SinkTemplate):
    def __init__(self, cfg, logger):
        super().__init__(name='CanSink', cfg=cfg, logger=logger)

        if cfg.output.protocol.CAN.use:
            self.start()

    def prepare_data(self, data_dict):
        if 'pred_boxes' not in data_dict or 'pred_attr' not in data_dict:
            return dict()

        return {'frame_start_timestamp': data_dict['frame_start_timestamp'],
                'pred_boxes': data_dict['pred_boxes'],
                'pred_attr': data_dict['pred_attr']}

    def prepare_run(self):
        can_cfg = self.cfg.output.protocol.CAN
        self.can_device = build_can(can_cfg.baud, can_cfg.device, logger=self.logger)
        self.can_device.set_baud(can_cfg.baud)
        self.can_device.set_tx_queue_size(40 * 1024)
        self.can_device.setup()

    def sink(self, data_dict):
        if 'pred_boxes' not in data_dict or 'pred_attr' not in data_dict:
            return

        pred_boxes, pred_attr = data_dict["pred_boxes"], data_dict["pred_attr"]

        can_status = quantization_status(pred_boxes.shape[0], data_dict['frame_start_timestamp'])
        self.send_status(can_status)

        i = 0
        for box, attr in zip(pred_boxes, pred_attr):
            can_data_a = quantization_obstacle_a(attr[4], box[0], box[1], box[2], attr[0], attr[6], attr[9], attr[8])
            can_data_b = quantization_obstacle_b(box[3], box[4], box[5], attr[5], attr[7])
            can_data_c = quantization_obstacle_c(box[6], attr[2], attr[3])
            self.send_one(i, can_data_a, can_data_b, can_data_c)
            i = i + 1

    def send_status(self, can_status):
        num_obstacles = can_status['num_obstacles']
        timestamp = can_status['timestamp']
        relative_timestamp = can_status['relative_timestamp']
        application_version = can_status['application_version']
        protocol_version = can_status['protocol_version']
        close_car = can_status['close_car']

        num_obstacles_byte = num_obstacles.to_bytes(1, 'big', signed=False)
        timestamp_byte = timestamp.to_bytes(1, 'big', signed=False)
        relative_timestamp_byte = relative_timestamp.to_bytes(1, 'big', signed=False)
        application_version_byte = application_version.to_bytes(1, 'big', signed=False)
        protocol_version_byte = protocol_version.to_bytes(1, 'big', signed=False)
        close_car_byte = close_car.to_bytes(1, 'big', signed=False)

        status = num_obstacles_byte + timestamp_byte + relative_timestamp_byte + application_version_byte \
            + protocol_version_byte + close_car_byte + bytes([0x00]) + bytes([0x00])
        self.can_device.send(id=0x568, data=status)

    def send_one(self, i, can_data_a, can_data_b, can_data_c):
        data_a = can_data_a['ids_byte'] + can_data_a['boxes_x_lowbyte'] + can_data_a['boxes_xy'] + can_data_a['boxes_y_hightbyte'] \
            + can_data_a['boxes_z_byte'] + can_data_a['velo_x_lowbyte'] + can_data_a['labels_velo_x'] + can_data_a['valid_state']
        data_b = can_data_b['boxes_l_byte'] + can_data_b['boxes_w_byte'] + can_data_b['boxes_h_byte'] + can_data_b['obstacle_age'] \
            + can_data_b['scores_byte'] + bytes([0x00]) + bytes([0x00]) + bytes([0x00])
        data_c = can_data_c['obstalce_angle_rate_lowbyte'] + can_data_c['obstalce_angle_rate_hightbyte'] + bytes([0x00]) + bytes([0x00]) \
            + can_data_c['object_accel_x_lowbyte'] + can_data_c['obstacle_replaced_accel_x'] + can_data_c['boxes_hd_lowbyte'] + can_data_c['boxes_hd_hightbyte']

        self.can_device.send(id=0x569 + i*3, data=data_a)
        self.can_device.send(id=0x56A + i*3, data=data_b)
        self.can_device.send(id=0x56B + i*3, data=data_c)
