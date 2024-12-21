import queue
import collections
import numpy as np
import cv2
from flask import request
from flask.helpers import make_response
from zerocm import ZCM

from sensor_driver.common_lib import cpp_utils
from sensor_driver.common_lib.logging.message import *
from proto import internal_pb2

TypeDict = dict(
    Byte            = std_msgs.Byte,
    Float           = std_msgs.Float,
    Int32           = std_msgs.Int32,
    Int64           = std_msgs.Int64,
    String          = std_msgs.String,
    Odometry        = nav_msgs.Odometry,
    Path            = nav_msgs.Path,
    Image           = sensor_msgs.Image,
    CompressedImage = sensor_msgs.CompressedImage,
    Imu             = sensor_msgs.Imu,
    NavSatFix       = sensor_msgs.NavSatFix,
    PointCloud      = sensor_msgs.PointCloud,
)

class MessageServer():
    def __init__(self, app):
        self.app = app
        self.zcm = ZCM("ipc://zcm_core")
        self.zcm.setQueueSize(100)
        self.subscription = None
        self.messages = collections.OrderedDict()
        self.message_types = collections.OrderedDict()

    def setup(self, perception):
        self.perception = perception
        self.app.add_url_rule("/v1/start-message-subscribe", view_func=self.start_subscription, methods=["GET"])
        self.app.add_url_rule("/v1/stop-message-subscribe",  view_func=self.stop_subscription,  methods=["GET"])
        self.app.add_url_rule("/v1/get-message-meta", view_func=self.get_message_meta,  methods=["GET"])
        self.app.add_url_rule("/v1/get-message-data", view_func=self.get_message_data, methods=["POST"])
        self.app.add_url_rule("/v1/publish-message", view_func=self.publish_message, methods=["POST"])

    def start_subscription(self):
        if self.subscription is not None:
            return ""
        self.perception.set_runtime_config({"ipc_enable": True})
        self.messages = collections.OrderedDict()
        self.message_types = collections.OrderedDict()
        self.subscription = self.zcm.subscribe_raw(".*", self.handler)
        self.zcm.start()
        return "ok"

    def stop_subscription(self):
        if self.subscription is None:
            return ""
        self.zcm.stop()
        self.zcm.unsubscribe(self.subscription)
        self.subscription = None
        return "ok"

    def get_message_meta(self):
        self.start_subscription()
        return dict(
            channels = list(self.messages.keys()),
            types    = list(self.message_types.values()),
        )

    def get_message_data(self):
        name = request.get_json()['name']
        if name not in self.messages or self.message_types[name] == "":
            return ""

        try:
            data = self.messages[name].get(block=False)
        except queue.Empty:
            return ""

        type_name = self.message_types[name]
        msg = TypeDict[type_name].decode(data)

        if type_name in ["Byte", "Float", "Int32", "Int64", "String"]:
            return str(msg.data)
        elif type_name == "Odometry":
            return self.format_odometry(msg)
        elif type_name == "Path":
            return self.format_path(msg)
        elif type_name == "Image":
            return self.format_image(msg)
        elif type_name == "CompressedImage":
            return self.format_compressedimage(msg)
        elif type_name == "Imu":
            return self.format_imu(msg)
        elif type_name == "NavSatFix":
            return self.format_navsatfix(msg)
        elif type_name == "PointCloud":
            return self.format_pointcloud(msg)

        return ""

    def format_odometry(self, msg):
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        return dict(
            stamp       = msg.header.stamp,
            position    = [position.x, position.y, position.z],
            orientation = [orientation.x, orientation.y, orientation.z, orientation.w],
        )

    def format_path(self, msg):
        stamp, positions, orientations = [], [], []
        for pose in msg.poses:
            stamp.append(pose.header.stamp)
            position = pose.pose.position
            orientation = pose.pose.orientation

            positions.append([position.x, position.y, position.z])
            orientations.append([orientation.x, orientation.y, orientation.z, orientation.w])
        return dict(
            stamp       = msg.header.stamp,
            pose_stamp  = stamp,
            position    = positions,
            orientation = orientations,
        )

    def format_image(self, msg):
        if msg.encoding == "BGR8":
            img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        elif msg.encoding == "YUV_I420":
            img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width)
            img = cv2.cvtColor(img, cv2.COLOR_YUV2BGR_I420)
        _, data = cv2.imencode('.bmp', img)

        proto_image = internal_pb2.LidarPointcloudMap()
        image = proto_image.image.add()
        image.camera_name = str(msg.header.stamp)
        image.image =  data.tobytes()

        response = make_response(proto_image.SerializeToString())
        response.headers["Content-Type"] = "application/octet-stream"
        return response

    def format_compressedimage(self, msg):
        proto_image = internal_pb2.LidarPointcloudMap()
        image = proto_image.image.add()
        image.camera_name = str(msg.header.stamp)
        image.image =  msg.data

        response = make_response(proto_image.SerializeToString())
        response.headers["Content-Type"] = "application/octet-stream"
        return response

    def format_imu(self, msg):
        return dict(
            stamp  = msg.header.stamp,
            gyro_x = msg.angular_velocity.x,
            gyro_y = msg.angular_velocity.y,
            gyro_z = msg.angular_velocity.z,
            acc_x  = msg.linear_acceleration.x,
            acc_y  = msg.linear_acceleration.y,
            acc_z  = msg.linear_acceleration.z,
        )

    def format_navsatfix(self, msg):
        return dict(
            stamp     = msg.header.stamp,
            latitude  = msg.latitude,
            longitude = msg.longitude,
            altitude  = msg.altitude,
        )

    def format_pointcloud(self, msg):
        points      = np.frombuffer(msg.data, dtype=np.float32)
        points      = points.reshape(-1, int(points.shape[0] / (msg.height * msg.width)))
        attr        = points[:, 4] if points.shape[1] == 8 else points[:, 3]
        attr_name   = msg.fields[3].name
        if attr_name == "intensity":
            intensity_min, intensity_max = np.min(attr), np.max(attr)
            attr = (attr - intensity_min) / (intensity_max - intensity_min + 1e-4)

        # format proto
        proto_points = internal_pb2.LidarPointcloudMap()
        lidar = proto_points.lp.add()
        lidar.lidar_name = str(msg.header.stamp)
        lidar.points = points[:, :3].tobytes()
        lidar.attr = attr.tobytes()
        lidar.type = attr_name

        response = make_response(proto_points.SerializeToString())
        response.headers["Content-Type"] = "application/octet-stream"
        return response

    def handler(self, channel, data):
        if channel not in self.messages:
            self.messages[channel] = queue.Queue(maxsize=3)
            self.message_types[channel] = self.detect_message_type(data)

        if self.messages[channel].full():
            self.messages[channel].get_nowait()

        self.messages[channel].put_nowait(data)

    def detect_message_type(self, data):
        data_type = ""
        for name, T in TypeDict.items():
            try:
                msg = T.decode(data)
                data_type = name
                break
            except Exception as e:
                pass

        return data_type

    def publish_message(self):
        name    = request.get_json()['name']
        channel = name.split('/')[-1].split('.')[0]
        data    = cpp_utils.publish_message(name)
        self.handler(channel, data)
        return ""
