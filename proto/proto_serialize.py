import time
from . import detection_pb2, internal_pb2
import numpy as np
from util.box_utils import DetectionDrawer
from util.common_util import encoder_image_jpeg
from sensor_driver.common_lib import cpp_utils

def serialize_to_string(result_dict, display_on_image=False, use_raw_image=False, sample_size=0):
    det = detection_pb2.Detection()
    det.header.version = 'V1.0'.encode()

    if 'frame_fps' in result_dict:
        det.header.fps = result_dict['frame_fps']

    # 3D object
    if 'pred_boxes' in result_dict and 'pred_attr' in result_dict and 'pred_traj' in result_dict:
        pred_boxes, pred_attr, pred_traj = result_dict["pred_boxes"], result_dict["pred_attr"], result_dict["pred_traj"]
        for box, attr, traj in zip(pred_boxes, pred_attr, pred_traj):
            pred_object = det.object.add()
            pred_object.id = int(attr[4])
            pred_object.type = int(attr[6])
            pred_object.confidence = attr[5]
            pred_object.box.center.x = box[0]
            pred_object.box.center.y = box[1]
            pred_object.box.center.z = box[2]
            pred_object.box.length = box[3]
            pred_object.box.width = box[4]
            pred_object.box.height = box[5]
            pred_object.box.heading = box[6]
            pred_object.velocity_x = attr[0]
            pred_object.velocity_y = attr[1]
            pred_object.angle_rate = attr[2]
            pred_object.accel_x = attr[3]
            pred_object.valid = bool(attr[8])
            pred_object.status = int(attr[9])
            pred_object.age = int(attr[7])
            if traj[0][-1] <= 0: # no prediction trajectory
                continue
            for sample in traj:
                trajectory = pred_object.trajectory.add()
                trajectory.x = sample[0]
                trajectory.y = sample[1]
                trajectory.z = sample[2]
                trajectory.heading = sample[3]
                trajectory.velocity_x = sample[4]
                trajectory.velocity_y = sample[5]
                trajectory.relative_timestamp = int(sample[6])

    # freespace
    if 'freespace' in result_dict:
        det.freespace = result_dict['freespace']

    #traffic_light
    if 'trafficlight' in result_dict:
        pred_ids, pred_scores, pred_pictograms, pred_colors, pred_names = result_dict['trafficlight']['pred_ids'], result_dict['trafficlight']['pred_scores'], result_dict['trafficlight']['pred_pictograms'], result_dict['trafficlight']['pred_colors'], result_dict['trafficlight']['pred_names']
        for id, score, pictogram, color, name in zip(pred_ids, pred_scores, pred_pictograms, pred_colors, pred_names):
            trafficlight = det.light.add()
            trafficlight.id = int(id)
            trafficlight.pictogram = int(pictogram)
            trafficlight.color = int(color)
            trafficlight.confidence = score
            trafficlight.name = name

    # points
    if result_dict['lidar_valid']:
        points = np.concatenate(list(result_dict['points'].values()), axis=0)
        points = cpp_utils.pointcloud_downsample(points, sample_size) if sample_size > 0 else points
        det.points = points.tobytes()
    else:
        det.points = np.zeros((0, 4), dtype=np.float32).tobytes()

    # image
    if result_dict['image_valid']:
        if use_raw_image is True or display_on_image is True or 'image_jpeg' not in result_dict:
            if display_on_image is True and 'pred_boxes' in result_dict and 'pred_attr' in result_dict:
                # draw detection results
                for name, img in result_dict['image'].copy().items():
                    result_dict['image'][name] = DetectionDrawer.draw_boxes(result_dict['image'][name],
                                                                            result_dict['image_param'][name],
                                                                            result_dict["pred_boxes"],
                                                                            result_dict["pred_attr"][:, 6])

            if display_on_image is True and 'trafficlight' in result_dict:
                result_dict['image'] = DetectionDrawer.draw_boxes_2D(result_dict['image'],
                                                                     result_dict['image_param'],
                                                                     result_dict['trafficlight'])

            images = result_dict['image'].copy()
            images = encoder_image_jpeg(images)
        else:
            images = result_dict['image_jpeg'].copy()
            for name, img in images.items():
                images[name] = img if isinstance(img, bytes) else img.tobytes()

        for name, img in images.items():
            camera_image = det.image.add()
            camera_image.camera_name = name
            camera_image.image = img

    # radar
    if result_dict['radar_valid']:
        for name, radar in result_dict['radar'].items():
            radar_msg = det.radar.add()
            radar_msg.radar_name = name
            radar_boxes, radar_ids, radar_types, radar_velo = radar["radar_boxes"], radar["radar_ids"], radar["radar_types"], radar["radar_velo"]
            for boxes, ids, types, velo in zip(radar_boxes, radar_ids, radar_types, radar_velo):
                radar_object = radar_msg.radar_object.add()
                radar_object.id = int(ids)
                radar_object.type = int(types)
                radar_object.confidence = 1.0
                radar_object.box.center.x = boxes[0]
                radar_object.box.center.y = boxes[1]
                radar_object.box.center.z = boxes[2]
                radar_object.box.length = boxes[3]
                radar_object.box.width = boxes[4]
                radar_object.box.height = boxes[5]
                radar_object.box.heading = boxes[6]
                radar_object.velocity_x = velo[0]
                radar_object.velocity_y = velo[1]
                radar_object.angle_rate = 0
                radar_object.accel_x = 0
                radar_object.valid = True
                radar_object.status = 0
                radar_object.age = 1

    # pose
    if 'pose' in result_dict and result_dict['slam_valid']:
        pose_dict = result_dict['pose']
        [x, y, z, roll, pitch, yaw] = cpp_utils.get_cfg_from_transform(pose_dict['odom_matrix'])
        if abs(roll) >= 90.0 or abs(pitch) >= 90:
            [x, y, z, roll, pitch, yaw] = cpp_utils.get_cfg_from_transform(cpp_utils.get_transform_from_cfg(x, y, z, roll, pitch, -yaw))
        else:
            yaw = -yaw
        det.pose.x = x
        det.pose.y = y
        det.pose.z = z
        det.pose.heading = yaw
        det.pose.pitch = pitch
        det.pose.roll = roll
        det.pose.latitude = pose_dict['latitude']
        det.pose.longitude = pose_dict['longitude']
        det.pose.altitude = pose_dict['altitude']
        det.pose.status = int(pose_dict['Status'])
        det.pose.state = pose_dict['state']
        if pose_dict['area'] is not None:
            det.pose.area.type = pose_dict['area']['type']
            det.pose.area.name = pose_dict['area']['name']
    elif result_dict['ins_valid']:
        det.pose.x = 0
        det.pose.y = 0
        det.pose.z = 0
        det.pose.heading = result_dict['ins_data']['heading']
        det.pose.pitch = result_dict['ins_data']['pitch']
        det.pose.roll = result_dict['ins_data']['roll']
        det.pose.latitude = result_dict['ins_data']['latitude']
        det.pose.longitude = result_dict['ins_data']['longitude']
        det.pose.altitude = result_dict['ins_data']['altitude']
        det.pose.status = int(result_dict['ins_data']['Status'])
        det.pose.state = result_dict['ins_data']['state'] if 'state' in result_dict['ins_data'] else "Unknown"

    # timestamp
    det.header.timestamp = result_dict['frame_start_timestamp']
    det.header.relative_timestamp = abs(int(time.time() * 1000000) - result_dict['frame_start_timestamp'])

    return det.SerializeToString()

def serialize_raw_to_string(result_dict):
    lidar_map = internal_pb2.LidarPointcloudMap()
    points = result_dict['points'] if 'points' in result_dict else {}
    for name, point in points.items():
        lidar = lidar_map.lp.add()
        lidar.lidar_name = name[0]
        lidar.points = point.tobytes()

    return lidar_map.SerializeToString()