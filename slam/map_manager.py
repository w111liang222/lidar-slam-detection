import os
import cv2
import copy
import datetime
import numpy as np
from threading import Thread

import slam_wrapper as slam
from proto import internal_pb2
from sensor_driver.common_lib.cpp_utils import get_transform_from_RPYT
from util.common_util import do_mkdir, run_cmd
from util.image_util import cvt_image

def prepend(header, file):
    with open(file, "rb") as fr:
        data = fr.read()
        with open(file, "wb") as fw:
            fw.write(header)
            fw.write(data)
            fw.close()

class MapManager():
    def __init__(self, config, logger):
        self.config = config
        self.logger = logger

        self.data = dict(
            points = dict(), images = dict(), images_jpeg = dict(),
            poses  = dict(),  edges = dict(),
            stamps = dict(),
        )
        self.meta = dict(
            vertex = dict(), edge = dict(),
        )
        self.vertex_id = 0
        self.color_map_bytes = b""
        self.save_keyframe_idx = 0
        self.thread = None

    def deinit(self):
        if self.thread is not None:
            self.thread.join()
            self.thread = None

    def update(self, data, replace):
        if replace is True:
            self.data = data
        else:
            self.data['points'].update(data['points'])
            self.data['images'].update(data['images'])
            self.data['images_jpeg'].update(data['images_jpeg'])
            self.data['poses'].update(data['poses'])
            self.data['stamps'].update(data['stamps'])

        keyframe_ids = list(self.data['stamps'])
        self.vertex_id = max([int(id) for id in keyframe_ids]) + 1 if len(keyframe_ids) > 0 else 0

    def update_pose(self, pose, replace):
        if replace is True:
            self.data['poses'] = pose
        else:
            self.data['poses'].update(pose)

    def update_edge(self, edges):
        self.data['edges'] = edges

    def update_meta(self, meta):
        self.meta = meta
        for k, v in self.data['stamps'].items():
            self.meta["vertex"][k]["stamps"] = v

    def get_meta_data(self):
        return self.meta

    def get_color_map(self, points):
        MAX_SEGMENT_LEN = int(1e8)
        if points is not None:
            proto_points = internal_pb2.LidarPointcloudMap()
            lidar = proto_points.lp.add()
            lidar.lidar_name = "color_map"
            lidar.points = points[:, :3].tobytes()
            lidar.attr = points[:, 3].tobytes()
            lidar.type = "rgb"
            self.color_map_bytes = proto_points.SerializeToString()

        slice_len = min(MAX_SEGMENT_LEN, len(self.color_map_bytes))
        segment = self.color_map_bytes[:slice_len]
        self.color_map_bytes = self.color_map_bytes[slice_len:]
        return segment

    def get_pose(self):
        vertexs = self.data['poses'].copy()
        for k, v in vertexs.items():
            vertexs[k] = v.flatten().tolist()
        return vertexs

    def get_edge(self):
        return self.data['edges']

    def del_vertex(self, id):
        id_str = str(id)
        self.data['points'].pop(id_str)
        self.data['images'].pop(id_str)
        self.data['images_jpeg'].pop(id_str)
        self.data['poses'].pop(id_str)
        self.data['stamps'].pop(id_str)
        slam.del_graph_vertex(id)
        self.logger.info('delete vertex (%d) success' % (id))

    def del_points(self, index):
        for idx, pointIdx in index.items():
            self.data['points'][idx] = np.delete(self.data['points'][idx], np.array(pointIdx, dtype=np.int32), axis = 0)

    def add_edge(self, prev_id, next_id, relative):
        relative = np.array(relative).reshape(4, 4)
        slam.add_graph_edge(self.data['points'][str(prev_id)], prev_id,
                            self.data['points'][str(next_id)], next_id,
                            relative)
        self.logger.info('add edge (%d - %d) success' % (prev_id, next_id))

    def del_edge(self, id):
        slam.del_graph_edge(id)
        self.logger.info('delete edge (%d) success' % (id))

    def set_vertex_fix(self, id, fix):
        slam.set_graph_vertex_fix(id, fix)

    def get_key_frame(self, index, item):
        pointcloud = self.data['points'][index] if index in self.data['points'] else np.zeros((0, 4), dtype=np.float32)
        images = self.data['images_jpeg'][index] if index in self.data['images_jpeg'] else {}
        keyframe = internal_pb2.LidarPointcloudMap()
        # serialize to proto
        if "p" in item:
            lidar = keyframe.lp.add()
            lidar.lidar_name = index
            lidar.points = pointcloud.tobytes()
        if "i" in item:
            for name, img in images.items():
                camera_image = keyframe.image.add()
                camera_image.camera_name = name
                camera_image.image = img if isinstance(img, bytes) else img.tobytes()

        return keyframe.SerializeToString()

    def keyframe_align(self, source, target, guess):
        guess = np.array(guess).reshape(4, 4)
        return slam.pointcloud_align(self.data['points'][source], self.data['points'][target], guess).flatten().tolist()

    def set_export_map_config(self, z_min, z_max, color):
        slam.reset_map_points(z_min, z_max)
        ins_ext_param = self.config.ins.extrinsic_parameters
        ins_extrinic = get_transform_from_RPYT(ins_ext_param[0], ins_ext_param[1], ins_ext_param[2],
                                               ins_ext_param[5], ins_ext_param[4], ins_ext_param[3])
        for idx, stamp in self.data['stamps'].items():
            if color != "rgb":
                slam.merge_pcd(self.data['points'][idx], self.data['poses'][idx], False)
                continue

            points_rgb = []
            for camera_config in self.config.camera:
                if camera_config.name not in self.data["images_jpeg"][idx]:
                    continue
                if camera_config.name not in self.data["images"][idx]:
                    self.data["images"][idx][camera_config.name] = cv2.imdecode(self.data["images_jpeg"][idx][camera_config.name], cv2.IMREAD_COLOR)

                image = cvt_image(self.data["images"][idx][camera_config.name], camera_config.output_width, camera_config.output_height)
                fx, fy, cx, cy = camera_config.intrinsic_parameters[:4]
                cam_ext_param = camera_config.extrinsic_parameters
                cam_extrinic = get_transform_from_RPYT(cam_ext_param[0], cam_ext_param[1], cam_ext_param[2],
                                                       cam_ext_param[5], cam_ext_param[4], cam_ext_param[3])
                cam_extrinic = np.dot(cam_extrinic, np.linalg.inv(ins_extrinic))
                points = self.data['points'][idx]
                points_hom = np.hstack((points[:, :3], np.ones((points.shape[0], 1), dtype=np.float32)))
                points_in_cam = np.dot(cam_extrinic, points_hom.T).T[:, 0:3]
                depth_idx = points_in_cam[:, 2] > 0.1
                u = (points_in_cam[:, 0] / points_in_cam[:, 2]) * fx + cx
                v = (points_in_cam[:, 1] / points_in_cam[:, 2]) * fy + cy
                u_idx = np.bitwise_and(u <= camera_config.output_width  - 1, u >= 0)
                v_idx = np.bitwise_and(v <= camera_config.output_height - 1, v >= 0)
                uv_idx = np.bitwise_and(u_idx, v_idx)
                uv_idx = np.bitwise_and(uv_idx, depth_idx)
                u = np.rint(u[uv_idx]).astype(np.int32)
                v = np.rint(v[uv_idx]).astype(np.int32)
                point_rgb = image[v, u]
                points = np.concatenate([points[uv_idx, :3], point_rgb], axis = 1)
                points_rgb.append(points)

            if len(points_rgb) != 0:
                points_rgb = np.concatenate(points_rgb, axis = 0)
                slam.merge_pcd(points_rgb, self.data['poses'][idx], True)

    def export_map(self):
        slam.dump_merged_pcd('output/export_map.pcd')
        origin = slam.get_map_origin()
        header = '''# This PCD file is generated by TSARI IPU\n# GNSS Anchor {:.10f} {:.10f} {:.10f}\n'''.format(origin[0, 0], origin[0, 1], origin[0, 2])
        prepend(header.encode(), 'output/export_map.pcd')

    def add_key_frame(self, points, image, image_jpeg, pose, stamp):
        index = str(self.vertex_id)
        self.data['points'][index] = points
        self.data['images'][index] = image
        self.data['images_jpeg'][index] = image_jpeg
        self.data['poses'][index] = pose
        self.data['stamps'][index] = stamp
        self.vertex_id += 1

    def get_save_progress(self):
        return (self.save_keyframe_idx / (self.total_frame + 1)) * 100

    def start_save_mapping(self, root_path, name):
        if self.thread is not None:
            self.thread.join()
            self.thread = None

        self.total_frame = len(self.data['stamps'])
        if self.total_frame <= 0:
            return "error"
        # create map path
        file_path = root_path + '/' + (datetime.datetime.now().strftime('%Y-%m-%d-%H-%M-%S') if name is None else name)
        graph_path = file_path + '/graph'
        if not do_mkdir(graph_path):
            return "error"

        os.chmod(file_path, 0o777)
        os.chmod(graph_path, 0o777)
        self.logger.info('mkdir success: %s' % file_path)

        # save meta data
        np.savetxt(graph_path + '/map_info.txt', slam.get_map_origin(), fmt='%1.10f')
        os.chmod(graph_path + '/map_info.txt', 0o777)

        self.save_keyframe_idx = 0
        self.thread = Thread(target=self.saving_thread_loop,
                             args=(graph_path,
                                   copy.deepcopy(self.data['poses']),
                                   self.data['points'].copy(),
                                   self.data['images_jpeg'].copy(),
                                   copy.deepcopy(self.data['stamps']),),
                             daemon=True)
        self.thread.start()
        self.logger.info("start to save mapping")
        return "ok"

    def saving_thread_loop(self, file_path, pose, points, image, stamps):
        vertexes = slam.dump_graph(file_path)
        for idx, stamp in stamps.items():
            if idx not in vertexes:
                continue

            try:
                dir_path = file_path + '/{0:06d}'.format(self.save_keyframe_idx)
                do_mkdir(dir_path)
                slam.dump_keyframe(dir_path, stamps[idx], int(idx), points[idx], pose[idx])
                # write meta data and images
                meta_fd = open(dir_path + "/meta", "w")
                meta_fd.write("image {}".format(len(image[idx])))
                for name, img in image[idx].items():
                    meta_fd.write(" " + name)
                    image_fd = open(dir_path + "/{}.jpg".format(name), "wb")
                    image_fd.write(img if isinstance(img, bytes) else img.tobytes())
                meta_fd.write("\n")
            except Exception as e:
                self.logger.error(e)
                break
            self.save_keyframe_idx += 1

        run_cmd('sync')
        run_cmd('chmod a+w+r {} -R'.format(file_path))
        self.logger.info("finsih saving map, total %d / %d frames" % (self.save_keyframe_idx, self.total_frame))
        self.save_keyframe_idx = self.total_frame + 1