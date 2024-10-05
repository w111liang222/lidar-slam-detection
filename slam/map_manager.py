import os
import copy
import datetime
import json
import yaml
import numpy as np
from pathlib import Path
from shapely.geometry import Point, Polygon
from threading import Thread

import slam_wrapper as slam
from proto import internal_pb2
from util.common_util import do_mkdir, run_cmd
from module.export_interface import call

def prepend(header, file):
    data = open(file, "rb").read() if os.path.exists(file) else b''
    with open(file, "wb") as fw:
        fw.write(header)
        fw.write(data)
        fw.close()

def glob_configs(map_path):
    map_path = Path(map_path).expanduser().absolute() / 'graph' / 'configs'
    config_list = list(map(str, map_path.glob('*.yaml')))
    config_list.sort()
    configs = [yaml.safe_load(Path(config_file).open()) for config_file in config_list]
    return configs

def sync_keys(source, target):
    for k, v in target.copy().items():
        if k not in source:
            target.pop(k)

def load_map_meta(file):
    if not os.path.exists(file):
        return { 'area': {} }
    return json.load(open(file, 'r'))

def dump_map_meta(file, meta):
    with open(file, 'w') as f:
        json.dump(meta, f, ensure_ascii=False, indent=4)

class MapManager():
    def __init__(self, config, logger):
        self.configs = [config]
        self.logger = logger

        self.data = dict(
            points = dict(), images = dict(),
            poses  = dict(), edges  = dict(),
            stamps = dict(),
        )
        self.meta = dict(
            vertex = dict(), edge = dict(), area = dict(),
        )
        self.area_polygon = {}
        self.vertex_id = 0
        self.color_map_bytes = b""
        self.save_keyframe_idx = 0
        self.thread = None

    def deinit(self):
        if self.thread is not None:
            self.thread.join()
            self.thread = None

    def on_open_map(self, map_path):
        self.configs = glob_configs(map_path)

    def on_merge_map(self, map_path):
        self.configs.extend(glob_configs(map_path))

    def update(self, data, meta, replace):
        if replace is True:
            self.data = data
            self.meta = meta
        else:
            self.data['points'].update(data['points'])
            self.data['images'].update(data['images'])
            self.data['poses'].update(data['poses'])
            self.data['stamps'].update(data['stamps'])
            self.meta['area'].update(meta['area'])
        self.update_area_polygon()

        keyframe_ids = list(self.data['stamps'])
        self.vertex_id = max([int(id) for id in keyframe_ids]) + 1 if len(keyframe_ids) > 0 else 0

    def update_pose(self, pose, replace):
        if replace is True:
            self.data['poses'] = pose
            sync_keys(self.data['poses'], self.data['points'])
            sync_keys(self.data['poses'], self.data['images'])
            sync_keys(self.data['poses'], self.data['stamps'])
        else:
            self.data['poses'].update(pose)

    def update_edge(self, edges):
        self.data['edges'] = edges

    def update_meta(self, meta):
        self.meta.update(meta)
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

    def add_area(self, area):
        ids = list(self.meta['area'])
        new_id = max([int(id) for id in ids]) + 1 if len(ids) > 0 else 0
        self.meta['area'][str(new_id)] = area
        self.update_area_polygon()
        self.logger.info('add area (%s) success' % (str(new_id)))

    def del_area(self, id):
        self.meta['area'].pop(id)
        self.update_area_polygon()
        self.logger.info('delete area (%s) success' % (id))

    def set_vertex_fix(self, id, fix):
        slam.set_graph_vertex_fix(id, fix)

    def get_key_frame(self, index, item):
        pointcloud = self.data['points'][index] if index in self.data['points'] else np.zeros((0, 4), dtype=np.float32)
        images = self.data['images'][index] if index in self.data['images'] else {}
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

    def update_area_polygon(self):
        self.area_polygon = {}
        for id, area in self.meta['area'].items():
            poly_pts = []
            for pts in area['polygon']:
                poly_pts.append(pts[:2])
            poly_pts.append(area['polygon'][0][:2])
            self.area_polygon[id] = Polygon(poly_pts)

    def is_in_area(self, pose):
        pt = Point(pose[0, 3], pose[1, 3])
        for id, area in self.area_polygon.items():
            if pt.within(area):
                return self.meta['area'][id]

        return None

    def set_export_map_config(self, z_min, z_max, color):
        slam.set_export_map_config(z_min, z_max, color)
        for idx, stamp in self.data['stamps'].items():
            slam.export_points(self.data['points'][idx], self.data['poses'][idx])

    def export_map(self):
        map_name = 'output/export_map.pcd'
        os.unlink(map_name)
        slam.dump_map_points(map_name)
        origin = slam.get_map_origin()
        header = '''# This PCD file is generated by LSD\n# GNSS Anchor {:.10f} {:.10f} {:.10f}\n'''.format(origin[0, 0], origin[0, 1], origin[0, 2])
        prepend(header.encode(), map_name)

    def add_key_frame(self, points, image, pose, stamp):
        index = str(self.vertex_id)
        self.data['points'][index] = points
        self.data['images'][index] = image
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
        config_path = graph_path + '/configs'
        if not do_mkdir(graph_path):
            return "error"

        if not do_mkdir(config_path):
            return "error"

        os.chmod(file_path, 0o777)
        os.chmod(graph_path, 0o777)
        os.chmod(config_path, 0o777)
        self.logger.info('mkdir success: %s' % file_path)

        # save meta data
        np.savetxt(graph_path + '/map_info.txt', slam.get_map_origin(), fmt='%1.10f')
        os.chmod(graph_path + '/map_info.txt', 0o777)

        self.save_keyframe_idx = 0
        self.thread = Thread(target=self.saving_thread_loop,
                             args=(graph_path,
                                   copy.deepcopy(self.data['poses']),
                                   self.data['points'].copy(),
                                   self.data['images'].copy(),
                                   copy.deepcopy(self.data['stamps']),),
                             daemon=True)
        self.thread.start()
        self.logger.info("start to save mapping")
        return "ok"

    def saving_thread_loop(self, file_path, pose, points, image, stamps):
        for idx, config in enumerate(self.configs):
            call('config.dump_config', path=file_path + f'/configs/{idx}.yaml', config=config, sync=True)
        slam.dump_odometry(file_path)
        dump_map_meta(file_path + "/map_meta.json", {'area': self.meta["area"]})
        vertexes = slam.dump_graph(file_path)
        for idx, stamp in stamps.items():
            if idx not in vertexes:
                continue

            try:
                dir_path = file_path + '/{0:06d}'.format(int(idx))
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
