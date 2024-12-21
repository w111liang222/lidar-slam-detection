import numpy as np

import slam_wrapper as slam
from .map_manager import MapManager, load_map_meta

from module.slam.slam_template import SLAMTemplate
from module.export_interface import register_interface

class SLAM(SLAMTemplate):
    def __init__(self, mode, method, map_path, sensor_input, resolution,
                       key_frames_interval, config, logger):
        self.method = method if mode == "mapping" else "Localization"
        super().__init__(name = self.method, logger = logger)
        self.mode = mode
        self.map_path = map_path
        self.sensor_input = sensor_input
        self.resolution = resolution
        self.key_frames_interval = key_frames_interval
        self.config = config
        self.ins_ext_params = config.ins.extrinsic_parameters
        self.imu_ext_params = config.ins.imu_extrinsic_parameters

        self.last_timestamp = 0
        self.is_inited = False
        self.map_manager = MapManager(config, logger)

        register_interface('slam.get_pose', self.get_pose)
        register_interface('slam.get_status', self.get_status)
        register_interface('slam.get_edge', self.get_edge)
        register_interface('slam.get_key_frame', self.get_key_frame)
        register_interface('slam.get_graph_meta', self.get_graph_meta)
        register_interface('slam.get_color_map', self.get_color_map)
        register_interface('slam.del_vertex', self.del_vertex)
        register_interface('slam.del_points', self.del_points)
        register_interface('slam.add_edge', self.add_edge)
        register_interface('slam.del_edge', self.del_edge)
        register_interface('slam.add_area', self.add_area)
        register_interface('slam.del_area', self.del_area)
        register_interface('slam.set_vertex_fix', self.set_vertex_fix)
        register_interface('slam.graph_optimize', self.graph_optimize)
        register_interface('slam.keyframe_align', self.keyframe_align)
        register_interface('slam.merge_map', self.merge_map)
        register_interface('slam.set_export_map_config', self.set_export_map_config)
        register_interface('slam.export_map', self.export_map)
        register_interface('slam.set_init_pose', self.set_init_pose)
        register_interface('slam.get_estimate_pose', self.get_estimate_pose)
        register_interface('slam.rotate_ground_constraint', self.rotate_ground_constraint)

    def start(self):
        self.sensor_input = slam.init_slam(self.config.input.mode,
                                           self.map_path, self.method, self.sensor_input, self.resolution,
                                           self.key_frames_interval[0], self.key_frames_interval[1],
                                           self.config.slam.mapping.key_frames_range)
        self.logger.info('SLAM use sensors: %s' % (self.sensor_input))
        # set camera parameters
        slam.set_camera_param(self.config.camera)
        # set ins - lidar extrinic parameter
        slam.set_ins_external_param(self.ins_ext_params[0], self.ins_ext_params[1], self.ins_ext_params[2],
                                    self.ins_ext_params[5], self.ins_ext_params[4], self.ins_ext_params[3])
        # set imu - lidar extrinic parameter
        slam.set_imu_external_param(self.imu_ext_params[0], self.imu_ext_params[1], self.imu_ext_params[2],
                                    self.imu_ext_params[5], self.imu_ext_params[4], self.imu_ext_params[3])
        # set ins config
        slam.set_ins_config(self.config.ins)
        # set destination ip and port for localization output
        slam.set_destination(self.config.output.localization.UDP.use,
                             self.config.output.localization.UDP.destination,
                             self.config.output.localization.UDP.port)
        # setup slam
        is_setup = slam.setup_slam()
        self.sensor_input = [] if not is_setup else self.sensor_input
        # set map origin
        if not self.config.slam.origin.use:
            slam.set_map_origin(self.config.slam.origin.latitude,
                                self.config.slam.origin.longitude,
                                self.config.slam.origin.altitude,
                                0, 0, 0)
        # set function switch
        slam.set_mapping_ground_constraint(self.config.slam.mapping.ground_constraint)
        slam.set_mapping_constraint(self.config.slam.mapping.loop_closure, self.config.slam.mapping.gravity_constraint)
        slam.set_map_colouration(self.config.slam.localization.colouration)

        # load map at initilzation for localization mode
        if self.mode == "localization":
            self.map_manager.update(slam.get_graph_map(), load_map_meta(self.map_path + '/graph/map_meta.json'), True)
            self.map_manager.on_open_map(self.map_path)

        self.is_inited = True

    def stop(self):
        self.map_manager.deinit()
        slam.deinit_slam()

    def isInited(self):
        return self.is_inited

    def enqueue(self, input_dict):
        if not input_dict:
            return False

        # check and collect the sensors
        points_list, points_attr_list = dict(), dict()
        images_list, images_jpeg_list = dict(), dict()
        for sensor in self.sensor_input:
            if sensor == "RTK" or sensor == "IMU":
                continue
            if sensor in input_dict['points']:
                points_list[sensor] = input_dict['points'][sensor]
                points_attr_list[sensor] = input_dict['points_attr'][sensor]
            elif input_dict['image_valid'] and sensor in input_dict['image'] and sensor in input_dict['image_jpeg']:
                images_list[sensor] = input_dict['image'][sensor]
                images_jpeg_list[sensor] = input_dict['image_jpeg'][sensor]

        if len(points_list) <= 0:
            # self.logger.warn('%s, lidar data is invalid at timestamp: %d' % (self.name, input_dict['frame_start_timestamp']))
            return False

        if self.mode == "mapping":
            # if "RTK" in self.sensor_input and input_dict['ins_valid'] is False:
            #     return False

            if "IMU" in self.sensor_input and ('imu_data' not in input_dict or input_dict['imu_data'].shape[0] < 2):
                self.logger.warn('%s, imu data is invalid at timestamp: %d' % (self.name, input_dict['frame_start_timestamp']))
                return False

        if input_dict['frame_start_timestamp'] <= self.last_timestamp:
            self.logger.warn('%s, timestamp is unordered' % (self.name))
            return False

        deviation_timestamp = input_dict['frame_start_timestamp'] - self.last_timestamp
        if deviation_timestamp < 50000 or deviation_timestamp > 150000:
            self.logger.warn('%s, timestamp is not continuous: %d - %d' % (self.name, input_dict['frame_start_timestamp'], self.last_timestamp))

        self.last_timestamp = input_dict['frame_start_timestamp']

        imu_data = input_dict['imu_data'] if 'imu_data' in input_dict else np.zeros((0, 7), dtype=np.float64)
        data_dict = dict(
            frame_start_timestamp=input_dict['frame_start_timestamp'],
            points=points_list,
            points_attr=points_attr_list,
            image=images_list,
            image_jpeg=images_jpeg_list,
            image_param=input_dict['image_param'],
            ins_data=input_dict['ins_data'],
            imu_data=imu_data,
        )
        self.input_queue.put_nowait(data_dict)
        return True

    def get_pose(self):
        data_dict = slam.update_odom()
        for keyframe in data_dict["keyframes"]:
            self.map_manager.add_key_frame(**keyframe)

        self.map_manager.update_pose(data_dict["odoms"], False)
        return self.map_manager.get_pose()

    def get_status(self):
        return slam.get_graph_status()

    def get_edge(self):
        self.map_manager.update_edge(slam.get_graph_edges())
        return self.map_manager.get_edge()

    def get_key_frame(self, id, item):
        return self.map_manager.get_key_frame(id, item)

    def get_graph_meta(self):
        self.get_pose() # flush all keyframe
        self.map_manager.update_meta(slam.get_graph_meta())
        return self.map_manager.get_meta_data()

    def get_color_map(self):
        points = slam.get_color_map() if len(self.map_manager.color_map_bytes) == 0 else None
        return self.map_manager.get_color_map(points)

    def del_vertex(self, id):
        self.map_manager.del_vertex(id)

    def del_points(self, index):
        self.map_manager.del_points(index)

    def add_edge(self, prev_id, next_id, relative):
        self.map_manager.add_edge(prev_id, next_id, relative)

    def del_edge(self, id):
        self.map_manager.del_edge(id)

    def add_area(self, area):
        self.map_manager.add_area(area)

    def del_area(self, id):
        self.map_manager.del_area(id)

    def set_vertex_fix(self, id, fix):
        self.map_manager.set_vertex_fix(id, fix)

    def graph_optimize(self):
        self.map_manager.update_pose(slam.run_graph_optimization(), True)

    def keyframe_align(self, source, target, guess):
        return self.map_manager.keyframe_align(source, target, guess)

    def merge_map(self, map_file):
        self.map_manager.update(slam.merge_map(map_file), load_map_meta(map_file + '/graph/map_meta.json'), False)
        self.map_manager.on_merge_map(map_file)
        self.graph_optimize()

    def set_export_map_config(self, z_min, z_max, color):
        self.map_manager.set_export_map_config(z_min, z_max, color)

    def export_map(self):
        self.map_manager.export_map()

    def set_init_pose(self, pose_range):
        slam.set_init_pose(pose_range[0], pose_range[1], pose_range[2], pose_range[3], pose_range[4], pose_range[5])

    def get_estimate_pose(self, pose_range):
        return slam.get_estimate_pose(pose_range[0][0], pose_range[0][1], pose_range[1][0], pose_range[1][1])

    def rotate_ground_constraint(self):
        is_enable = not slam.get_mapping_ground_constraint()
        slam.set_mapping_ground_constraint(is_enable)
        return "enable" if is_enable else "disable"

    def start_save_mapping(self, root_path, name):
        self.get_pose() # flush all keyframe
        self.map_manager.update_pose(slam.run_robust_graph_optimization(self.mode), True)
        return self.map_manager.start_save_mapping(root_path, name)

    def get_save_progress(self):
        return self.map_manager.get_save_progress()

    def process(self, input_dict):
        result = slam.process(input_dict['points'],
                              input_dict['points_attr'],
                              input_dict['image'],
                              input_dict['image_jpeg'],
                              input_dict['image_param'],
                              input_dict['ins_data'],
                              input_dict['imu_data'],
                              input_dict['frame_start_timestamp'])
        result['pose']['area'] = self.map_manager.is_in_area(result['pose']['odom_matrix'])
        return result
