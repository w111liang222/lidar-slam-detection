import time
from flask import request, send_file
from flask.helpers import make_response
from jsonrpc.backend.flask import api

from util.common_util import has_extension_disk, get_dir_files

add_method = api.dispatcher.add_method

class SlamServer:
    def __init__(self, app):
        self.app = app

    def setup(self, perception):
        self.perception = perception

        self.app.add_url_rule("/v1/restart-mapping", view_func=self.restart_mapping)
        self.app.add_url_rule("/v1/rotate-ground-constraint", view_func=self.rotate_ground_constraint, methods=["POST"])
        self.app.add_url_rule("/v1/save-map", view_func=self.save_map, methods=["POST"])
        self.app.add_url_rule("/v1/get-save-progress", view_func=self.get_save_progress)
        self.app.add_url_rule("/v1/map-vertex", view_func=self.get_map_vertex, methods=["GET"])
        self.app.add_url_rule("/v1/map-status", view_func=self.get_map_status, methods=["GET"])
        self.app.add_url_rule("/v1/get-color-map", view_func=self.get_color_map, methods=["POST"])
        add_method(self.get_map_edge, name='get_map_edge')
        add_method(self.get_map_meta, name='get_map_meta')
        self.app.add_url_rule("/v1/vertex-data", view_func=self.get_vertext_data, methods=["POST"])
        self.app.add_url_rule("/v1/set-init-pose", view_func=self.set_init_pose, methods=["POST"])
        self.app.add_url_rule("/v1/get-estimate-pose", view_func=self.get_estimate_pose, methods=["POST"])
        # map editor
        self.app.add_url_rule("/v1/map-files", view_func=self.get_map_files)
        self.app.add_url_rule("/v1/open-map-file", view_func=self.open_map_file, methods=["POST"])
        self.app.add_url_rule("/v1/merge-map-file", view_func=self.merge_map_file, methods=["POST"])
        add_method(self.map_keyframe_align, name='map_keyframe_align')
        add_method(self.map_add_edge, name='map_add_edge')
        self.app.add_url_rule("/v1/map-del-vertex", view_func=self.map_del_vertex, methods=["POST"])
        add_method(self.map_del_points, name='map-del-points')
        self.app.add_url_rule("/v1/map-del-edge", view_func=self.map_del_edge, methods=["POST"])
        self.app.add_url_rule("/v1/map-add-area", view_func=self.map_add_area, methods=["POST"])
        self.app.add_url_rule("/v1/map-del-area", view_func=self.map_del_area, methods=["POST"])
        self.app.add_url_rule("/v1/map-set-vertex-fix", view_func=self.map_set_vertex_fix, methods=["POST"])
        self.app.add_url_rule("/v1/map-optimize", view_func=self.map_optimize)
        self.app.add_url_rule("/v1/set-export-map-config", view_func=self.set_export_map_config, methods=["POST"])
        self.app.add_url_rule("/v1/map-export-pcd", view_func=self.map_export_pcd)

    def restart_mapping(self):
        self.perception.pause()
        time.sleep(3.0)
        self.perception.call('slam.restart_mapping', dict(config=self.perception.get_config()))
        self.perception.start()
        return ""

    def rotate_ground_constraint(self):
        return self.perception.call('slam.rotate_ground_constraint')

    def save_map(self):
        return self.perception.call('slam.save_mapping', request.get_json())

    def get_save_progress(self):
        return str(self.perception.call('slam.get_save_progress'))

    def get_map_vertex(self):
        result = self.perception.call('slam.get_pose')
        result = result if result is not None else dict()
        return result

    def get_map_status(self):
        result = self.perception.call('slam.get_status')
        result = result if result is not None else dict()
        return result

    def get_color_map(self):
        MAX_SEGMENT_LEN = 1e8
        data, segment = b"", b""
        while len(segment := self.perception.call('slam.get_color_map')) == MAX_SEGMENT_LEN:
            data = data + segment
        data = data + segment

        response = make_response(data)
        response.headers["Content-Type"] = "application/octet-stream"
        return response

    def get_map_edge(self):
        result = self.perception.call('slam.get_edge')
        result = result if result is not None else []
        return result

    def get_map_meta(self):
        result = self.perception.call('slam.get_graph_meta')
        result = result if result is not None else dict()
        return result

    def get_vertext_data(self):
        response = make_response(self.perception.call('slam.get_key_frame', request.get_json()))
        response.headers["Content-Type"] = "application/octet-stream"
        return response

    def set_init_pose(self):
        pose_range = request.get_json()['pose_range']
        self.perception.call('slam.set_init_pose',
                             dict(pose_range=pose_range)
                            )
        return ""

    def get_estimate_pose(self):
        pose_range = request.get_json()['pose_range']
        return self.perception.call('slam.get_estimate_pose',
                                    dict(pose_range=pose_range)
                                   )

    # map editor

    def get_map_files(self):
        return get_dir_files(directory='lp_log/map/')

    def open_map_file(self):
        has_disk, disk_name = has_extension_disk()
        config = self.perception.get_config()
        config['slam']['mode'] = "localization"
        config['slam']['localization']['map_path'] = request.get_json()['map_file'].replace(disk_name + "/", '')
        self.perception.call('slam.restart_mapping', dict(config=config))
        time.sleep(1.0)
        while self.perception.get_status()['status'] == "Initializing":
            time.sleep(1.0)
        return ""

    def merge_map_file(self):
        self.perception.call('slam.merge_map', request.get_json())
        return ""

    def map_keyframe_align(self, source, target, guess):
        return self.perception.call('slam.keyframe_align',
                                    dict(source=source,
                                         target=target,
                                         guess=guess)
                                   )

    def map_add_edge(self, prev, next, relative):
        self.perception.call('slam.add_edge',
                             dict(prev_id=prev,
                                  next_id=next,
                                  relative=relative)
                            )
        return ""

    def map_del_vertex(self):
        self.perception.call("slam.del_vertex", request.get_json())
        return ""

    def map_del_points(self, index):
        self.perception.call('slam.del_points',
                             dict(index=index)
                            )
        return ""

    def map_del_edge(self):
        self.perception.call("slam.del_edge", request.get_json())
        return ""

    def map_add_area(self):
        self.perception.call("slam.add_area", dict(area=request.get_json()))
        return ""

    def map_del_area(self):
        self.perception.call("slam.del_area", request.get_json())
        return ""

    def map_set_vertex_fix(self):
        self.perception.call("slam.set_vertex_fix", request.get_json())
        return ""

    def map_optimize(self):
        self.perception.call('slam.graph_optimize')
        return ""

    def set_export_map_config(self):
        self.perception.call("slam.set_export_map_config", request.get_json())
        return ""

    def map_export_pcd(self):
        self.perception.call('slam.export_map')
        f = open("output/export_map.pcd", mode='rb')
        return send_file(f, download_name='export_map.pcd', as_attachment=True)