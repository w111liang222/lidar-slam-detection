
from datetime import datetime
from flask import jsonify, request
from flask.helpers import make_response
from jsonrpc.backend.flask import api

from system_server import SystemServer
from message_server import MessageServer
from module.source.player_server import PlayerServer
from module.slam.slam_server import SlamServer
from calibration.calibration_server import CalibrationServer

from util.common_util import set_system_time, get_disk_status

add_method = api.dispatcher.add_method

__all__ = {
    'system'     : SystemServer,
    'message'    : MessageServer,
    'player'     : PlayerServer,
    'slam'       : SlamServer,
    'calibration': CalibrationServer,
}

class PerceptionServer:
    def __init__(self, app):
        self.app = app
        self.blacklist = set()
        self.client_users = dict()
        self.servers = dict()
        for name, server in __all__.items():
            self.servers[name] = server(app)

    def setup(self, perception):
        self.perception = perception
        for name, server in self.servers.items():
            server.setup(perception)

        # user manager
        self.app.add_url_rule("/v1/client-users", view_func=self.get_client_users, methods=["GET"])
        self.app.add_url_rule("/v1/add-blacklist", view_func=self.add_blacklist, methods=["POST"])
        self.app.add_url_rule("/v1/remove-blacklist", view_func=self.remove_blacklist, methods=["POST"])

        # common
        self.app.add_url_rule("/v1/status", view_func=self.get_status, methods=["POST"])

        # config
        self.app.add_url_rule("/v1/config", view_func=self.get_config, methods=["GET"])
        self.app.add_url_rule("/v1/config", view_func=self.set_config, methods=["POST"])
        self.app.add_url_rule("/v1/restore-config", view_func=self.restore_config, methods=["GET"])

        # roi
        self.app.add_url_rule("/v1/roi", view_func=self.get_roi, methods=["GET"])
        self.app.add_url_rule("/v1/roi", view_func=self.set_roi, methods=["POST"])

        # preview
        self.app.add_url_rule("/v1/detection-pb", view_func=self.get_output, methods=["POST"])
        self.app.add_url_rule("/v1/lidar-pointcloud-map", view_func=self.get_raw_output)

        # json rpc
        add_method(self.reboot, name='reboot')
        add_method(self.start_record, name='start_record')
        add_method(self.stop_record, name='stop_record')

        self.app.register_blueprint(api.as_blueprint(), url_prefix="/api")

    def get_client_users(self):
        client_ip = request.remote_addr
        return {'client_ip': client_ip, 'users': self.client_users}

    def add_blacklist(self):
        ip = request.get_json()['ip']
        if ip in self.client_users and ip not in self.blacklist:
            self.blacklist.add(ip)
            self.client_users[ip]['disable'] = True
        return "ok"

    def remove_blacklist(self):
        ip = request.get_json()['ip']
        if ip in self.client_users and ip in self.blacklist:
            self.blacklist.remove(ip)
            self.client_users[ip]['disable'] = False
        return "ok"

    def get_status(self):
        set_system_time(request.get_json()['host_date'])
        status = self.perception.get_status()
        status['time'] = datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
        status['disk'].update(get_disk_status())
        return status

    def get_config(self):
        return self.perception.get_config()

    def set_config(self):
        return self.perception.set_config(request.get_json())

    def restore_config(self):
        import yaml
        with open("cfg/board_cfg_all_default.yaml", 'r') as f:
            default_config = yaml.safe_load(f)
            self.perception.set_config(default_config)
        return "ok"

    def get_roi(self):
        return jsonify(self.perception.get_config()['roi'])

    def set_roi(self):
        config = self.perception.get_config()
        config['roi'] = [request.get_json()]
        self.perception.set_config(config)
        return ""

    def get_output(self):
        response = make_response(self.perception.call('sink.get_proto_http', request.get_json()))
        response.headers["Content-Type"] = "application/octet-stream"
        return response

    def get_raw_output(self):
        response = make_response(self.perception.call('sink.get_proto_http_raw'))
        response.headers["Content-Type"] = "application/octet-stream"
        return response

    def reboot(self, is_confirmed, hostname):
        return self.perception.do_reboot(is_confirmed, hostname)

    def start_record(self, directory = None):
        self.perception.call('frame.start_record', dict(directory=directory))
        return ""

    def stop_record(self):
        self.perception.call('frame.stop_record')
        return ""
