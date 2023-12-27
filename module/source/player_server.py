from flask import request
from util.common_util import get_dir_files

class PlayerServer:
    def __init__(self, app):
        self.app = app

    def setup(self, perception):
        self.perception = perception

        self.app.add_url_rule("/v1/player-status", view_func=self.get_player_status)
        self.app.add_url_rule("/v1/player-start", view_func=self.player_start)
        self.app.add_url_rule("/v1/player-pause", view_func=self.player_pause)
        self.app.add_url_rule("/v1/player-seek", view_func=self.player_seek, methods=["POST"])
        self.app.add_url_rule("/v1/player-rate", view_func=self.set_player_rate, methods=["POST"])
        self.app.add_url_rule("/v1/player-step", view_func=self.set_player_step, methods=["POST"])
        self.app.add_url_rule("/v1/record-files", view_func=self.get_record_files)
        self.app.add_url_rule("/v1/play-record-file", view_func=self.set_record_file, methods=["POST"])

    def get_player_status(self):
        return self.perception.call("playback.get_player_status")

    def player_start(self):
        self.perception.start()
        return ""

    def player_pause(self):
        self.perception.pause()
        return ""

    def player_seek(self):
        return self.perception.call("playback.player_seek", request.get_json())

    def set_player_rate(self):
        self.perception.call("playback.set_player_rate", request.get_json())
        return ""

    def set_player_step(self):
        return self.perception.call("playback.set_player_step", request.get_json())

    def get_record_files(self):
        return get_dir_files(directory='lp_log/')

    def set_record_file(self):
        config = self.perception.get_config()
        config['input']['data_path'] = request.get_json()['record_file']
        self.perception.set_config(config)
        return ""