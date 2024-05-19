import os
import pickle
from flask import request
from jsonrpc.backend.flask import api

add_method = api.dispatcher.add_method

class SystemServer:
    def __init__(self, app):
        self.app = app
        self.web_data_path = 'build/web_data'

        if not os.path.exists(self.web_data_path):
            self.web_store = dict()
        else:
            with open(self.web_data_path, 'rb') as f:
                self.web_store = pickle.load(f)

    def setup(self, perception):
        self.app.add_url_rule("/v1/get-web-store", view_func=self.get_web_store, methods=["GET"])
        add_method(self.set_web_store, name='set_web_store')

    def get_web_store(self):
        return self.web_store

    def set_web_store(self, store):
        self.web_store = store
        with open(self.web_data_path, 'wb') as f:
            pickle.dump(store, f)
        return self.web_store