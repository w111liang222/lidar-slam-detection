import os
import pickle
from flask import request
from jsonrpc.backend.flask import api

add_method = api.dispatcher.add_method

TRAIL_TIME    = 5 * 60
LICENCE_STORE = 'licence/licences'
LICENCE_USAGE = 'licence/licence_usage'
FULL_LICENCE  = 'licence/full_license'
WEB_STORE     = 'licence/web_store'

class SystemServer:
    def __init__(self, app):
        self.app = app
        self.work_space = '/home/znqc/work/'

        try:
            os.system('mkdir -p ' + self.work_space + 'licence')
            if not os.path.exists(self.work_space + LICENCE_STORE):
                self.licences = dict(
                    WXstAP9SAqBY= 1 * 30 * 24 * 60 * 60,
                    B6s8LkHMnCAT= 3 * 30 * 24 * 60 * 60,
                    y76pBT6EBW2V=12 * 30 * 24 * 60 * 60,
                    kfo674JmowKw=-1,
                )
                with open(self.work_space + LICENCE_STORE, 'wb') as f:
                    pickle.dump(self.licences, f)
            else:
                with open(self.work_space + LICENCE_STORE, 'rb') as f:
                    self.licences = pickle.load(f)

            if not os.path.exists(self.work_space + LICENCE_USAGE):
                self.licence_fd = open(self.work_space + LICENCE_USAGE, 'w+')
                self.usage_left = TRAIL_TIME
                self.licence_fd.write(str(self.usage_left))
            else:
                self.licence_fd = open(self.work_space + LICENCE_USAGE, 'r+')
                usage_left = self.licence_fd.read()
                self.usage_left = int(usage_left) if usage_left.isdigit() else TRAIL_TIME

            if not os.path.exists(self.work_space + WEB_STORE):
                self.web_store = dict()
            else:
                with open(self.work_space + WEB_STORE, 'rb') as f:
                    self.web_store = pickle.load(f)

        except Exception as e:
            print(e)

    def setup(self, perception):
        self.app.add_url_rule("/v1/is-permit", view_func=self.is_permit, methods=["GET"])
        self.app.add_url_rule("/v1/licence-register", view_func=self.licence_register, methods=["POST"])

        # web store
        self.app.add_url_rule("/v1/get-web-store", view_func=self.get_web_store, methods=["GET"])
        add_method(self.set_web_store, name='set_web_store')

    def is_permit(self):
        result = dict(
            licence="registered",
            time=-1,
        )
        # if os.path.exists(self.work_space + FULL_LICENCE):
        return result

        self.usage_left = max(0, self.usage_left - 10)
        self.licence_fd.seek(0)
        self.licence_fd.write(str(self.usage_left))

        result['time'] = self.usage_left
        if self.usage_left <= 0:
            result['licence'] = "error"
        elif self.usage_left > TRAIL_TIME:
            result['licence'] = "active"
        else:
            result['licence'] = "trial"

        return result

    def licence_register(self):
        licence = request.get_json()['licence']
        if licence not in self.licences or self.licences[licence] == 0:
            return "error"

        if self.licences[licence] == -1:
            os.system('touch ' + self.work_space + FULL_LICENCE)
        else:
            self.usage_left = self.licences[licence]
            self.licences[licence] = 0
            self.licence_fd.seek(0)
            self.licence_fd.write(str(self.usage_left))
            with open(self.work_space + LICENCE_STORE, 'wb') as f:
                pickle.dump(self.licences, f)

        os.system('sync')
        return "ok"

    def get_web_store(self):
        return self.web_store

    def set_web_store(self, store):
        self.web_store = store
        try:
            with open(self.work_space + WEB_STORE, 'wb') as f:
                pickle.dump(store, f)
        except Exception as e:
            print(e)
        return self.web_store