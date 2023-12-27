import os
import sys
import time
import zerorpc
from threading import Thread
from multiprocessing import Process

sys.path.append(os.getcwd())

def run_perception_rpc_server():
    def start_perception_server(perception, endpoint):
        s = zerorpc.Server(perception)
        s.bind(endpoint)
        s.run()

    def perception_main():
        from module.perception import Perception
        perception = Perception()
        # start perception tcp server
        server_thread = Thread(target=start_perception_server, args=(perception, "tcp://0.0.0.0:37040",))
        server_thread.name = "zerorpc"
        server_thread.daemon = True
        server_thread.start()
        # start perception ipc server
        start_perception_server(perception, "ipc:///tmp/perception")
    p = Process(target=perception_main)
    p.start()
    return p

# start perception first
run_perception_rpc_server()

# do post-processing of web server
from flask import Flask, request, send_file, abort
from flask_cors import CORS
from perception_server import PerceptionServer

import sensor_driver.common_lib.cpp_utils as util
util.set_thread_priority("Flask", 30)

cwd = os.getcwd()
app = Flask(__name__, static_url_path="/", static_folder=cwd + "/www")
app.config['SEND_FILE_MAX_AGE_DEFAULT'] = 0
CORS(app)

server = PerceptionServer(app)

@app.before_first_request
def connect_perception_rpc_server():
    perception = zerorpc.Client(heartbeat=None, timeout=1000000)
    perception.connect("ipc:///tmp/perception")
    server.setup(perception)

@app.before_request
def user_management():
    ip = request.remote_addr
    if ip in server.blacklist:
        abort(403)

    if ip == "::ffff:127.0.0.1":
        return

    if ip not in server.client_users:
        server.client_users[ip] = dict(
            time=0,
            requests=0,
            disable=False,
        )
    server.client_users[ip]['time'] = time.time()
    server.client_users[ip]['requests'] += 1

@app.after_request
def add_header(r):
    """
    Add headers to both force latest IE rendering engine or Chrome Frame,
    and also to cache the rendered page for 10 minutes.
    """
    if request.path.startswith("/v1") or request.path.startswith("/api"):
        r.headers["Cache-Control"] = "no-cache, no-store, must-revalidate"
        r.headers["Pragma"] = "no-cache"
        r.headers["Expires"] = "0"
        r.headers["Cache-Control"] = "public, max-age=0"
    return r

@app.route("/")
def get_index():
    return send_file(cwd + "/www/index.html")

@app.errorhandler(404)
def handle_404(e):
    return send_file(cwd + "/www/index.html")

if __name__ == "__main__":
    try:
        from gevent.pywsgi import WSGIServer
        http_server = WSGIServer(("", 80), app, log = None)
        http_server.serve_forever()
    except Exception as e:
        http_server = WSGIServer(("", 1234), app, log = None)
        http_server.serve_forever()
