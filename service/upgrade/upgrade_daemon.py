from flask import jsonify, request, send_file
from pathlib import Path
import time
import zipfile
from io import BytesIO

import sys
import os

sys.path.append(os.getcwd())

from service.xterm.app import app, socketio
from service.upgrade.upgrade import Upgrade
from util.common_util import run_cmd

upgrader = Upgrade()


@app.route("/v1/firmware", methods=["POST"])
def upload_firmware():
    file = request.files["file"]
    upgrader.upgrade(file.read())
    return ""


MAP = {
    "No Upgrade": "idle",
    "Downloading": "uploading",
    "Prepare": "preparing",
    "Post Process": "postprocessing",
}


@app.route("/v1/status")
def get_upgrade_status():
    status, percent = upgrader.get_status()
    log = upgrader.get_log()
    return dict(
        stage=MAP[status] if status in MAP else status.lower(),
        percentage=percent,
        log=log,
    )

@app.route("/v1/version")
def get_version():
    ver = upgrader.get_current_version()
    return dict(
        version=ver,
    )

LOG_ROOT = "output/logs"

@app.route("/v1/log-file-list")
def get_log_files():
    log_files = [x.name for x in Path(LOG_ROOT).glob("*")]
    log_files.append('dmesg')
    log_files.append('perception.log')
    return jsonify(log_files)


@app.route("/v1/log-content")
def get_log_content():
    filename = request.args.get("filename", False)
    if not filename:
        return ""

    if filename == "dmesg":
        log_content = os.popen('dmesg').read()
    elif filename == "perception.log":
        log_content = os.popen('journalctl -u perception.service --no-pager --no-hostname -n 1000').read()
    else:
        log_content = Path(LOG_ROOT).joinpath(filename).open().read()
    return log_content

@app.route("/v1/log-download")
def get_log_download():
    log_files = [x.name for x in Path(LOG_ROOT).glob("*")]
    memory_file = BytesIO()
    with zipfile.ZipFile(memory_file, 'w') as zf:
        for log in log_files:
            data = zipfile.ZipInfo(log)
            data.date_time = time.localtime(time.time())[:6]
            data.compress_type = zipfile.ZIP_DEFLATED
            zf.writestr(data, Path(LOG_ROOT).joinpath(log).open().read())
        # dmesg
        data = zipfile.ZipInfo("kernel.log")
        data.date_time = time.localtime(time.time())[:6]
        data.compress_type = zipfile.ZIP_DEFLATED
        zf.writestr(data, os.popen('dmesg').read())
        # perception.service
        data = zipfile.ZipInfo("perception.log")
        data.date_time = time.localtime(time.time())[:6]
        data.compress_type = zipfile.ZIP_DEFLATED
        zf.writestr(data, os.popen('journalctl -u perception.service --no-pager --no-hostname').read())
    memory_file.seek(0)
    version = upgrader.get_current_version()
    log_time = time.strftime("%Y-%m-%d-%H-%M-%S", time.localtime())
    return send_file(memory_file, download_name='log-'+ version + '-'+ log_time +'.zip', as_attachment=True)

@app.route("/v1/system-power-action", methods=["POST"])
def system_power_action():
    req = request.get_json()
    if req["action"] == "PowerOff":
        run_cmd("shutdown now")
    elif req["action"] == "Reboot":
        run_cmd("systemctl restart perception.service")
    return ""

if __name__ == "__main__":
    socketio.run(app, debug=False, port=1235, host="0.0.0.0")
