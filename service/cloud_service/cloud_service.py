import sys
import os
sys.path.append(os.getcwd())

import requests
from requests.packages.urllib3.exceptions import InsecureRequestWarning
requests.packages.urllib3.disable_warnings(InsecureRequestWarning)

import json
import time
from datetime import datetime
from threading import Thread
from easydict import EasyDict
import yaml

from service.cloud_service.http_proxy import HttpProxy
from service.cloud_service.remote_control import RemoteControl
from util.log import create_logger

HIGH_INTERVAL = 0.1
VALID_INTERVAL = 1
IDLE_INTERVAL = 10

class CloudService:
    def __init__(self):
        self.logger = create_logger()
        self.heart_beat = IDLE_INTERVAL
        self.disconnect_count = 0
        self.config = EasyDict(yaml.safe_load(open('cfg/board_cfg_all.yaml')))
        self.cloud_server = 'https://' + self.config.board.cloud_server.IP + ':' + \
                            str(self.config.board.cloud_server.port)
        self.status_session = requests.Session()
        self.cloud_session = requests.Session()
        self.serial_number = self.get_board_serial_number()
        self.proxy = HttpProxy(self.logger)
        self.remote_control = RemoteControl(self.logger)

    def get_board_serial_number(self):
        try:
            with open('/sys/firmware/devicetree/base/serial-number',"r") as f:
                serial_number = f.read().rstrip('\x00')
        except Exception as e:
            with open('/sys/class/dmi/id/product_uuid',"r") as f:
                serial_number = f.read().rstrip('\n')

        return serial_number

    def get_status(self):
        status = dict(
            status='Offline',
            time=datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
            lidar=dict(),
            camera=dict(),
            ins=dict(
                valid=False,
                latitude=0,
                longitude=0,
                heading=0,
                velocity=0,
            ),
            disk=dict(
                disk_name='',
                has_disk=False,
                total=0,
                used_percent=0,
            ),
        )
        try:
            host = 'http://127.0.0.1:80'
            headers = {
                "User-Agent": "Mozilla/5.0 (Macintosh; Intel Mac OS X 10_10_1) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/39.0.2171.95 Safari/537.36",
                "Content-Type": "application/json; charset=UTF-8"
            }
            args = {'host_date': time.time() * 1000}
            status = self.status_session.post(host + '/v1/status', data=json.dumps(args), headers=headers).json()
            status['disk'].pop('status', None)
            status['ins'].pop('received_count', None)
            status['ins'].pop('valid_count', None)
        except Exception as e:
            pass
        return status

    def start_daemon(self):
        self.remote_control.start_daemon()
        self.daemon_start = True
        self.daemon_thread = Thread(target=self.run, daemon=True)
        self.daemon_thread.start()

    def stop_daemon(self):
        self.daemon_start = False
        self.daemon_thread.join()

    def run(self):
        while self.daemon_start:
            self.report_status()
            time.sleep(self.heart_beat)

    def stop_service(self):
        self.proxy.stop_proxy()

    def parse_request(self, request):
        if 'proxy_remote_ports' not in request or 'proxy_driving_port' not in request:
            self.stop_service()
            return

        self.proxy.start_proxy(self.config.board.cloud_server.IP, self.serial_number, request['proxy_remote_ports'])
        if request['proxy_driving_port'] == -1:
            self.proxy.stop_driving_proxy()
            self.heart_beat = VALID_INTERVAL
        else:
            self.proxy.start_driving_proxy(self.config.board.cloud_server.IP, self.serial_number, request['proxy_driving_port'])
            self.heart_beat = HIGH_INTERVAL

        self.remote_control.set_certification(request['certification'])

    def report_status(self):
        status = self.get_status()
        status['remote_control'] = dict(
            connection=self.remote_control.get_status()
        )
        status['serial_number'] = self.serial_number
        try:
            headers = {
                "User-Agent": "Mozilla/5.0 (Macintosh; Intel Mac OS X 10_10_1) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/39.0.2171.95 Safari/537.36",
                "Content-Type": "application/json; charset=UTF-8"
            }
            result = self.cloud_session.post(self.cloud_server + '/v1/ipc_report_status', data=json.dumps(status), headers=headers, timeout=2.0, verify=False).json()
            if result['result'] == 'success':
                self.disconnect_count = 0
                self.parse_request(result)
        except Exception as e:
            if self.heart_beat == VALID_INTERVAL:
                self.logger.warn(e)
                self.disconnect_count = self.disconnect_count + 1
                if self.disconnect_count > 5:
                    self.heart_beat = IDLE_INTERVAL
                    self.stop_service()
                    self.logger.warn('I-Cloud lost connection')

if __name__ == '__main__':
    cloud_service = CloudService()
    cloud_service.start_daemon()

    while True:
        time.sleep(100)
