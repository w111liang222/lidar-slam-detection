import os
import time
import zerorpc
import requests
import json
import numpy as np
import cv2
from pathlib import Path
from socket import *

try:
    from proto import detection_pb2
    from util.common_util import get_network
except Exception as e:
    print(e)

session = requests.Session()

def run_cmd(cmd):
    return os.system('%s' % (cmd))

def start_local_sys(prepare_trt = True):
    run_cmd('mkdir -p output/logs')
    run_cmd('touch output/logs/perception.log')
    if prepare_trt:
        run_cmd('python tools/build/prepare_inference_trt.py')
    run_cmd('./tools/scripts/start_system.sh 2>&1 | tee -a output/logs/perception.log &')
    run_cmd('./tools/scripts/start_daemon.sh')

def start_service_sys():
    run_cmd(''' systemctl restart perception.service ''')
    time.sleep(5.0)

def stop_sys():
    run_cmd('./tools/scripts/stop_daemon.sh')
    run_cmd('./tools/scripts/stop_system.sh')

def get_test_job():
    if 'CI_JOB_NAME' in os.environ:
        return os.environ['CI_JOB_NAME']
    else:
        return "commit_test"

def get_board_info():
    from hardware.platform_common import BOARD_NAME, MACHINE, JETPACK
    if 'BOARD_NAME' in os.environ and 'JETPACK' in os.environ:
        board_name, jetpack = os.environ['BOARD_NAME'], os.environ['JETPACK']
        if board_name != BOARD_NAME or jetpack != JETPACK:
            print("WARN: Device inconsistent  {} != {} or {} != {}".format(board_name, BOARD_NAME, jetpack, JETPACK))
            return "", ""
    else:
        board_name, jetpack = BOARD_NAME, JETPACK
    return board_name, jetpack

def get_test_slave():
    BOARD_NAME, JETPACK = get_board_info()

    if BOARD_NAME == "Xavier-NX":
        return "10.10.20.130"
    elif BOARD_NAME == "Xavier-AGX":
        return "10.10.20.131"
    elif BOARD_NAME == "AGX-Orin-32GB":
        return "10.10.20.132"
    else:
        return None

def get_sys_log():
    job = get_test_job()
    if "commit_test" not in job:
        test_slave = get_test_slave()
        run_cmd(''' sshpass -p "tsari123" ssh znqc@{} 'echo "tsari123" | sudo -S chown znqc /tmp/detection_sys/ -R' '''.format(test_slave))
        run_cmd(''' sshpass -p "tsari123" ssh znqc@{} 'journalctl -u perception.service --no-pager --no-hostname >> /tmp/detection_sys/output/logs/perception.log' '''.format(test_slave))
        run_cmd(''' sshpass -p "tsari123" scp -r znqc@{}:/tmp/detection_sys/output ./ '''.format(test_slave))

def get_status_ipc():
    process = os.popen('cat output/logs/status')
    status = process.read().replace("\n","")
    return status

def get_status_rpc():
    try:
        test_slave = 'http://' + get_test_slave() + ':80'
        headers = {
            "User-Agent": "Mozilla/5.0 (Macintosh; Intel Mac OS X 10_10_1) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/39.0.2171.95 Safari/537.36",
            "Content-Type": "application/json; charset=UTF-8"
        }
        args = {'host_date': time.time() * 1000}
        response = session.post(test_slave + '/v1/status', data=json.dumps(args), headers=headers).json()
        return response['status']
    except Exception as e:
        return ""

def wait_sys_boot_finish(timeout = 300):
    job = get_test_job()

    retry = 0
    status = ""
    while retry < timeout:
        time.sleep(1.0)
        retry = retry + 1
        if "commit_test" in job:
            status = get_status_ipc()
        else:
            status = get_status_rpc()
        if status == "Running":
            break
    assert(status == "Running")
    return status

def upgrade():
    job = get_test_job()
    print(job)
    if "commit_test" in job:
        return
    slave_ip = get_test_slave()
    if slave_ip is None:
        print("WARN: no valid slave for upgrading")
        return

    network = get_network('eth0')
    print("Host network: {}, slave ip: {}".format(network[0], slave_ip))
    if slave_ip == network[0]:
        print("Perception server is running at local CI device, start service firstly")
        start_service_sys()

    test_slave = 'http://' + slave_ip + ':1235'
    print("test slaver url: {}".format(test_slave))
    try:
        if 'TestVersion' in os.environ and 'VersionType' in os.environ:
            BOARD_NAME, JETPACK = get_board_info()
            version = os.environ['TestVersion']
            version_type = os.environ['VersionType']
            firmware = version + '-' + BOARD_NAME + '-' + JETPACK + '-' + version_type + '.bin'
            run_cmd('rm ' + firmware)
            run_cmd('wget http://10.10.80.28:8888/IPU-JetPack5/' + firmware)
        else:
            firmware_path = Path('../release/').expanduser().absolute()
            firmware = list(map(str, firmware_path.glob('*.bin')))[0]

        print(firmware)
        f = open(firmware, 'rb')
        # head_bytes = f.read(4096)

        files = {'file': f}
        values = {'DB': 'photcat', 'OUT': 'csv', 'SHORT': 'short'}
        response = session.post(test_slave + '/v1/firmware', files=files, data=values)

        retry = 0
        while retry < 600:
            time.sleep(1)
            retry = retry + 1
            response = session.get(test_slave + '/v1/status').json()
            print(response["log"])
            if response["stage"] == "success" or response["stage"] == "failed":
                break

        assert(response["stage"] == "success")
    except Exception as e:
        print(e)
        exit(1)

    time.sleep(60)

def proto_det_to_dict(data):
    data_dict = {}
    try:
        det = detection_pb2.Detection()
        det.ParseFromString(data)
        data_dict['frame_start_timestamp'] = det.header.timestamp

        freespace = detection_pb2.Freespace()
        freespace.ParseFromString(det.freespace)
        data_dict['freespace'] = {}
        data_dict['freespace']['x_min'] = freespace.info.x_min
        data_dict['freespace']['x_max'] = freespace.info.x_max
        data_dict['freespace']['y_min'] = freespace.info.y_min
        data_dict['freespace']['y_max'] = freespace.info.y_max
        data_dict['freespace']['z_min'] = freespace.info.z_min
        data_dict['freespace']['z_max'] = freespace.info.z_max
        data_dict['freespace']['resolution'] = freespace.info.resolution
        data_dict['freespace']['x_num'] = freespace.info.x_num
        data_dict['freespace']['y_num'] = freespace.info.y_num
        data_dict['freespace']['data'] = freespace.cells

        # Lidar
        data_dict['points'] = np.frombuffer(det.points, dtype=np.float32).reshape(-1, 4)
        if data_dict['points'].shape[0] == 0:
            data_dict['lidar_valid'] = False
        else:
            data_dict['lidar_valid'] = True

        # Camera
        data_dict['image'] = {}
        for camera_image in det.image:
            image = cv2.imdecode(np.frombuffer(camera_image.image, dtype=np.uint8), cv2.IMREAD_COLOR)
            data_dict['image'][camera_image.camera_name] = image

        if not bool(data_dict['image']):
            data_dict['image_valid'] = False
        else:
            data_dict['image_valid'] = True

        # Radar
        data_dict['radar'] = {}
        for radar in det.radar:
            data_dict['radar'][radar.radar_name] = []
            for radar_object in radar.radar_object:
                data_dict['radar'][radar.radar_name].append(
                      dict(id=radar_object.id,
                           type=radar_object.type,
                           x=radar_object.box.center.x,
                           y=radar_object.box.center.y,
                           z=radar_object.box.center.z,
                           length=radar_object.box.length,
                           width=radar_object.box.width,
                           height=radar_object.box.height,
                           heading=radar_object.box.heading,
                           vx=radar_object.velocity_x,
                           vy=radar_object.velocity_y))

        if not bool(data_dict['radar']):
            data_dict['radar_valid'] = False
        else:
            data_dict['radar_valid'] = True

    except Exception as e:
        print(e)
        pass

    return data_dict

def system_power_off():
    network = get_network('eth0')
    HOST = '10.10.21.58'
    if network[0] != HOST:
        print('System power off, host IP: {}, not match'.format(network[0]))
        return
    PORT = 6000
    BUFSIZ = 1024
    ADDR = (HOST, PORT)
    tcpSerSock = socket(AF_INET, SOCK_STREAM)
    tcpSerSock.bind(ADDR)
    tcpSerSock.settimeout(5)
    tcpSerSock.listen(5)
    tcpCliSock, addr = tcpSerSock.accept()
    meg = "AT+STACH1=1\r\n"
    tcpCliSock.send(meg.encode())
    recv_data = tcpCliSock.recv(BUFSIZ)
    time.sleep(0.5)
    tcpCliSock.close()
    tcpSerSock.close()

def system_power_on():
    network = get_network('eth0')
    HOST = '10.10.21.58'
    if network[0] != HOST:
        print('System power on, host IP: {}, not match'.format(network[0]))
        return
    PORT = 6000
    BUFSIZ = 1024
    ADDR = (HOST, PORT)
    tcpSerSock = socket(AF_INET, SOCK_STREAM)
    tcpSerSock.bind(ADDR)
    tcpSerSock.settimeout(5)
    tcpSerSock.listen(5)
    tcpCliSock, addr = tcpSerSock.accept()
    meg = "AT+STACH1=0\r\n"
    tcpCliSock.send(meg.encode())
    recv_data = tcpCliSock.recv(BUFSIZ)
    time.sleep(0.5)
    tcpCliSock.close()
    tcpSerSock.close()

class PerceptionRemote:
    def __init__(self):
        self.test_slave = 'http://' + get_test_slave() + ':80'

    def get_config(self):
        return session.get(self.test_slave + '/v1/config').json()

    def set_config(self, config):
        headers = {
            "User-Agent": "Mozilla/5.0 (Macintosh; Intel Mac OS X 10_10_1) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/39.0.2171.95 Safari/537.36",
            "Content-Type": "application/json; charset=UTF-8"
        }
        return session.post(self.test_slave + '/v1/config', data=json.dumps(config), headers=headers).json()

    def start_record(self):
        payload = {
            "method": "start_record",
            "jsonrpc": "2.0",
            "id": 0,
        }
        response = session.post(self.test_slave + "/api", json=payload).json()
        assert response["id"] == 0

    def stop_record(self):
        payload = {
            "method": "stop_record",
            "jsonrpc": "2.0",
            "id": 0,
        }
        response = session.post(self.test_slave + "/api", json=payload).json()
        assert response["id"] == 0

    def get_output(self, timeout = 30):
        headers = {
            "User-Agent": "Mozilla/5.0 (Macintosh; Intel Mac OS X 10_10_1) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/39.0.2171.95 Safari/537.36",
            "Content-Type": "application/json; charset=UTF-8"
        }
        retry = 0
        while retry < timeout:
            retry = retry + 1
            data = session.post(self.test_slave + '/v1/detection-pb', data=json.dumps(dict(display_on_image=False)), headers=headers).content
            data_dict = proto_det_to_dict(data)
            if bool(data_dict) and data_dict['frame_start_timestamp'] != 0:
                break
        return data_dict

    def reboot(self, confirmed):
        payload = {
            "method": "reboot",
            "jsonrpc": "2.0",
            "id": 0,
            "params": {
                "is_confirmed": confirmed,
                "hostname": "",
            },
        }
        response = session.post(self.test_slave + "/api", json=payload).json()
        assert response["id"] == 0
        time.sleep(5.0)
        return response["result"]

class PerceptionLocal:
    def __init__(self):
        self._init()

    def _init(self):
        self.perception = zerorpc.Client(heartbeat=None, timeout=30)
        self.perception.connect("ipc:///tmp/perception")

    def get_config(self):
        return self.perception.get_config()

    def set_config(self, config):
        return self.perception.set_config(config)

    def start_record(self):
        return self.perception.call('frame.start_record')

    def stop_record(self):
        return self.perception.call('frame.stop_record')

    def get_output(self, timeout = 30):
        retry = 0
        while retry < timeout:
            retry = retry + 1
            data = self.perception.call('sink.get_proto_http', dict(display_on_image=False))
            data_dict = proto_det_to_dict(data)
            if bool(data_dict) and data_dict['frame_start_timestamp'] != 0:
                break
        return data_dict

    def reboot(self, confirmed):
        result = self.perception.do_reboot(confirmed, "", False)
        time.sleep(2.0)
        stop_sys()
        time.sleep(2.0)
        start_local_sys(prepare_trt = False)
        time.sleep(5.0)
        self._init()
        return result

perception = None
def get_perception():
    global perception
    if perception == None:
        job = get_test_job()
        if "commit_test" in job:
            perception = PerceptionLocal()
        else:
            perception = PerceptionRemote()

    return perception
