import os
from util.common_util import run_cmd
from hardware.platform_common import MACHINE

SERVER_TEMPLATE = '''\
[common]
server_addr = {}
server_port = 7000
pool_count = 5
'''

PORT_TEMPLATE = '''\
[{}]
type = tcp
local_ip = 127.0.0.1
local_port = {}
remote_port = {}
'''

CONFIG_TEMPLATE = '''\
[{}]
type = tcp
local_ip = 127.0.0.1
local_port = 80
remote_port = {}
'''

UPGRADE_TEMPLATE = '''\
[{}]
type = tcp
local_ip = 127.0.0.1
local_port = 1235
remote_port = {}
'''

AUDIO_TEMPLATE = '''\
[{}]
type = udp
local_ip = 127.0.0.1
local_port = 9088
remote_port = {}
'''

DRIVING_TEMPLATE = '''\
[{}]
type = udp
local_ip = 127.0.0.1
local_port = 12660
remote_port = {}
'''

class HttpProxy:
    def __init__(self, logger):
        self.logger = logger
        self.rtsp_base_port = 8554
        self.is_running = self.is_proxy_run()
        self.stop_proxy()
        self.driving_proxy_running = False
        self.remote_ports = []

    def is_proxy_run(self):
        process = os.popen(''' ps aux | grep "frpc" | grep -v grep | awk '{print $2}' ''')
        pids = process.read().replace("\n","")
        if pids == '':
            return False
        else:
            return True

    def start_proxy(self, server_ip, serial_num, remote_ports):
        # check restart frpc
        if self.remote_ports != remote_ports:
            self.logger.info('restart http proxy {} -> {}'.format(self.remote_ports, remote_ports))
            self.stop_proxy()
            self.remote_ports = remote_ports

        if self.is_running or len(remote_ports) <= 1:
            return

        frpc_ini = SERVER_TEMPLATE.format(server_ip)
        for idx in range(len(self.remote_ports) - 2):
            proxy_name = serial_num + '-' + str(idx)
            frpc_ini = frpc_ini + PORT_TEMPLATE.format(proxy_name, self.rtsp_base_port + idx, self.remote_ports[idx])

        frpc_ini = frpc_ini + CONFIG_TEMPLATE.format(serial_num + '-' + 'config', self.remote_ports[-2])
        frpc_ini = frpc_ini + UPGRADE_TEMPLATE.format(serial_num + '-' + 'upgrade', self.remote_ports[-1])
        frpc_ini = frpc_ini + AUDIO_TEMPLATE.format(serial_num + '-' + 'audio', self.remote_ports[-2])

        with open('service/cloud_service/frpc.ini', mode='w') as f:
            f.write(frpc_ini)
            os.fsync(f)
        self.logger.info('start http proxy')
        run_cmd(f''' service/cloud_service/{MACHINE}/frpc -c service/cloud_service/frpc.ini &''')
        self.is_running = True

    def stop_proxy(self):
        if not self.is_running:
            return

        self.logger.info('stop http proxy')
        run_cmd(''' kill -9 `ps aux | grep "frpc" | grep -v grep | awk '{print $2}'` > /dev/null 2>&1 ''')
        self.is_running = False
        self.driving_proxy_running = False
        self.remote_ports = []

    def start_driving_proxy(self, server_ip, serial_num, driving_ports):
        if self.driving_proxy_running:
            return

        frpc_ini = SERVER_TEMPLATE.format(server_ip)
        frpc_ini = frpc_ini + DRIVING_TEMPLATE.format(serial_num + '-' + 'driving', driving_ports)
        with open('service/cloud_service/frpc_driving.ini', mode='w') as f:
            f.write(frpc_ini)
            os.fsync(f)
        self.logger.info('start driving proxy')
        run_cmd(f''' service/cloud_service/{MACHINE}/frpc -c service/cloud_service/frpc_driving.ini &''')
        self.driving_proxy_running = True

    def stop_driving_proxy(self):
        if not self.driving_proxy_running:
            return

        self.logger.info('stop driving proxy')
        run_cmd(''' kill -9 `ps aux | grep "service/cloud_service/{}/frpc -c service/cloud_service/frpc_driving.ini" | grep -v grep | awk '{print $2}'` > /dev/null 2>&1 '''.format(MACHINE))
        self.driving_proxy_running = False