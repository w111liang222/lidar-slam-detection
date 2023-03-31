import pytest
import time
import sys
import os
import requests
import json
import socket
sys.path.append(os.getcwd())

from tests import system
from tests import test_lidar_add_del
from util.common_util import get_network

def setup_function():
    system.wait_sys_boot_finish()
    print('setup_function')

def teardown_function():
    print('teardown_function')

def test_point_forward_close():
    perception = system.get_perception()
    config = perception.get_config()
    config['output']['point_cloud']['use'] = False
    perception.set_config(config)

def test_point_forward_open():
    test_lidar_add_del.test_lidar_add()
    perception = system.get_perception()
    config = perception.get_config()
    network_ip = get_network('eth0')
    if network_ip is None:
        return
    network_ip = network_ip[0]
    config['output']['point_cloud']['use'] = True
    config['output']['point_cloud']['destination'] = network_ip
    perception.set_config(config)
    time.sleep(1)

    job = system.get_test_job()
    if "commit_test" in job:
        return

    BUFSIZE = 1024
    ip_port = (network_ip, 6699)
    server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # udp协议
    server.bind(ip_port)
    server.settimeout(5)
    data, client_addr = server.recvfrom(BUFSIZE)
    server.close()
    assert(data is not None)
