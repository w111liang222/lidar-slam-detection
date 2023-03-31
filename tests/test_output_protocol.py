import pytest
import time
import sys
import os
import requests
import json
import socket
sys.path.append(os.getcwd())

from tests import system
from tests import test_switch_run_mode
from util.common_util import get_network

def setup_function():
    system.wait_sys_boot_finish()
    test_switch_run_mode.test_offline_mode()
    print('setup_function')

def teardown_function():
    print('teardown_function')

def test_protocol_open():
    perception = system.get_perception()
    config = perception.get_config()
    network_ip = get_network('eth0')
    if network_ip is None:
        return
    network_ip = network_ip[0]
    config['output']['protocol']['CAN']['baud'] = 500000
    config['output']['protocol']['CAN']['device'] = "can0"
    config['output']['protocol']['CAN']['use'] = True
    config['output']['protocol']['UDP']['destination'] = network_ip
    config['output']['protocol']['UDP']['port'] = 19000
    config['output']['protocol']['UDP']['use'] = True
    perception.set_config(config)
    time.sleep(1)

    BUFSIZE = 1024
    ip_port = (network_ip, 19000)
    server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # udp协议
    server.bind(ip_port)
    server.settimeout(5)
    data, client_addr = server.recvfrom(BUFSIZE)
    server.close()
    assert(data is not None)

def test_protocol_close():
    perception = system.get_perception()
    config = perception.get_config()
    config['output']['protocol']['CAN']['use'] = False
    config['output']['protocol']['UDP']['use'] = False
    perception.set_config(config)
