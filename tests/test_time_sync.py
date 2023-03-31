import pytest
import time
import sys
import os
sys.path.append(os.getcwd())

from tests import system

def setup_function():
    system.wait_sys_boot_finish()
    print('setup_function')

def teardown_function():
    print('teardown_function')

def test_ptp_ntp_close():
    perception = system.get_perception()
    config = perception.get_config()
    config['board']['time_sync']['ptp'].clear()
    config['board']['time_sync']['ntp'].clear()
    result = perception.set_config(config)
    if result["status"] == "Reboot":
        time.sleep(1)
        perception.reboot(True)
        time.sleep(5.0)
        system.wait_sys_boot_finish()

def test_ptp_ntp_open():
    perception = system.get_perception()
    config = perception.get_config()
    config['board']['time_sync']['ptp'].clear()
    config['board']['time_sync']['ntp'].clear()
    test_ptp = {}
    test_ptp['interface'] = "eth0"
    test_ptp["mode"] = "master"
    config['board']['time_sync']['ptp'].append(test_ptp)
    test_ntp = {}
    test_ntp['server'] = "ntp.aliyun.com"
    config['board']['time_sync']['ntp'].append(test_ntp)
    result = perception.set_config(config)
    if result["status"] == "Reboot":
        time.sleep(1)
        perception.reboot(True)
        time.sleep(5.0)
        system.wait_sys_boot_finish()
