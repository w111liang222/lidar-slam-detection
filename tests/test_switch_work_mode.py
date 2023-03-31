import pytest
import time
import sys
import os
sys.path.append(os.getcwd())

from tests import system
from tests import test_switch_run_mode

def setup_function():
    system.wait_sys_boot_finish()
    test_switch_run_mode.test_offline_mode()
    print('setup_function')

def teardown_function():
    print('teardown_function')

def test_object_detection():
    perception = system.get_perception()
    config = perception.get_config()
    config['pipeline'] = [["Source", "Detect", "Sink"]]
    result = perception.set_config(config)
    time.sleep(1)
    if result["status"] == "Reboot":
        perception.reboot(True)
        time.sleep(5.0)
    system.wait_sys_boot_finish()
    output = perception.get_output()
    assert(bool(output) == True)

def test_slam():
    perception = system.get_perception()
    config = perception.get_config()
    config['pipeline'] = [["Source", "SLAM", "Sink"]]
    result = perception.set_config(config)
    time.sleep(1)
    if result["status"] == "Reboot":
        perception.reboot(True)
        time.sleep(5.0)
    system.wait_sys_boot_finish()

def test_object_detection_slam():
    perception = system.get_perception()
    config = perception.get_config()
    config['pipeline'] = [["Source", "Split", "Detect", "Merge", "Sink"],["Source", "Split", "SLAM", "Merge", "Sink"]]
    result = perception.set_config(config)
    time.sleep(1)
    if result["status"] == "Reboot":
        perception.reboot(True)
        time.sleep(5.0)
    system.wait_sys_boot_finish()
    output = perception.get_output()
    assert(bool(output) == True)

def test_data_capture():
    perception = system.get_perception()
    config = perception.get_config()
    config['pipeline'] = [["Source", "Sink"]]
    result = perception.set_config(config)
    time.sleep(1)
    if result["status"] == "Reboot":
        perception.reboot(True)
        time.sleep(5.0)
    system.wait_sys_boot_finish()
    output = perception.get_output()
    assert(bool(output) == True)
