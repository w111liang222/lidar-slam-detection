import pytest
import time
import sys
import os
sys.path.append(os.getcwd())

from tests import system
from tests import test_switch_run_mode

def setup_function():
    system.wait_sys_boot_finish()
    test_switch_run_mode.test_online_mode()
    print('setup_function')

def teardown_function():
    print('teardown_function')

def test_lidar_del():
    perception = system.get_perception()
    config = perception.get_config()
    config['lidar'].clear()
    perception.set_config(config)

def test_lidar_add():
    perception = system.get_perception()
    config = perception.get_config()
    config['lidar'].clear()
    test_lidar = {}
    test_lidar['epsilon'] = 0.5
    test_lidar['extrinsic_parameters'] = [0,0,0,0,0,0]
    test_lidar['name'] = "RS-LiDAR-16"
    test_lidar['port'] = 6699
    test_lidar['range'] = [-72, -72, -2, 72, 72, 4]
    config['lidar'].append(test_lidar)
    perception.set_config(config)
    time.sleep(1)
    job = system.get_test_job()
    if "commit_test" in job:
        return

    output = perception.get_output()
    assert(bool(output) == True)
    assert(output['lidar_valid'] == True)
