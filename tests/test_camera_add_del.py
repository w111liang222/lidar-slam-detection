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

def test_camera_del():
    perception = system.get_perception()
    config = perception.get_config()
    config['camera'].clear()
    perception.set_config(config)

def test_camera_add():
    perception = system.get_perception()
    config = perception.get_config()
    config['camera'].clear()

    camera_name = 'rtsp://admin:@10.10.20.208'
    test_camera = {}
    test_camera['extrinsic_parameters'] = [0, 0, 0, 0, 0, 0]
    test_camera['intrinsic_parameters'] = [0, 0, 0, 0, 0, 0, 0, 0]
    test_camera['name'] = camera_name
    test_camera['output_height'] = 1080
    test_camera['output_width'] = 1920
    config['camera'].append(test_camera)
    perception.set_config(config)
    time.sleep(1)

    output = perception.get_output()
    assert(bool(output) == True)
    assert(output['image_valid'] and camera_name in output['image'])

    camera_data = output['image'][camera_name]
    assert(camera_data.shape[0] == 1080 and camera_data.shape[1] == 1920)
