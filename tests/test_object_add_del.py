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

def test_object_del():
    perception = system.get_perception()
    config = perception.get_config()
    config['output']['freespace']['use'] = False
    config['output']['object']['cyclist'] = False
    config['output']['object']['pedestrian'] = False
    config['output']['object']['traffic_cone'] = False
    config['output']['object']['use'] = True
    config['output']['object']['vehicle'] = False
    perception.set_config(config)
    time.sleep(1)

def test_object_add():
    perception = system.get_perception()
    config = perception.get_config()
    config['output']['freespace']['use'] = True
    config['output']['object']['cyclist'] = True
    config['output']['object']['pedestrian'] = True
    config['output']['object']['traffic_cone'] = True
    config['output']['object']['use'] = True
    config['output']['object']['vehicle'] = True
    perception.set_config(config)
    time.sleep(1)
