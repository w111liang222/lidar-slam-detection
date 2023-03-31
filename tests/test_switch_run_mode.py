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

def test_offline_mode():
    perception = system.get_perception()
    config = perception.get_config()
    config['input']['mode'] = 'offline'
    config['input']['data_path'] = '/media/pickle_data'
    perception.set_config(config)
    time.sleep(1)

def test_online_mode():
    perception = system.get_perception()
    config = perception.get_config()
    config['input']['mode'] = 'online'
    perception.set_config(config)
    time.sleep(1)
