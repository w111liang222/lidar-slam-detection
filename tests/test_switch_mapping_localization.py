import pytest
import time
import sys
import os
sys.path.append(os.getcwd())

from tests import system
from tests import test_switch_run_mode
from tests import test_switch_work_mode

def setup_function():
    system.wait_sys_boot_finish()
    test_switch_run_mode.test_offline_mode()
    print('setup_function')

def teardown_function():
    print('teardown_function')

def set_mapping_mode():
    test_switch_work_mode.test_slam()
    perception = system.get_perception()
    config = perception.get_config()
    config['slam']['mode'] = 'mapping'
    perception.set_config(config)
    time.sleep(1)

def test_localization_mode():
    test_switch_work_mode.test_slam()
    perception = system.get_perception()
    config = perception.get_config()
    config['slam']['mode'] = 'localization'
    perception.set_config(config)
    time.sleep(1)

def test_mapping_RTKM():
    set_mapping_mode()
    perception = system.get_perception()
    config = perception.get_config()
    config['slam']['method'] = 'RTKM'
    config['slam']['mapping']['sensor_input'] = ['1-Ouster-OS1-128', 'RTK', 'IMU']
    config['ins']['ins_fix']['use'] = True
    config['ins']['ins_fix']['status'] = 42
    config['ins']['ins_fix']['stable_time'] = 2
    config['ins']['ins_float']['use'] = True
    config['ins']['ins_float']['status'] = 52
    config['ins']['ins_float']['stable_time'] = 2
    perception.set_config(config)
    time.sleep(10)
    output = perception.get_output()
    assert(bool(output) == True)

def test_mapping_GICPM():
    set_mapping_mode()
    perception = system.get_perception()
    config = perception.get_config()
    config['slam']['method'] = 'GICPM'
    config['slam']['mapping']['sensor_input'] = ['1-Ouster-OS1-128', 'RTK', 'IMU']
    config['ins']['ins_fix']['use'] = True
    config['ins']['ins_fix']['status'] = 42
    config['ins']['ins_fix']['stable_time'] = 2
    config['ins']['ins_float']['use'] = True
    config['ins']['ins_float']['status'] = 52
    config['ins']['ins_float']['stable_time'] = 2
    perception.set_config(config)
    time.sleep(10)
    output = perception.get_output()
    assert(bool(output) == True)

def test_mapping_FastLIO():
    set_mapping_mode()
    perception = system.get_perception()
    config = perception.get_config()
    config['slam']['method'] = 'FastLIO'
    config['slam']['mapping']['sensor_input'] = ['1-Ouster-OS1-128', 'RTK', 'IMU']
    config['ins']['ins_fix']['use'] = True
    config['ins']['ins_fix']['status'] = 42
    config['ins']['ins_fix']['stable_time'] = 2
    config['ins']['ins_float']['use'] = True
    config['ins']['ins_float']['status'] = 52
    config['ins']['ins_float']['stable_time'] = 2
    perception.set_config(config)
    time.sleep(10)
    output = perception.get_output()
    assert(bool(output) == True)
