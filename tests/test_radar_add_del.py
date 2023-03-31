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

def test_radar_add():
    perception = system.get_perception()
    config = perception.get_config()
    config['radar'].clear()
    test_radar = {}
    test_radar['baud'] = 500000
    test_radar['device'] = 'can0'
    test_radar['extrinsic_parameters'] = [0,0,0,0,0,0]
    test_radar['name'] = 'ARS408'
    config['radar'].append(test_radar)
    perception.set_config(config)
    time.sleep(1)
    job = system.get_test_job()
    if "commit_test" in job:
        return

    output = perception.get_output()
    assert(bool(output) == True)
    assert(output['radar_valid'] and '0-ARS408' in output['radar'])

def test_radar_del():
    perception = system.get_perception()
    config = perception.get_config()
    config['radar'].clear()
    perception.set_config(config)
    time.sleep(1)
