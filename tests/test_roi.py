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

def test_roi_inside():
    perception = system.get_perception()
    config = perception.get_config()
    config['roi'] = []
    test_roi = {}
    test_roi['contour'] = [[2.573407573981699,2.0237128002732843],[-2.8541988304442008,1.9104380966288246],[-2.521217181602222,-2.212826389946295],[2.7500364234786447,-2.0844084938114276],[2.573407573981699,2.0237128002732843]]
    test_roi['include'] = True
    config['roi'].append(test_roi)
    perception.set_config(config)
    time.sleep(1)

def test_roi_outside():
    perception = system.get_perception()
    config = perception.get_config()
    config['roi'] = []
    test_roi = {}
    test_roi['contour'] = [[2.573407573981699,2.0237128002732843],[-2.8541988304442008,1.9104380966288246],[-2.521217181602222,-2.212826389946295],[2.7500364234786447,-2.0844084938114276],[2.573407573981699,2.0237128002732843]]
    test_roi['include'] = False
    config['roi'].append(test_roi)
    perception.set_config(config)
    time.sleep(1)

def test_roi_redraw():
    perception = system.get_perception()
    config = perception.get_config()
    config['roi'] = []
    test_roi = {}
    test_roi['contour'] = []
    test_roi['include'] = True
    config['roi'].append(test_roi)
    perception.set_config(config)
    time.sleep(1)
