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

def test_start_record():
    perception = system.get_perception()
    perception.start_record()
    time.sleep(10)

def test_stop_record():
    perception = system.get_perception()
    perception.stop_record()
