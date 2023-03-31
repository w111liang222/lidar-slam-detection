import pytest
import sys
import os
sys.path.append(os.getcwd())

from tests import system

def setup_function():
    system.wait_sys_boot_finish()
    print('setup_function')

def teardown_function():
    print('teardown_function')

def test_get_config():
    perception = system.get_perception()
    config = perception.get_config()

def test_set_config():
    perception = system.get_perception()
    config = perception.get_config()
    config["output"]["freespace"]["use"] = not config["output"]["freespace"]["use"]
    perception.set_config(config)