import pytest
import sys
import os
sys.path.append(os.getcwd())

from tests import system

def setup_function():
    print('setup_function')

def teardown_function():
    print('teardown_function')

def test_boot():
    status = system.wait_sys_boot_finish()
    assert(status == "Running")