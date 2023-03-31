import pytest
import sys
import os
import time
sys.path.append(os.getcwd())

from tests import system

def setup_function():
    system.wait_sys_boot_finish()
    print('setup_function')

def teardown_function():
    print('teardown_function')

def test_power_off_on():
    job = system.get_test_job()
    if "commit_test" in job:
        return

    system.system_power_off()
    time.sleep(1.0)
    system.system_power_on()
    system.wait_sys_boot_finish()
