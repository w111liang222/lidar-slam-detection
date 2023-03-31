import pytest
import sys
import os
sys.path.append(os.getcwd())

from tests import system

def setup_function():
    print('setup_function')

def teardown_function():
    print('teardown_function')

def test_startup():
    job = system.get_test_job()
    if "commit_test" in job:
        system.start_local_sys()
    else:
        system.start_service_sys()
