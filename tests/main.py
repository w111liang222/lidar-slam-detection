import os
import sys
import pytest
sys.path.append(os.getcwd())

from tests import system

def run_cmd(cmd):
    return os.system('%s' % (cmd))

if __name__ == "__main__":
    job = system.get_test_job()
    if "commit_test" not in job and system.get_test_slave() is None:
        print("WARN: No valid slave for test")
        exit(0)

    test_args = ['--junitxml=report.xml', 'tests/']
    if "commit_test" in job:
        test_args = ['--exitfirst'] + test_args
    ret = pytest.main(test_args)

    if "commit_test" in job:
        run_cmd(''' python -c 'import tests.system as sys; sys.stop_sys()' ''')
    else:
        run_cmd(''' systemctl stop perception.service ''')

    run_cmd(''' python -c 'import tests.system as sys; sys.get_sys_log()' ''')

    print(ret)
    if ret != 0:
        exit(1)
    else:
        exit(0)