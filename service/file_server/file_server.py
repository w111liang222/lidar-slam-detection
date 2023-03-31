import sys
import os
sys.path.append(os.getcwd())

import time
from threading import Thread
from util.common_util import run_cmd

def has_extension_disk():
    has_disk = False
    disk_name = ''
    with open('/proc/mounts') as f:
        for v in f:
            v = v.split()
            mount_name = v[1]
            if '/media' in mount_name:
                has_disk = True
                disk_name = mount_name
                break
    return has_disk, disk_name

class FileServer:
    def __init__(self):
        self.has_disk = False
        run_cmd('''kill -9 `ps aux | grep "HTTPFileServer.py" | grep -v grep | awk '{print $2}'` > /dev/null 2>&1 ''')

    def start_daemon(self):
        self.daemon_start = True
        self.daemon_thread = Thread(target=self.run, daemon=True)
        self.daemon_thread.start()

    def stop_daemon(self):
        self.daemon_start = False
        self.daemon_thread.join()

    def run(self):
        while self.daemon_start:
            has_disk, disk_name = has_extension_disk()
            if self.has_disk != has_disk:
                if not has_disk:
                    print('kill file server')
                    run_cmd('''kill -9 `ps aux | grep "HTTPFileServer.py" | grep -v grep | awk '{print $2}'` > /dev/null 2>&1 ''')
                elif has_disk:
                    print('start file server on {}'.format(disk_name))
                    run_cmd('mkdir -m777 -p {}/lp_log'.format(disk_name))
                    run_cmd('mkdir -m777 -p {}/lp_log/map'.format(disk_name))
                    os.chmod(disk_name + "/lp_log", 0o777)
                    os.chmod(disk_name + "/lp_log/map", 0o777)
                    run_cmd('python service/file_server/HTTPFileServer.py 8000 {}/lp_log &'.format(disk_name))
            self.has_disk = has_disk
            time.sleep(1)

if __name__ == '__main__':
    file_service = FileServer()
    file_service.start_daemon()

    while True:
        time.sleep(100)