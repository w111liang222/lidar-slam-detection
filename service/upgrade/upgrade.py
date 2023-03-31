import sys
import os
sys.path.append(os.getcwd())

from threading import Thread
from pathlib import Path
from easydict import EasyDict
import yaml
import json
import time

from service.upgrade.scripts import *
from util.log import create_logger
from util.common_util import run_cmd

class Upgrade:
    def __init__(self):
        self.upgrade_status = ['No Upgrade', 'Downloading',
                               'Prepare', 'Upgrading',
                               'Verifying', 'Post Process',
                               'Success', 'Failed']
        self.upgrade_thread = None
        self._reset()

    def generate_and_run_command(self, script_string):
        fs = open('service/upgrade/current_process.sh', 'w')
        fs.write(script_string)
        fs.close()
        result = run_cmd("bash service/upgrade/current_process.sh >> output/logs/upgrade.log 2>&1")
        os.unlink('service/upgrade/current_process.sh')
        return result

    def get_current_version(self):
        if not os.path.exists('VERSION'):
            return 'Dev'
        else:
            f = open('VERSION', 'r')
            return f.read().replace("\n","")

    def get_firmware_header(self, head_bytes: bytes):
        valid = True
        try:
            magic_number = str(head_bytes[0:5], encoding = "utf-8", errors='ignore')
            if magic_number != 'TSARI':
                self.logger.error("magic number is not match, %s" % (magic_number))
                return '', '', False
            head_bytes = head_bytes[5:]
            version_len = int().from_bytes(head_bytes[0:4], byteorder='big', signed=False)
            head_bytes = head_bytes[4:]
            version = str(head_bytes[0:version_len], encoding = "utf-8", errors='ignore')
            head_bytes = head_bytes[version_len:]

            release_note_len = int().from_bytes(head_bytes[0:4], byteorder='big', signed=False)
            head_bytes = head_bytes[4:]
            release_note = str(head_bytes[0:release_note_len], encoding = "utf-8", errors='ignore')
        except Exception as e:
            self.logger.error(e)
            version, release_note, valid = '', '', False

        return version, release_note, valid

    def upgrade(self, data: bytes):
        if self.upgrade_thread is not None:
            self.upgrade_thread.join()
        self._reset()
        run_cmd('mkdir -p service/upgrade/cache')
        self.upgrade_thread = Thread(target=self._do_upgrade, args=(data, ), daemon=True)
        self.upgrade_thread.start()
        time.sleep(1.0)

    def get_status(self):
        if self.progress >= len(self.upgrade_status):
            self.progress = 0

        status = self.upgrade_status[self.progress]
        percent = self.progress / (len(self.upgrade_status) - 2) * 100
        # if status is Failed, set percent to 0
        if self.progress == self.upgrade_status.index('Failed'):
            percent = 0
        return status, percent

    def get_log(self):
        return self.log_fd.read()

    def _set_status(self, status):
        self.progress = self.upgrade_status.index(status)

    def _reset(self):
        run_cmd('mkdir -p output/logs')
        run_cmd('echo > output/logs/upgrade.log')
        self.logger = create_logger()
        self.log_fd = open('output/logs/upgrade.log', 'r')
        self._set_status('No Upgrade')

    def _do_upgrade(self, data):
        # stage 0
        self.logger.info('downloading...')
        result = self._download_firmware(data)
        if result == False:
            self._set_status('Failed')
            return

        # stage 1
        self.logger.info('pareparing...')
        result = self._upgrade_prepare()
        if result == False:
            self._set_status('Failed')
            return

        # stage 2
        self.logger.info('upgrading...')
        result = self._upgrading()
        if result == False:
            self._set_status('Failed')
            return

        # stage 3
        self.logger.info('verifying...')
        result = self._verifying()
        if result == False:
            self._set_status('Failed')
            return

        # stage 4
        self.logger.info('post processing...')
        result = self._post_process()
        if result == False:
            self._set_status('Failed')
            return

        # stage 5
        self.logger.info('restarting...')
        self._restart_system()

    def _download_firmware(self, data):
        self._set_status('Downloading')
        data = data[4096:]
        signature_len=256
        signature = data[0:signature_len]
        fs = open('service/upgrade/cache/signature.bin', 'wb')
        fs.write(signature)

        data = data[signature_len:]
        f = open('service/upgrade/cache/firmware.zip', 'wb')
        f.write(data)
        return True

    def _upgrade_prepare(self):
        self._set_status('Prepare')
        result = self.generate_and_run_command(command_1)
        if result != 0:
            return False

        return True

    def _upgrading(self):
        self._set_status('Upgrading')
        result = self.generate_and_run_command(command_2)
        if result != 0:
            return False

        return True

    def _verifying(self):
        self._set_status('Verifying')
        result = self.generate_and_run_command(command_3)
        if result != 0:
            return False

        self._merge_config()
        return True

    def _post_process(self):
        self._set_status('Post Process')
        result = self.generate_and_run_command(command_4)
        if result != 0:
            return False

        return True

    def _restart_system(self):
        self._set_status('Success')
        self.generate_and_run_command(command_5)

    def _iterate_update(self, old_dict, new_dict, parent_str):
        for key, value in new_dict.copy().items():
            child_str = parent_str + '.' + str(key)
            if key in old_dict.keys():
                if isinstance(new_dict[key], dict) and isinstance(old_dict[key], dict):
                    new_dict[key] = self._iterate_update(old_dict[key], new_dict[key], child_str)
                elif isinstance(new_dict[key], type(old_dict[key])) and new_dict[key] != old_dict[key]:
                    self.logger.info('update [{}] = {}'.format(child_str, old_dict[key]))
                    new_dict[key] = old_dict[key]
                else:
                    continue
            else:
                continue
        return new_dict

    def _merge_config(self):
        old_config = EasyDict(yaml.safe_load(Path('cfg/board_cfg_all.yaml').open()))
        new_config = EasyDict(yaml.safe_load(Path('service/upgrade/cache/firmware/detection_sys/cfg/board_cfg_all.yaml').open()))
        old_config.lidar_all = new_config.lidar_all
        old_config.radar_all = new_config.radar_all

        merged_config = self._iterate_update(old_config, new_config, 'cfg')
        merged_config = json.loads(json.dumps(merged_config))
        with open('service/upgrade/cache/firmware/detection_sys/cfg/board_cfg_all.yaml', 'w') as f:
            yaml.dump(merged_config, f)

if __name__ == '__main__':
    import time
    # load firmware
    upgrade_service = Upgrade()
    upgrade_service.get_current_version()

    f = open('/home/liangwang/Downloads/detection_sys.bin', 'rb')
    head_bytes = f.read(4096)
    print(upgrade_service.get_firmware_header(head_bytes))

    firmware_data = f.read()

    upgrade_service.upgrade(firmware_data)
    time.sleep(100)
