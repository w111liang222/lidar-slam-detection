import sys
import os
sys.path.append(os.getcwd())

from pathlib import Path
from easydict import EasyDict
import yaml

from util.common_util import run_cmd

class PtpTimeSync:
    def __init__(self):
        self.config = EasyDict(yaml.safe_load(Path('cfg/board_cfg_all.yaml').open()))

    def start(self):
        ptp_configs = self.config.board.time_sync.ptp
        for ptp_config in ptp_configs:
            if ptp_config.mode == 'master':
                mode = '-M'
            elif ptp_config.mode == 'slave':
                mode = '-s'
            else:
                mode = '-m'
            run_ptp = ''' ptpd -i {} {} -L'''.format(ptp_config.interface, mode)

            print('run:', run_ptp)
            run_cmd(run_ptp)

NTP_CONFIG_TEMPLATE = '''\
{}"refclock SHM 0 refid NMEA" "allow 0.0.0.0/0" "driftfile /var/lib/chrony/drift" "makestep 1 -1" "rtcsync"
'''
class NtpTimeSync:
    def __init__(self):
        self.config = EasyDict(yaml.safe_load(Path('cfg/board_cfg_all.yaml').open()))

    def start(self):
        ntp_configs = self.config.board.time_sync.ntp
        if len(ntp_configs) <= 0:
            run_cmd('timedatectl set-ntp true')
            return

        run_cmd('timedatectl set-ntp false')
        ntp_servers = ''
        for conf in ntp_configs:
            ntp_servers = ntp_servers + '''"server {} iburst" '''.format(conf.server)
        ntp_cfg = NTP_CONFIG_TEMPLATE.format(ntp_servers)
        run_ntp = ''' chronyd {} '''.format(ntp_cfg)
        run_cmd(run_ntp)

class GpsTimeSync:
    def __init__(self):
        self.config = EasyDict(yaml.safe_load(Path('cfg/board_cfg_all.yaml').open()))

    def start(self):
        gps_config = self.config.board.time_sync.gps
        if not gps_config.use:
            return

        run_gpsd = ''' gpsd -n {} /dev/pps1'''.format(gps_config.device)
        run_cmd(run_gpsd)

class PPSOut:
    def __init__(self):
        self.config = EasyDict(yaml.safe_load(Path('cfg/board_cfg_all.yaml').open()))

    def start(self):
        pps_out_configs = self.config.board.time_sync.pps_out
        if pps_out_configs.use:
            run_cmd('rmmod pps_generator > /dev/null 2>&1')
            run_cmd('''echo {} > /tmp/pps_out_gpio'''.format(pps_out_configs.gpio))
            run_cmd('insmod service/time_sync/pps/`uname -r`/pps_generator.ko')
        else:
            run_cmd('rmmod pps_generator > /dev/null 2>&1')

class GpioTrigger:
    def __init__(self):
        self.config = EasyDict(yaml.safe_load(Path('cfg/board_cfg_all.yaml').open()))

    def start(self):
        gpio_trigger_configs = self.config.board.time_sync.gpio_trigger
        if gpio_trigger_configs.use:
            run_cmd('rmmod gpio_trigger > /dev/null 2>&1')
            run_cmd('''echo "{} {}" > /tmp/gpio_trigger'''.format(gpio_trigger_configs.gpio, gpio_trigger_configs.freq))
            run_cmd('insmod service/time_sync/gpio_tigger/`uname -r`/gpio_trigger.ko')
        else:
            run_cmd('rmmod gpio_trigger > /dev/null 2>&1')

if __name__ == '__main__':
    ptp_service = PtpTimeSync()
    ptp_service.start()

    ntp_service = NtpTimeSync()
    ntp_service.start()

    gps_service = GpsTimeSync()
    gps_service.start()

    pps_out = PPSOut()
    pps_out.start()

    gpio_tigger = GpioTrigger()
    gpio_tigger.start()
