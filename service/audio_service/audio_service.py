import sys
import os
sys.path.append(os.getcwd())

from pathlib import Path
from easydict import EasyDict
import yaml

from util.common_util import run_cmd

class AudioService:
    def __init__(self):
        self.config = EasyDict(yaml.safe_load(Path('cfg/board_cfg_all.yaml').open()))

    def start(self):
        audio_configs = self.config.board.audio_service
        if audio_configs.capture.use:
            port = audio_configs.capture.port
            name = audio_configs.capture.audio_name
            run_cap = ''' gst-launch-1.0 udpsrc address=127.0.0.1 port={} ! application/x-rtp,media=audio,payload=96,clock-rate=48000,encoding-name=OPUS ! rtpopusdepay ! opusdec ! audioconvert ! alsasink device="plughw:{}" &'''.format(port, name)
            print('run:', run_cap)
            run_cmd(run_cap)

if __name__ == '__main__':
    service = AudioService()
    service.start()
