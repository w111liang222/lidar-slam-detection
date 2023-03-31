import sys
import os
import glob
import numpy as np
import argparse
from pathlib import Path
import time
sys.path.append(os.getcwd())
sys.path.append(os.getcwd()+'/sensor_inference')

import time

from sensor_inference import infer
from util.log import get_logger
from utils.config import cfg, cfg_from_yaml_file

def parse_config():
    parser = argparse.ArgumentParser(description='arg parser')
    parser.add_argument('--cfg_file', type=str, default=None,
                        help='specify the config for demo')

    args = parser.parse_args()
    cfg_from_yaml_file(args.cfg_file, cfg)
    return args, cfg

def main():
    args, cfg = parse_config()
    logger = get_logger()

    engine = infer.DetInfer(logger = logger, image_cfg_file=args.cfg_file, serialize_engine=True)
    engine.initialize()

    engine.prepare_image()
    engine.image_engine.ctx.pop()

if __name__ == '__main__':
    main()