import sys
import os
import glob
import numpy as np
import argparse
from pathlib import Path
import time
sys.path.append(os.getcwd())
sys.path.append(os.getcwd()+'/sensor_inference')

from sensor_driver.lidar_driver import lidar_driver
from sensor_inference.dataset.demo_dataset import RealtimeDataset
from sensor_inference.model.post_process import PostProcesser
from sensor_inference import infer
from trtcalibrator import DetNetEntropyCalibrator
from util.log import get_logger
from utils.config import cfg, cfg_from_yaml_file

def get_data_list(root_path):
    ext = '.bin'
    data_file_list = glob.glob(str(root_path / f'*{ext}')) if root_path.is_dir() else [root_path]
    data_file_list.sort()
    return data_file_list

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
    data_file_list = get_data_list(Path('/home/liangwang/shared/data/ICRA2020/2020-08-11-13-43-28-small-loop/Ouster128'))

    lidar = lidar_driver.LidarDriver('Ouster-OS1-128', 7502, token = '0',
                                      ex_param=[0, 0, 1.66, 0, 0.77, -2.91],
                                      points_range=[-40, -40, -2, 40, 40, 4],
                                      logger = logger)
    lidar.open()
    lidar.start()

    engine = infer.DetInfer(logger = logger, lidar_cfg_file=args.cfg_file, serialize_engine=True)
    engine.initialize()

    # calib = DetNetEntropyCalibrator(1, data_file_list.copy(), lidar, engine.dataset)

    engine.prepare_lidar()
    engine.lidar_engine.ctx.pop()

    lidar.stop()
    lidar.close()

if __name__ == '__main__':
    main()