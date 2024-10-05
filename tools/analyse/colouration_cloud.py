import os
import sys

sys.path.append(os.getcwd())

import yaml
import argparse
import pickle
import cv2
import numpy as np
from pathlib import Path

from util.common_util import decode_image_jpeg
import slam_wrapper as slam

LIDAR_NAME = '0-Custom'
CAMERA_NAME = []

def enhance_contrast(images):
    for name, img in images.items():
        lab = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)
        l_channel, a, b = cv2.split(lab)
        # Applying CLAHE to L-channel
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
        cl = clahe.apply(l_channel)
        # merge the CLAHE enhanced L-channel with the a and b channel
        limg = cv2.merge((cl, a, b))
        # Converting image from LAB Color model to BGR color spcae
        images[name] = cv2.cvtColor(limg, cv2.COLOR_LAB2BGR)
    return images

def parse_pickle(file_name):
    f = open(file_name, 'rb', buffering=10*1024*1024)
    data_dict = pickle.loads(f.read())
    # decode
    data_dict['image_jpeg'] = dict()
    for name, img in data_dict['image'].items():
        data_dict['image_jpeg'][name] = np.frombuffer(data_dict['image'][name], dtype=np.uint8)
    data_dict['image'] = decode_image_jpeg(data_dict['image_jpeg'].copy())
    # data_dict['image'] = enhance_contrast(data_dict['image'])

    return data_dict

def read_data(file_name):
    if file_name.endswith('.pkl'):
        return parse_pickle(file_name)
    raise Exception("no support file extension")

def filter_cloud(points, attr, points_range, exclude):
    x_idx  = np.bitwise_and(points[:, 0] < points_range[3], points[:, 0] > points_range[0])
    y_idx  = np.bitwise_and(points[:, 1] < points_range[4], points[:, 1] > points_range[1])
    z_idx  = np.bitwise_and(points[:, 2] < points_range[5], points[:, 2] > points_range[2])

    ex_idx = np.bitwise_and(points[:, 0] < exclude[3], points[:, 0] > exclude[0])
    ey_idx = np.bitwise_and(points[:, 1] < exclude[4], points[:, 1] > exclude[1])
    ez_idx = np.bitwise_and(points[:, 2] < exclude[5], points[:, 2] > exclude[2])
    xy_idx = ~(ex_idx & ey_idx & ez_idx) & x_idx & y_idx & z_idx

    points = points[xy_idx]
    attr['points_attr'] = attr['points_attr'][xy_idx]
    return points, attr

def filter_sensor_data(input_dict):
    sensor_input = [LIDAR_NAME, *CAMERA_NAME]
    points_list, points_attr_list = dict(), dict()
    images_list, images_jpeg_list = dict(), dict()
    for sensor in sensor_input:
        if sensor in input_dict['points']:
            points_list[sensor] = input_dict['points'][sensor]
            points_attr_list[sensor] = input_dict['points_attr'][sensor]
        elif input_dict['image_valid'] and sensor in input_dict['image'] and sensor in input_dict['image_jpeg']:
            images_list[sensor] = input_dict['image'][sensor]
            images_jpeg_list[sensor] = input_dict['image_jpeg'][sensor]
    return points_list, points_attr_list, images_list, images_jpeg_list

def run(data_path, pose_path, output):
    # glob all pkl files
    root_path = Path(data_path).expanduser().absolute()
    pkl_data_list = list(map(str, root_path.glob('*.pkl')))

    data_list = []
    if len(pkl_data_list) != 0:
        data_list = pkl_data_list
        data_list.sort()
        config = yaml.safe_load((root_path / 'cfg.yaml').open())
        config_lidar = config["lidar"][int(LIDAR_NAME[0])]
        config_camera = config["camera"]

    if len(data_list) <= 0:
        return

    print("Start to colouration {} frames to rgb map".format(len(data_list)))

    # setup configs
    slam.set_colouration_config(config_camera)

    # load odometry
    odometrys = np.loadtxt(pose_path)
    odometrys = odometrys.astype(np.float64)
    odometrys[:, 0] = odometrys[:, 0] * 1000000
    slam.set_map_odometrys(odometrys)

    # process frames
    for i in range(0, len(data_list)):
        data = read_data(data_list[i])
        # preprocess sensor data
        data['points'][LIDAR_NAME], data['points_attr'][LIDAR_NAME] = filter_cloud(data['points'][LIDAR_NAME], data['points_attr'][LIDAR_NAME], config_lidar["range"], config_lidar["exclude"])
        points_list, points_attr_list, images_list, images_jpeg_list = filter_sensor_data(data)

        # colouration
        slam.colouration_frame(LIDAR_NAME, points_list, points_attr_list, images_list, images_jpeg_list, data['image_param'])
        if (i % 100) == 0:
            print("process progress: {} / {}".format(i, len(data_list)))

    output_file = "{}/render_map.pcd".format(output)
    print("save output to: ", output_file)
    slam.save_render_cloud(output_file)

def main():
    parser = argparse.ArgumentParser(description='accumulate single frame into a map')
    parser.add_argument('-i', '--data_path', required=True, help='single frame data path')
    parser.add_argument('-p', '--pose_path', required=True, help='pose data path')
    parser.add_argument('-o', '--output', required=True, help='output path for save')
    args = parser.parse_args()
    run(args.data_path, args.pose_path, args.output)

if __name__ == "__main__":
    main()