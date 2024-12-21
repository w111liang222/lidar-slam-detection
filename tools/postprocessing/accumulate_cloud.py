import os
import sys

sys.path.append(os.getcwd())

import yaml
import copy
import argparse
import pickle
import numpy as np
from pathlib import Path

import slam_wrapper as slam

LIDAR_NAME = '0-Custom'

def parse_pickle(file_name):
    f = open(file_name, 'rb', buffering=10*1024*1024)
    data_dict = pickle.loads(f.read())
    imu_valid = not (data_dict['imu_data'].shape[0] < 2)
    data_valid = imu_valid
    return data_valid, data_dict

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

def get_between_odometry(stamp1, stamp2, odometrys):
    if odometrys.shape[0] <= 0 or stamp1 < odometrys[0, 0]:
        return np.zeros((0, 8)), odometrys

    while odometrys.shape[0] > 0 and int(odometrys[0, 0]) != int(stamp1):
        odometrys = odometrys[1:]

    odometry_poses = []
    while odometrys.shape[0] > 0 and int(odometrys[0, 0]) != int(stamp2):
        odometry_poses.append(odometrys[0])
        odometrys = odometrys[1:]

    if odometrys.shape[0] <= 0:
        return np.zeros((0, 8)), odometrys

    odometry_poses.append(odometrys[0])
    return np.array(odometry_poses), odometrys

def data_loader(data_list, odometrys):
    _, prev_data = read_data(data_list[0])
    for i in range(1, len(data_list)):
        data_valid, curr_data = read_data(data_list[i])
        if not data_valid:
            continue

        # get data
        stamp1      = prev_data['points_attr'][LIDAR_NAME]['timestamp']
        stamp2      = curr_data['points_attr'][LIDAR_NAME]['timestamp']
        points      = prev_data['points'][LIDAR_NAME]
        points_attr = prev_data['points_attr'][LIDAR_NAME]
        prev_data   = curr_data

        odometry_poses, odometrys = get_between_odometry(stamp1, stamp2, odometrys)
        if odometrys.shape[0] <= 0:
            break
        if odometry_poses.shape[0] <= 0:
            continue

        assert odometry_poses[0,  0] == stamp1
        assert odometry_poses[-1, 0] == stamp2
        yield points, points_attr, odometry_poses, i

def run(data_path, pose_path, resolution, output, args):
    # glob all pkl files
    root_path = Path(data_path).expanduser().absolute()
    pkl_data_list = list(map(str, root_path.glob('*.pkl')))

    data_list = []
    if len(pkl_data_list) != 0:
        data_list = pkl_data_list
        data_list.sort()
        config = yaml.safe_load((root_path / 'cfg.yaml').open())
        config_lidar = config["lidar"][int(LIDAR_NAME[0])]
        config_lidar["range"] = [-args.distance, -args.distance, args.z_min, args.distance, args.distance, args.z_max]

    if len(data_list) <= 0:
        return

    print("LiDAR config: \n", config_lidar)
    print("Start to accumulate {} frames to dense map".format(len(data_list)))

    # load odometry
    odometrys = np.loadtxt(pose_path)
    odometrys = odometrys.astype(np.float64)
    odometrys[:, 0] = odometrys[:, 0] * 1000000
    odomtry_type = "TUM"

    # process frames
    for points, points_attr, odometry_poses, i in data_loader(data_list, copy.deepcopy(odometrys)):
        points, points_attr = filter_cloud(points, points_attr, config_lidar["range"], config_lidar["exclude"])
        slam.accumulate_cloud(points, points_attr, odometry_poses, odomtry_type, args.ground)

    slam.save_accumulate_cloud("{}/dense_map_{}.pcd".format(output, i), resolution)

def main():
    parser = argparse.ArgumentParser(description='accumulate single frame into a map')
    parser.add_argument('-i', '--data_path', required=True, help='single frame data path')
    parser.add_argument('-p', '--pose_path', required=True, help='pose data path')
    parser.add_argument('-r', '--resolution', default=0.0, type=float, help='downsample resolution of cloud')
    parser.add_argument('-d', '--distance', default=200.0, type=float, help='max distance of single frame')
    parser.add_argument('-zl','--z_min', default=-0.5, type=float, help='z min of single frame')
    parser.add_argument('-zh','--z_max', default=100.0, type=float, help='z max of single frame')
    parser.add_argument('-g', '--ground', default=False, action='store_true', help='whether to extract the ground')
    parser.add_argument('-o', '--output', required=True, help='Output path for save')
    args = parser.parse_args()
    run(args.data_path, args.pose_path, args.resolution, args.output, args)

if __name__ == "__main__":
    main()