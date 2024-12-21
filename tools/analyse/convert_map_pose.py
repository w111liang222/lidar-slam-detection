import os
import sys

sys.path.append(os.getcwd())

import functools
print = functools.partial(print, flush=True)

import math
import yaml
import argparse
import numpy as np
from pathlib import Path
from scipy.spatial.transform import Rotation as R
from sensor_driver.common_lib.cpp_utils import get_transform_from_cfg, get_cfg_from_transform, get_projection_backward
import slam_wrapper as slam

def to_matrix(odom):
    matrix = np.eye(4, dtype=np.float64)
    matrix[:3,  3] = odom[1:4]
    matrix[:3, :3] = R.from_quat([odom[4], odom[5], odom[6], odom[7]]).as_matrix()
    return matrix

def to_list(odom):
    rotation = R.from_matrix(odom[:3, :3]).as_quat()
    return [*odom[:3, 3], *rotation]

def odometry_to_map(odom2map, odometrys):
    stamps, poses = [], []
    for i in range(len(odometrys)):
        odom = to_matrix(odometrys[i])
        pose = odom2map.dot(odom)
        stamps.append(odometrys[i][0])
        poses.append(pose)

    return stamps, poses

def parse_graph_pose(file):
    with open(file, "r") as f:
        for line_num, line in enumerate(f):
            temp_data = line.strip().split()
            if line_num == 0:
                pose_timestamp = int(temp_data[1]) + int(temp_data[2]) / 1e9
            if line_num == 2:
                pose_row1 = [float(temp_data[0]), float(temp_data[1]), float(temp_data[2]), float(temp_data[3])]
            if line_num == 3:
                pose_row2 = [float(temp_data[0]), float(temp_data[1]), float(temp_data[2]), float(temp_data[3])]
            if line_num == 4:
                pose_row3 = [float(temp_data[0]), float(temp_data[1]), float(temp_data[2]), float(temp_data[3])]
            if line_num == 5:
                pose_row4 = [float(temp_data[0]), float(temp_data[1]), float(temp_data[2]), float(temp_data[3])]
            if line_num == 11:
                pose_id = int(temp_data[1])

        pose_list = [pose_row1, pose_row2, pose_row3, pose_row4]
        pose_matrix = np.array(pose_list, dtype=np.float64)
    return pose_timestamp, pose_id, pose_matrix

def get_between_odometry(stamp1, stamp2, odometrys):
    prev_odometry_poses = []
    while int(odometrys[0][0] * 1000000) != int(stamp1 * 1000000):
        prev_odometry_poses.append(odometrys[0])
        odometrys = odometrys[1:]

    odometry_poses = []
    while int(odometrys[0][0] * 1000000) != int(stamp2 * 1000000):
        odometry_poses.append(odometrys[0])
        odometrys = odometrys[1:]

    odometry_poses.append(odometrys[0])
    return prev_odometry_poses, odometry_poses, odometrys

def convert(input_path,  output_path):
    print("convert map graph path: " + input_path)
    root_path = Path(input_path).expanduser().absolute()

    # load origin
    origin = np.loadtxt(input_path + "/map_info.txt")

    # load local odometry
    odometrys = np.loadtxt(input_path + "/odometrys.txt")

    # load configs
    config_path = root_path / 'configs'
    config_list = list(map(str, config_path.glob('*.yaml')))
    config_list = sorted(config_list, key=lambda x: int(x.split('/')[-1].replace('.yaml', '')))
    configs = [yaml.safe_load(Path(config_file).open()) for config_file in config_list]

    # load graph poses
    data_list = list(map(str, root_path.glob('**/data')))
    data_list.sort()

    graph_stamp, graph_id, graph_pose = [], [], []
    for file_name in data_list:
        pose_timestamp, pose_id, pose_matrix = parse_graph_pose(file_name)
        graph_stamp.append(pose_timestamp)
        graph_id.append(pose_id)
        graph_pose.append(pose_matrix)

    # split the multi-graph by the delimiter (timestamp == 0)
    multi_graph_stamp, multi_graph_pose, multi_odometrys = [], [], []
    single_graph_stamp, single_graph_pose, single_odometrys = [], [], []
    for i in range(len(graph_stamp)):
        while int(odometrys[0, 0] * 1000000) != int(graph_stamp[i] * 1000000):
            # check the delimiter
            if int(odometrys[0, 0] * 1000000) == 0:
                print("find map splition between: {} <-> {}".format(graph_id[i - 1], graph_id[i]))
                odometrys = odometrys[1:]
                multi_graph_stamp.append(single_graph_stamp)
                multi_graph_pose.append(single_graph_pose)
                multi_odometrys.append(single_odometrys)
                single_graph_stamp, single_graph_pose, single_odometrys = [], [], []
                break

            single_odometrys.append(odometrys[0])
            odometrys = odometrys[1:]

        single_graph_stamp.append(graph_stamp[i])
        single_graph_pose.append(graph_pose[i])

    single_odometrys.extend(odometrys)
    multi_graph_stamp.append(single_graph_stamp)
    multi_graph_pose.append(single_graph_pose)
    multi_odometrys.append(single_odometrys)

    # fusion
    multi_map_poses = []
    for graph_stamp, graph_pose, odometrys in zip(multi_graph_stamp, multi_graph_pose, multi_odometrys):
        map_poses = []
        for i in range(1, len(graph_stamp)):
            prev_odometry_poses, odometry_poses, odometrys = get_between_odometry(graph_stamp[i - 1], graph_stamp[i], odometrys)
            last_odometry_poses = odometrys if i == (len(graph_stamp) - 1) else []

            estimate1, estimate2 = graph_pose[i - 1], graph_pose[i]
            odom1, odom2         = to_matrix(odometry_poses[0]), to_matrix(odometry_poses[-1])
            odom2map1 = estimate1.dot(np.linalg.inv(odom1))
            odom2map2 = estimate2.dot(np.linalg.inv(odom2))

            # odometry to map
            prev_odometry_stamps, prev_odometry_map_poses = odometry_to_map(odom2map1, prev_odometry_poses)
            odometry_stamps, odometry_map_poses           = odometry_to_map(odom2map1, odometry_poses)
            last_odometry_stamps, last_odometry_map_poses = odometry_to_map(odom2map2, last_odometry_poses)

            # align the pose
            odometry_map_poses = slam.align_pose(graph_stamp[i - 1], graph_stamp[i], estimate1, estimate2, odometry_stamps, odometry_map_poses)

            # concat pose
            odometry_stamps = prev_odometry_stamps + odometry_stamps[:-1] + last_odometry_stamps
            odometry_map_poses = prev_odometry_map_poses + odometry_map_poses[:-1] + last_odometry_map_poses

            # combine the timestamp and pose
            for j in range(len(odometry_stamps)):
                map_poses.append([odometry_stamps[j], odometry_map_poses[j]])

        multi_map_poses.append(map_poses)

    # save to tum/gps txt
    assert len(multi_map_poses) == len(configs)
    for i, map_poses in enumerate(multi_map_poses):
        config           = configs[i]
        dataset          = config["input"]["data_path"].split('/')[-1]
        static_transform = get_transform_from_cfg(*config["ins"]["extrinsic_parameters"])
        output_file      = output_path + f"/{i}_tum_{dataset}.txt"

        print("save output to: ", output_file)
        with open(output_file, "w") as f_out:
            for map_pose in map_poses:
                data = [map_pose[0], *to_list(map_pose[1].dot(static_transform))]
                data = np.array(data).reshape(1, -1)
                np.savetxt(f_out, data, fmt='%1.6f')

        if abs(origin[0]) < 1e-4 and abs(origin[1]) < 1e-4 and abs(origin[2]) < 1e-4:
            continue

        utm_zone_width = 6
        utm_project_no = math.floor(origin[1] / utm_zone_width)
        longitude0 = utm_project_no * utm_zone_width + utm_zone_width / 2

        output_file = output_path + f"/{i}_gps_{dataset}.txt"
        print("save output to: ", output_file)
        with open(output_file, "w") as f_out:
            for map_pose in map_poses:
                pose = map_pose[1].dot(static_transform)
                lat, lon = get_projection_backward(origin[0], origin[1], pose[0, 3], pose[1, 3])
                alt = pose[2, 3] + origin[2]
                [x, y, z, roll, pitch, yaw] = get_cfg_from_transform(pose)
                if abs(roll) >= 90.0 or abs(pitch) >= 90:
                    [x, y, z, roll, pitch, yaw] = get_cfg_from_transform(get_transform_from_cfg(x, y, z, roll, pitch, -yaw))
                else:
                    yaw = -yaw

                grid_convergence = math.atan(math.tan((lon - longitude0) / 180.0 * math.pi) * math.sin(lat / 180.0 * math.pi)) * 180.0 / math.pi
                yaw = yaw + grid_convergence
                yaw = yaw + 360 if yaw < 0 else yaw
                data = [map_pose[0], lat, lon, alt, roll, pitch, yaw]
                data = np.array(data).reshape(1, -1)
                np.savetxt(f_out, data, fmt='%1.6f %1.10f %1.10f %1.10f %1.10f %1.10f %1.10f')

def main():
    parser = argparse.ArgumentParser(description='convert keyframe pose in graph to tum txt')
    parser.add_argument('-i', '--input', required=True, help='Input path for map data path, to "graph"')
    parser.add_argument('-o', '--output', required=True, help='Output path for save')
    args = parser.parse_args()
    convert(args.input, args.output)

if __name__ == "__main__":
    main()
