import os
import sys

sys.path.append(os.getcwd())

import argparse
import slam_wrapper as slam


def run(mesh_path, cloud_path, output_path):
    slam.texture_mesh(mesh_path, cloud_path, output_path)

def main():
    parser = argparse.ArgumentParser(description='texture mesh with a rgb pointcloud')
    parser.add_argument('-i', '--mesh_path', required=True, help='mesh obj path')
    parser.add_argument('-p', '--cloud_path', required=True, help='rgb pointcloud path')
    parser.add_argument('-o', '--output', required=True, help='Output directory for save')
    args = parser.parse_args()
    run(args.mesh_path, args.cloud_path, args.output)

if __name__ == "__main__":
    main()