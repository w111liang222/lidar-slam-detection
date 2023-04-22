# LSD (LiDAR SLAM & Detection)

LSD is an open source perception architecture for autonomous vehicle and robotics.

<img src="docs/home.gif" width="720pix" />

LSD currently supports many features:

- [x] support multiple LiDAR, camera, radar and INS/IMU sensors.
- [x] support user-friendly calibration for LiDAR and camera etc.
- [x] support software time sync, data record and playback.
- [x] support CNN based pointcloud object detection, tracking and prediction.
- [x] support GICP, FLOAM and FastLIO based frontend odometry and G2O based pose graph optimization.
- [x] support Web based interactive map correction tool(editor).
- [x] support communication with [ROS](#ros).

# Overview

- [Quick Demo](docs/demo/README.md)
- [Architecture](docs/architecture.md)

# Prerequisites

LSD can be worked both on x86 PC and Nvidia embedded boards (Xavier-NX, Xavier-AGX and Orin).

### Basic Enviroment

Ubuntu20.04, Python3.8, Eigen 3.3.7, Ceres 1.14.0, Protobuf 3.8.0, NLOPT 2.4.2, G2O, OpenCV 4.5.5, PCL 1.9.1, GTSAM 4.0

### Nvidia Embedded Board

The LSD is tested on Xavier-NX, Xavier-AGX and Orin with JetPack5.0.2, [Installation](docs/nvidia/README.md)

# Getting Started

A x86_64 docker image is provided to test.
```bash
docker pull 15liangwang/auto-ipu
sudo docker run -it -d --net=host --privileged --shm-size=4g --name="LSD" -v /media:/root/exchange 15liangwang/auto-ipu
docker exec -it LSD /bin/bash
```

Clone this repository and build the source code
```bash
cd /home/znqc/work/
git clone https://github.com/w111liang222/lidar-slam-detection.git
cd lidar-slam-detection/
unzip slam/data/ORBvoc.zip -d slam/data/
python setup.py install
```

Run LSD
```bash
tools/scripts/start_system.sh
```
Open http://localhost (or http://localhost:1234) in your browser, e.g. Chrome, and you can see this screen.

<img src="docs/home.png" width="720pix" />


## Example Data

Download the demo data [Google Drive](https://drive.google.com/file/d/1wi3KATudMX3b4Wz0Bu-qcScaFuQDvXwW/view?usp=sharing) | [百度网盘(密码sk5h)](https://pan.baidu.com/s/1N7-w-Ls294MzfvX2X866Uw) and unzip it

```bash
unzip demo_data.zip -d /home/znqc/work/
tools/scripts/start_system.sh # re-run LSD
```

More usages can be found [here](docs/guide.md)

# ROS

LSD is NOT built on the Robot Operating System (ROS), but we provides some tools to bridge the communication with ROS.

- [rosbag proxy](tools/rosbag_proxy/README.md): a tool which send the ros topic data to LSD.
- [pickle to rosbag](tools/pkl_to_rosbag/README.md): a convenient tool to convert the pickle files which are recorded by LSD to rosbag.

# License

LSD is released under the [Apache 2.0 license](LICENSE).

# Acknowledgments

In the development of LSD, we stand on the shoulders of the following repositories:

- [lidar_align](https://github.com/ethz-asl/lidar_align): A simple method for finding the extrinsic calibration between a 3D lidar and a 6-dof pose sensor.
- [lidar_imu_calib](https://github.com/chennuo0125-HIT/lidar_imu_calib): automatic calibration of 3D lidar and IMU extrinsics.
- [OpenPCDet](https://github.com/open-mmlab/OpenPCDet): OpenPCDet Toolbox for LiDAR-based 3D Object Detection.
- [AB3DMOT](https://github.com/xinshuoweng/AB3DMOT): 3D Multi-Object Tracking: A Baseline and New Evaluation Metrics.
- [FAST-LIO](https://github.com/hku-mars/FAST_LIO): A computationally efficient and robust LiDAR-inertial odometry package.
- [FLOAM](https://github.com/wh200720041/floam): Fast and Optimized Lidar Odometry And Mapping for indoor/outdoor localization.
- [hdl_graph_slam](https://github.com/koide3/hdl_graph_slam): an open source ROS package for real-time 6DOF SLAM using a 3D LIDAR.
- [hdl_localization](https://github.com/koide3/hdl_localization): Real-time 3D localization using a (velodyne) 3D LIDAR.
- [ORB_SLAM2](https://github.com/raulmur/ORB_SLAM2): Real-Time SLAM for Monocular, Stereo and RGB-D Cameras, with Loop Detection and Relocalization Capabilities.
- [scancontext](https://github.com/irapkaist/scancontext): Global LiDAR descriptor for place recognition and long-term localization.

# Citation
If you find this project useful in your research, please consider cite and star this project:

```
@misc{LiDAR-SLAM-Detection,
    title={LiDAR SLAM & Detection: an open source perception architecture for autonomous vehicle and robotics},
    author={LiangWang},
    howpublished = {\url{https://github.com/w111liang222/lidar-slam-detection}},
    year={2023}
}
```

# Contact

LiangWang 15lwang@alumni.tongji.edu.cn