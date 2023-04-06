# Nvidia Board Setting

This document describes the dependency installation of nvidia embedded board.

Xavier-NX, Xavier-AGX and AGX Orin 32GB with JetPack5.0.2 are tested.

# Requirements

JetPack 5.0.2 (Ubuntu 20.04)

Python 3.8.10, CUDA 11.4.14, CUDNN 8.4.1, TensorRT 8.4.1

Eigen 3.3.7, Ceres 1.14.0, Protobuf 3.8.0, NLOPT 2.4.2, G2O, OpenCV 4.5.5, PCL 1.9.1, GTSAM 4.0

## 1.Set default python version to python3.8
```bash
sudo update-alternatives --install /usr/bin/python python /usr/bin/python3.8 1
sudo update-alternatives --install /usr/bin/pip pip /usr/bin/pip3 1
```

## 2. Apt install libs
```bash
sudo apt install libboost-dev libboost-filesystem-dev libboost-date-time-dev libboost-iostreams-dev
sudo apt install libopenmpi-dev libtbb-dev libopenblas-dev busybox cmake
sudo apt install gfortran libjpeg-dev libgeos-dev libturbojpeg clang-12 libelf-dev
sudo apt install libzmq3-dev gpsd libflann-dev libqhull-dev libgps-dev libgstrtspserver-1.0-dev
sudo apt install libeigen3-dev libceres-dev

sudo add-apt-repository ppa:borglab/gtsam-release-4.0
sudo apt install libgtsam-dev libgtsam-unstable-dev
```

## 3. Install python libs
```bash
sudo rm -rf /usr/lib/python3/dist-packages/numpy*
sudo rm -rf /usr/lib/python3/dist-packages/scipy*
sudo pip install Cython
sudo pip install numpy==1.23.4
sudo pip install flask flask_cors colorlog pycuda easydict pybind11 pyinstaller json-rpc
sudo pip install protobuf==3.16.0
sudo pip install scipy filterpy shapely python-can onnx gevent websockets zerorpc flask_socketio gevent-websocket
```

## 4. Install prebuilt libs
```bash
git clone https://github.com/w111liang222/LSD-Nvidia-Libs.git
cd LSD-Nvidia-Libs
sudo ./install.sh # need to run this script twice
```

## 5. Clone this repository and build the source code
```bash
git clone https://github.com/w111liang222/lidar-slam-detection.git
cd lidar-slam-detection/
unzip slam/data/ORBvoc.zip -d slam/data/

sudo tools/scripts/install_dependency.sh
sudo python setup.py install
```

# TensorRT engine

If you want to run object detection on nvidia board, the pre-generated tensorrt engine files are provided.

You can download the trt files from the following links:

- Xavier NX [Google Drive](https://drive.google.com/file/d/1vXFOs_7U645ePXokTHyGDuZOUk_crOLc/view?usp=sharing) | [百度网盘(密码b75q)](https://pan.baidu.com/s/1hh4VdvOKtW8Q29U7eU2zVg)
- Xavier AGX [Google Drive](https://drive.google.com/file/d/1TTrUVVcYJD8C3NmSpiKV7MOwp_1Z1mYa/view?usp=sharing) | [百度网盘(密码339b)](https://pan.baidu.com/s/1GrnJI2X7JZeuiccE0Wa4-w)
- AGX Orin [Google Drive](https://drive.google.com/file/d/18CqRQfm9Z_sx6gMfDSzSHu5Y2BE00hix/view?usp=sharing) | [百度网盘(密码jir9)](https://pan.baidu.com/s/1WQN6Xcs_z_86DHXkqhgl4w)

Uncompress the zip file and move all *.trt files (detection_image.trt, detection_lidar_l.trt and detection_lidar_s.trt) to **lidar-slam-detection/ sensor_inference/**