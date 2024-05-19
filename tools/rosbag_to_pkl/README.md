# Rosbag to pickle

This tool convert the rosbag to pickle files

## Build and Run

This tool should be built on a ROS installed PC (NOT inside the provided test docker)

```shell
cd tools/rosbag_to_pkl/
mkdir build && cd build && cmake ..
make
```

## Convert rosbag

You should modify the items in **tools/rosbag_to_pkl/config.yaml** to match your bags.
```shell
./rosbag_to_pkl
```
the converted data will be found in **pickle_path**

## Run LSD

Modiy the **input->data_path** with the **pickle_path** in **lidar-slam-detection/cfg/board_cfg_all.yaml**, then start LSD
