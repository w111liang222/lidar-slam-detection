# Rosbag proxy

This tool bridges the communication between ROS and LSD, it subscribes the ros topic and send it to LSD.

## Build and Run

This tool should be built on a ROS installed PC (NOT inside the provided test docker)

```shell
cd tools/rosbag_proxy/
catkin_make
source devel/setup.bash
roslaunch rosbag_proxy rosbag_proxy.launch

# Open a new terminal and play the rosbag
rosbag play xxx.bag
```

## Sample

The example rosbag can be downloaded [Google Drive](https://drive.google.com/file/d/1ED7BYVzpvKATieZQetqkcPP395wGwIIV/view?usp=sharing) | [百度网盘(密码8m6n)](https://pan.baidu.com/s/1LZ2KCd2gE5XttUkzfV0C-g)

```shell
# follow the above doc and build the rosbag proxy tool, then roslaunch it
roslaunch rosbag_proxy rosbag_proxy.launch

# Open a new terminal and play the demo rosbag
rosbag play -l demo.bag
```

Replace the cfg/board_cfg_all.yaml with this [cfg](./board_cfg_all.yaml):

```shell
# start the LSD and preview the result at the web.
tools/scripts/start_system.sh
```

## Playback other rosbag

You can modify the **points_topic** and **imu_topic** in **rosbag_proxy.launch** to match your bags.
