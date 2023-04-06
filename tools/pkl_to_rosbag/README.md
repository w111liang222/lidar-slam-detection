# Pickle to rosbag

Convert the pickle files which are recorded by LSD to rosbag

## Build

This tool should be built on a ROS installed PC (NOT inside the provided test docker)

```shell
cd tools/pkl_to_rosbag/
mkdir build && cd build
cmake ..
make
```

The executable file named **pkl_to_rosbag** will be generated in the **build** folder.

## Usage

```
-i [path with .pkl files]
-o [path to the rosbag.bag file]
```

## Sample

```shell
./pkl_to_rosbag -i ./demo_data/ -o ./demo.bag
```



