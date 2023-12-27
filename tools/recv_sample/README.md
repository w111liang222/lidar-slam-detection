# Sample code for receiving the output of LSD

# Build
```shell
cd tools/recv_sample/
mkdir build && cd build
cmake ..
make -j
```

# RUN
## Receive Object Detection
```shell
./recv_detection_udp
```

## Receive Localization
```shell
./recv_localization_udp
```