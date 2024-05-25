# Running Mode

- Online: receive and process the real-time sensor data
- Offline: read the recorded data and playback

We can switch the running mode easily by clicking the switch buttom at the home page of web UI.

# Processing Mode

Currently, there are four processing modes are supported by LSD, all modes can be running at online or offline.
- Data Acquisition        --- preview the sensor data, sensor calibration or data recording.
- Object Detection        --- cnn based network detection of pointcloud, object tracking, prediction.
- SLAM                    --- pointcloud mapping or localization with LiDAR, camera and IMU.
- Object Detection & SLAM --- running object detetcion and SLAM simultaneously.

These modes can be configured by "Web UI => Config => Device => Mode" and click the "Upload" button to save. (this operation needs to restart LSD manually)

# Sensor Configure

LSD can support various sensors including LiDAR, Camera, Radar, INS/IMU etc.

|  Type   | Sensor                                                | Specification               |
|---------|:------------------------------------------------------|-----------------------------|
| LiDAR   | Ouster 32/64/128, RoboSense 16/32/80, VLP-16, LS-C-16 | 10Hz mode                   |
| Camera  | GMSL, RTSP, UVC                                       | GMSL, UVC: v4l2, RTSP: H264 |
| Radar   | ARS408                                                | Object mode                 |
| INS/IMU | CGI-610(CHCNAV)                                       |                             |

## LiDAR Configure

We can add a LiDAR through "Web UI => Config => LiDAR", then input the LiDAR name and the UDP port (should be configured before)

## Camera Configure

We can add a camera through "Web UI => Config => Camera" and the camera name should be input and follow the rules:
- GMSL, UVC: the index of v4l2 device. example: "Name" should be "0" if the v4l2 device is /dev/video0.
- RTSP: rtsp://username:password@(ipaddress)/(videotype)/ch(number)/(streamtype). example: rtsp://admin:12345@172.6.22.106:554/h264/ch33/main/av_stream

Note: the v4l2 device should be mapped into container firstly when start the docker container (--device=/dev/video0:/dev/video0)

## Radar Configure

We can add a LiDAR through "Web UI => Config => Radar", then input the CAN interface and baud of Radar.

# Data Recording

LSD can support software time syncing for sensors and real-time data recording.

The data is recorded as a python dictionary and serialized to serveral pickle files.

Here is the example data listed as the below table:

|  Data Name            | Type                 | Description                                                               |
|-----------------------|:---------------------|:--------------------------------------------------------------------------|
| frame_start_timestamp | uint64_t             | the start time of frame (us)                                              |
| lidar_valid           | boolean              | indicate the LiDAR data is valid                                          |
| points                | Dict: name - > numpy | pointcloud of LiDAR, key is the sensor name, value is a n x 4 numpy array |
| image_valid           | boolean              | indicate the camera data is valid                                         |
| image                 | Dict: name - > bytes | JPEG image data, key is the sensor name, value is compressed jpeg data    |
| ins_valid             | boolean              | indicate the INS/IMU data is valid                                        |
| ins_data              | Dict                 | store the data of latitude, longitude, altitude, gyro and accelerator etc |

Here are the operation steps.

### Mount a directory into /media

Currently, LSD will automatically detect the external disk which is mounted at **/media** and use it as the storage path to read/write data.

We can simulate an external disk for testing the data recording by following command:

```bash
mount --bind /root/exchange/ /media/external/
```

### Mode Setting

The "Data Acquisition" and "Online" mode should be configured by above step, then check the sensor data through the preview page of web.

### Start Recording

Click the button which is right align to the "Disk" at the home page.

<img src="demo/assets/disk.png" width="360pix" />