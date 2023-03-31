#!/bin/bash -e

set -o xtrace

echo "start build release"

# get version
version=`cat VERSION`
commit=`git rev-parse --short HEAD`

rm -rf ../release && mkdir -p ../release && cd ../release

#copy the develop git
rsync -rtl --exclude=*.git ../detection_sys/* detection_sys_release
cd detection_sys_release

# sensor_inference release post-process
rm -rf sensor_inference/detection_image.onnx
rm -rf sensor_inference/detection_lidar_l.onnx
rm -rf sensor_inference/detection_lidar_s.onnx

mv sensor_inference/cfgs/detection_image_release.yaml sensor_inference/cfgs/detection_image.yaml
mv sensor_inference/cfgs/detection_lidar_large_release.yaml sensor_inference/cfgs/detection_lidar_large.yaml
mv sensor_inference/cfgs/detection_lidar_small_release.yaml sensor_inference/cfgs/detection_lidar_small.yaml

python tools/build/prepare_inference_trt.py
BOARD_NAME=`cat /tmp/BOARD_NAME`
JETPACK=`cat /tmp/JETPACK`

# clean image model
rm -rf sensor_inference/image_model/darknet.py
rm -rf sensor_inference/image_model/image_detection.py
rm -rf sensor_inference/image_model/network_blocks.py
rm -rf sensor_inference/image_model/rtm3d_head.py
rm -rf sensor_inference/image_model/yolo_pafpn.py
rm -rf sensor_inference/image_pytorch2onnx.py

# clean lidar model
rm -rf sensor_inference/model/center_point_head.py
rm -rf sensor_inference/model/CSPDarknet53Large.py
rm -rf sensor_inference/model/CSPDarknet53Small.py
rm -rf sensor_inference/model/multi_center_point_head.py
rm -rf sensor_inference/model/pillar_vfe.py
rm -rf sensor_inference/model/point_pillar.py
rm -rf sensor_inference/model/pointpillar_scatter.py
rm -rf sensor_inference/lidar_pytorch2onnx.py

# clean other scripts
rm -rf sensor_inference/pytorch2onnx.sh
rm -rf sensor_inference/data
rm -rf sensor_inference/trt_engine/prepare_engine.py
rm -rf sensor_inference/trt_engine/TensorRT7_1/prepare_engine.py
rm -rf sensor_inference/trt_engine/TensorRT8_2/prepare_engine.py
rm -rf sensor_inference/trt_engine/TensorRT8_4/prepare_engine.py
rm -rf sensor_inference/utils/visualize_utils.py

rm -rf tests/
rm -rf README.md

# commit & tag & push
if [ -z "$1" ]
then
    message="$version"
else
    message="$1"
fi
git init
git add .
git commit -m "${message}" >> /dev/null || true
git tag -f ${version} || true

# package to firmware
cd ..

cp -rd detection_sys_release detection_sys
cd detection_sys
rm -rf .git
echo "-${commit}" >> VERSION
./tools/build/build_module.sh

if [ "$NUITKA_COMPILE" == "True" ];then
    echo "build nuitka binary"
    suffix="release"
    export PYTHONPATH=$PYTHONPATH:`pwd`
    python -m nuitka --jobs=4 --remove-output --show-modules --prefer-source-code --clang --lto=yes --follow-imports --nofollow-import-to=flask --nofollow-import-to=flask_cors --nofollow-import-to=zerorpc --nofollow-import-to=jsonrpc --nofollow-import-to=can --nofollow-import-to=pycuda --nofollow-import-to=tensorrt --nofollow-import-to=onnxruntime --nofollow-import-to=cv2 --nofollow-import-to=onnx --nofollow-import-to=numpy --nofollow-import-to=scipy --nofollow-import-to=gevent --nofollow-import-to=six --nofollow-import-to=requests --nofollow-import-to=apport --nofollow-import-to=third_party web_backend/server.py

    rm -rf CMakeLists.txt setup.py
    rm -rf calibration/
    rm -rf freespace_detection/
    rm -rf hardware/gstreamer
    rm -rf hardware/nvivafilter/gst-custom-opencv_cudaprocess.cu
    rm -rf hardware/nvivafilter/Makefile
    rm -rf module/
    rm -rf proto/
    rm -rf sensor_driver/
    rm -rf sensor_fusion/
    # clean sensor_inference exclude cfgs and trt/onnx files
    if [ "$BOARD_NAME" == "IPC" ];then
        engine_suffix="*.onnx"
    else
        engine_suffix="*.trt"
    fi
    mv sensor_inference/${engine_suffix} ./ && mv sensor_inference/cfgs ./
    rm -rf sensor_inference/ && mkdir sensor_inference
    mv ./${engine_suffix} sensor_inference/ && mv ./cfgs sensor_inference/

    rm -rf service/time_sync/gpio_tigger/gpio_trigger.c
    rm -rf service/time_sync/gpio_tigger/Makefile
    rm -rf service/time_sync/pps/pps_generator.c
    rm -rf service/time_sync/pps/Makefile

    mv slam/data ./slam_data
    rm -rf slam/ && mkdir slam
    mv ./slam_data slam/data

    rm -rf third_party/pybind11
    rm -rf third_party/readerwriterqueue
    rm -rf third_party/launch_rtsp_server.c

    rm -rf util/box_utils.py
    rm -rf util/image_util.py
    rm -rf util/period_calculator.py

    rm -rf web_backend/

    # compile the upgrade service
    python -m nuitka --jobs=4 --remove-output --show-modules --prefer-source-code --clang --lto=yes --follow-imports --nofollow-import-to=flask --nofollow-import-to=flask_cors --nofollow-import-to=flask_socketio --nofollow-import-to=gevent --nofollow-import-to=numpy --nofollow-import-to=requests --nofollow-import-to=apport --nofollow-import-to=third_party service/upgrade/upgrade_daemon.py -o service/upgrade/upgrade_daemon.bin
    rm -rf service/upgrade/scripts
    rm -rf service/upgrade/upgrade_daemon.py
    rm -rf service/upgrade/upgrade.py
else
    suffix="debug"
fi

cd ..
7za a -tzip -pCrvsVM4z5ReV -mem=AES256 -mx=9 firmware.zip detection_sys/*
openssl sha1 -out digest.txt firmware.zip
openssl rsautl -sign -inkey /root/rsaprivatekey.pem -in digest.txt -out signature.bin

cd detection_sys_release
python tools/build/append_header.py --input ../firmware.zip --signature ../signature.bin --out ../${version}-${commit}-${BOARD_NAME}-${JETPACK}-${suffix}.bin
rm -rf ../signature.bin

mkdir -p ../../detection_sys/firmware
cp ../${version}-${commit}-${BOARD_NAME}-${JETPACK}-${suffix}.bin ../../detection_sys/firmware/

cd ../../detection_sys
