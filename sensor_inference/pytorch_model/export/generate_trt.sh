#!/bin/bash -e

# object model
/usr/src/tensorrt/bin/trtexec --onnx=sensor_inference/detection_lidar_rpn.onnx \
                              --saveEngine=sensor_inference/detection_lidar_rpn.trt \
                              --workspace=4096 --fp16 \
                              --inputIOFormats=fp16:chw --verbose --dumpLayerInfo \
                              --dumpProfile --separateProfileRun \
                              --profilingVerbosity=detailed

# traffic light model
/usr/src/tensorrt/bin/trtexec --onnx=sensor_inference/detection_trafficlight.onnx \
                              --saveEngine=sensor_inference/detection_trafficlight.trt \
                              --workspace=4096 --fp16 \
                              --minShapes=images:1x480x640x3 \
                              --optShapes=images:1x704x1280x3 \
                              --maxShapes=images:1x2176x4096x3 \
                              --verbose --dumpLayerInfo \
                              --dumpProfile --separateProfileRun \
                              --profilingVerbosity=detailed

echo "generate trt success"