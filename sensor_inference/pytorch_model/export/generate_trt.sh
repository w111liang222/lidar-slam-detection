#!/bin/bash -e

# object model
/usr/src/tensorrt/bin/trtexec --onnx=sensor_inference/detection_lidar_rpn.onnx \
                              --saveEngine=sensor_inference/detection_lidar_rpn.trt \
                              --memPoolSize=workspace:4096 --fp16 \
                              --inputIOFormats=fp16:chw --verbose --dumpLayerInfo \
                              --dumpProfile --separateProfileRun \
                              --profilingVerbosity=detailed

echo "generate trt success"