#!/bin/bash -e

rm -rf replays/ inputs.json golden.json polygraphy_debug.engine

# lidar large model
# python tools/build/onnx2trt_lidar.py --cfg_file sensor_inference/cfgs/detection_lidar_large.yaml
cp sensor_inference/data/lidar_l_inputs.json inputs.json
polygraphy run sensor_inference/detection_lidar_l.onnx --onnxrt --load-inputs inputs.json --save-outputs golden.json
polygraphy debug build sensor_inference/detection_lidar_l.onnx --trt-min-shapes voxel_features:[1,1,20,4] coords:[1,1,3] mask:[1,1,20,1] --trt-opt-shapes voxel_features:[1,20000,20,4] coords:[1,20000,3] mask:[1,20000,20,1] --trt-max-shapes voxel_features:[1,40000,20,4] coords:[1,40000,3] mask:[1,40000,20,1] --tf32 --fp16 --save-tactics replay.json --artifacts-dir replays --artifacts replay.json --until=good --show-output --no-remove-intermediate --check polygraphy run polygraphy_debug.engine --trt --load-inputs inputs.json --load-outputs golden.json --rtol 1e3 cls_preds:1e-1 --atol 1e3 cls_preds:5e-3
mv polygraphy_debug.engine sensor_inference/detection_lidar_l.trt
rm -rf replays/ inputs.json golden.json polygraphy_debug.engine

# lidar small model
# python tools/build/onnx2trt_lidar.py --cfg_file sensor_inference/cfgs/detection_lidar_small.yaml
cp sensor_inference/data/lidar_s_inputs.json inputs.json
polygraphy run sensor_inference/detection_lidar_s.onnx --onnxrt --load-inputs inputs.json --save-outputs golden.json
polygraphy debug build sensor_inference/detection_lidar_s.onnx --trt-min-shapes voxel_features:[1,1,20,4] coords:[1,1,3] mask:[1,1,20,1] --trt-opt-shapes voxel_features:[1,20000,20,4] coords:[1,20000,3] mask:[1,20000,20,1] --trt-max-shapes voxel_features:[1,40000,20,4] coords:[1,40000,3] mask:[1,40000,20,1] --tf32 --fp16 --save-tactics replay.json --artifacts-dir replays --artifacts replay.json --until=good --show-output --no-remove-intermediate --check polygraphy run polygraphy_debug.engine --trt --load-inputs inputs.json --load-outputs golden.json --rtol 1e3 cls_preds:1e-1 --atol 1e3 cls_preds:5e-3
mv polygraphy_debug.engine sensor_inference/detection_lidar_s.trt
rm -rf replays/ inputs.json golden.json polygraphy_debug.engine

# image model
python tools/build/onnx2trt_image.py --cfg_file sensor_inference/cfgs/detection_image.yaml

echo "generate trt success"