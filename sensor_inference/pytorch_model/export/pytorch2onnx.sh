python sensor_inference/pytorch_model/export/spconv_object_pytorch2onnx.py \
    --ckpt /root/OpenPCDet/output/misc_models/tsari_detection_segment_2frames_wnlt/default/ckpt/checkpoint_epoch_45.pth \
    --cfg_file sensor_inference/cfgs/detection_object.yaml \
    --use-quantization
