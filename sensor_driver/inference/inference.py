import numpy as np

try:
    from inference_ext import inference_init as inference_init_impl
    from inference_ext import inference_forward as inference_forward_impl
    from inference_ext import inference_reset as inference_reset_impl
except Exception as e:
    print(f"WARN: Inference engine is unavailable: {str(e)}")
    def inference_init_impl(*args, **kwargs):
        return
    def inference_forward_impl(*args, **kwargs):
        return
    def inference_reset_impl(*args, **kwargs):
        return

def inference_init(scn_file, rpn_file, voxel_size, coors_range, max_points, max_voxels, max_points_use, frame_num):
    inference_init_impl(scn_file, rpn_file, voxel_size, coors_range, max_points, max_voxels, max_points_use, frame_num)

def inference_forward(points, motion_t, realtime):
    cls_preds, box_preds, label_preds, freespace = np.zeros((2048), dtype=np.float32), np.zeros((2048, 7), dtype=np.float32), np.zeros((2048), dtype=np.int32), np.ones((2, 200, 200), dtype=np.float32)
    inference_forward_impl(points, motion_t, realtime, cls_preds, box_preds, label_preds, freespace)
    return cls_preds, box_preds, label_preds, freespace

def inference_reset():
    return inference_reset_impl()