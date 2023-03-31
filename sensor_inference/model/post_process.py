import numpy as np
from sensor_inference.utils import model_nms_utils

class PostProcesser():
    def __init__(self, model_cfg, num_class, dataset):
        self.model_cfg = model_cfg
        self.num_class = num_class
        self.dataset = dataset
        self.class_names = dataset.class_names
        self.score_dict = dict()
        for name, thresh in model_cfg.POST_PROCESSING.SCORE_THRESH.items():
            self.score_dict[self.class_names.index(name)+1] = thresh
    
    def forward(self, cls_preds, box_preds, label_preds):
        pred_dicts = self.post_processing(cls_preds, box_preds, label_preds)
        return pred_dicts
    
    def post_processing(self, cls_preds, box_preds, label_preds):
        """
        Args:
            batch_dict:
                batch_size:
                batch_cls_preds: (B, num_boxes, num_classes | 1) or (N1+N2+..., num_classes | 1)
                                or [(B, num_boxes, num_class1), (B, num_boxes, num_class2) ...]
                multihead_label_mapping: [(num_class1), (num_class2), ...]
                batch_box_preds: (B, num_boxes, 7+C) or (N1+N2+..., 7+C)
                cls_preds_normalized: indicate whether batch_cls_preds is normalized
                batch_index: optional (N1+N2+...)
                has_class_labels: True/False
                roi_labels: (B, num_rois)  1 .. num_classes
                batch_pred_labels: (B, num_boxes, 1)
        Returns:

        """
        post_process_cfg = self.model_cfg.POST_PROCESSING

        final_scores, final_boxes, final_labels = model_nms_utils.class_agnostic_nms_lidar(
            box_scores=cls_preds, box_preds=box_preds, box_labels=label_preds,
            nms_config=post_process_cfg.NMS_CONFIG,
            score_thresh=self.score_dict
        )
        final_labels = final_labels.reshape(final_labels.shape[0], 1)
        final_scores = final_scores.reshape(final_scores.shape[0], 1)
        final_sensor = np.zeros((final_scores.shape[0], 1), dtype=np.int32)

        record_dict = {
            'pred_boxes': final_boxes,
            'pred_scores': final_scores,
            'pred_labels': final_labels,
            'pred_sensor': final_sensor,
        }

        return record_dict