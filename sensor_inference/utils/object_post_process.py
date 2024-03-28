import numpy as np
from sensor_inference.utils import model_nms_utils

class PostProcesser():
    def __init__(self, model_cfg, num_class, class_names):
        self.model_cfg = model_cfg
        self.num_class = num_class
        self.class_names = class_names
        self.score_dict = dict()
        for name, thresh in model_cfg.POST_PROCESSING.SCORE_THRESH.items():
            self.score_dict[self.class_names.index(name)+1] = thresh

        self.map_score = model_cfg.POST_PROCESSING.MAP_SCORE_THRESH

    def forward(self, cls_preds, box_preds, label_preds, freespace):
        post_process_cfg = self.model_cfg.POST_PROCESSING

        final_scores, final_boxes, final_labels = model_nms_utils.class_agnostic_nms(
            box_scores=cls_preds, box_preds=box_preds, box_labels=label_preds,
            nms_config=post_process_cfg.NMS_CONFIG,
            score_thresh=self.score_dict
        )
        final_labels = final_labels.reshape(final_labels.shape[0], 1)
        final_scores = final_scores.reshape(final_scores.shape[0], 1)

        result_dict = {
            'pred_boxes' : final_boxes,
            'pred_scores': final_scores,
            'pred_labels': final_labels,
        }

        return result_dict