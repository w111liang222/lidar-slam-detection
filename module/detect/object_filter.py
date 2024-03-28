from shapely.geometry import Point, Polygon
import numpy as np

class ObjectFilter():
    def __init__(self, logger = None):
        self.logger = logger
        self.cls_dict = dict()
        for idx, cls_name in enumerate(['Vehicle', 'Pedestrian', 'Cyclist', 'Traffic_Cone']):
            self.cls_dict[cls_name.lower()] = idx + 1

        self.filter_roi = False
        self.include_poly = []
        self.exclude_poly = []

    def set_config(self, cfg):
        self.set_output(cfg.output)
        if len(cfg.roi[0]['contour']) == 0:
            self.disable_roi()
        else:
            self.enable_roi(cfg.roi[0]['contour'], is_include=cfg.roi[0]['include'])

    def set_output(self, cfg):
        self.cfg = cfg
        self.output_cls = dict()
        for key, val in cfg.object.items():
            if key == 'use' or key=='freespace':
                continue
            self.output_cls[key] = val
        self.logger.info("class mapping: {}".format(self.cls_dict))

    def enable_roi(self, poly_pts, is_include = True):
        self.include_poly = []
        self.exclude_poly = []
        poly = Polygon(poly_pts)
        if is_include:
            self.include_poly.append(poly)
        else:
            self.exclude_poly.append(poly)
        self.filter_roi = True

    def disable_roi(self):
        self.filter_roi = False
        self.include_poly = []
        self.exclude_poly = []

    def filter(self, result_dict):
        if not self.cfg.object.freespace:
            result_dict.pop('freespace', None)

        if not result_dict['lidar_valid'] and not result_dict['image_valid']:
            return result_dict

        if 'pred_boxes' not in result_dict and 'pred_attr' not in result_dict and 'pred_traj' not in result_dict:
            return result_dict

        for key, val in self.output_cls.items():
            if val:
                continue
            filter_mask = ~(result_dict['pred_attr'][:, 6] == self.cls_dict[key])
            for result_key, result_val in result_dict.items():
                if result_key not in ['pred_boxes', 'pred_attr', 'pred_traj']:
                    continue
                result_dict[result_key] = result_dict[result_key][filter_mask]

        if not self.filter_roi:
            return result_dict

        # include ROI
        for poly in self.include_poly:
            valid_mask = np.ones(result_dict['pred_attr'].shape[0], dtype=np.bool)
            for idx in range(0, valid_mask.shape[0]):
                pt = Point(result_dict['pred_boxes'][idx, 0], result_dict['pred_boxes'][idx, 1])
                valid_mask[idx] = pt.within(poly)
            for result_key, result_val in result_dict.items():
                if result_key not in ['pred_boxes', 'pred_attr', 'pred_traj']:
                    continue
                result_dict[result_key] = result_dict[result_key][valid_mask]
        # exclude ROI
        for poly in self.exclude_poly:
            valid_mask = np.ones(result_dict['pred_attr'].shape[0], dtype=np.bool)
            for idx in range(0, valid_mask.shape[0]):
                pt = Point(result_dict['pred_boxes'][idx, 0], result_dict['pred_boxes'][idx, 1])
                valid_mask[idx] = not pt.within(poly)
            for result_key, result_val in result_dict.items():
                if result_key not in ['pred_boxes', 'pred_attr', 'pred_traj']:
                    continue
                result_dict[result_key] = result_dict[result_key][valid_mask]
        return result_dict
