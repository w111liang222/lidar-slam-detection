import os
import sys
sys.path.append(os.getcwd())

import copy
import pickle
import numpy as np
from polygraphy.json import save_json
from sensor_inference.utils.config import cfg, cfg_from_yaml_file
from sensor_inference.dataset.demo_dataset import RealtimeDataset

def parse_config(cfg_file):
    cfg_from_yaml_file(cfg_file, cfg)
    return cfg

# Option 1: Define a function that will yield feed_dicts (i.e. Dict[str, np.ndarray])
def load_data():
    config = copy.deepcopy(parse_config("sensor_inference/cfgs/detection_lidar_large.yaml"))
    dataset = RealtimeDataset(dataset_cfg=config.DATA_CONFIG,
                              class_names=config.CLASS_NAMES,
                              training=False)

    for i in range(5):
        data_file = "sensor_inference/data/{0:06d}.pkl".format(i)
        f = open(data_file, 'rb', buffering=10*1024*1024)
        data_dict = pickle.loads(f.read())
        points = data_dict.pop('points', None)

        lidar_data = dataset.prepare_data(np.concatenate(list(points.values()), axis=0))
        lidar_data = list(lidar_data.values())
        lidar_data[1] = lidar_data[1].astype(np.float32)
        yield {"voxel_features": lidar_data[0], "coords": lidar_data[1], "mask": lidar_data[2]}

# Option 2: Create a JSON file containing the input data using the `save_json()` helper.
#   The input to `save_json()` should have type: List[Dict[str, np.ndarray]].
#   For convenience, we'll reuse our `load_data()` implementation to generate the list.
input_data = list(load_data())
save_json(input_data, "inputs.json", description="custom input data")