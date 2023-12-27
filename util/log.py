import logging
import colorlog
import yaml
import threading
from easydict import EasyDict
from pathlib import Path

def thread_id_filter(log):
    log.thread_id = threading.get_native_id()
    return log

def create_logger(log_level='INFO'):
    log_colors_config = {
        'DEBUG': 'white',
        'INFO': 'green',
        'WARNING': 'yellow',
        'ERROR': 'red',
        'CRITICAL': 'bold_red',
    }
    logger = logging.getLogger(__name__)
    logger.setLevel(log_level)
    formatter = logging.Formatter('%(asctime)s[%(process)d:%(thread_id)s][%(filename)15s:%(lineno)3d] %(message)s')
    console_formatter = colorlog.ColoredFormatter(fmt='%(log_color)s%(asctime)s[%(process)d:%(thread_id)s][%(filename)15s:%(lineno)3d] %(message)s', log_colors=log_colors_config)
    console = logging.StreamHandler()
    console.setLevel('DEBUG')
    console.setFormatter(console_formatter)
    console.addFilter(thread_id_filter)
    logger.addHandler(console)
    return logger

def get_logger(log_level='INFO'):
    log_dir = Path('output/logs')
    log_dir.mkdir(parents=True, exist_ok=True)
    logger = create_logger(log_level)
    return logger

def get_logger_from_config(config_path):
    config = EasyDict(yaml.safe_load(open(config_path)))
    logger = get_logger(config.system.log_level)
    return logger, config.system.log_level