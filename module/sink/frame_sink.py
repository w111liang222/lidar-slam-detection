import datetime
import os
import time
import pickle
from threading import Thread
import queue

from .sink_template import SinkTemplate

from util.common_util import run_cmd, do_mkdir, has_extension_disk, get_disk_info, opener
from ..export_interface import register_interface, call

class FrameSink(SinkTemplate):
    def __init__(self, cfg, logger):
        super().__init__('FrameSink', cfg, logger, queue_max_size=100)
        self.is_start = False
        self.is_free_disk = False
        self.run_mode = cfg.input.mode
        self.mode = cfg.system.record.mode
        self.loop_duration = cfg.system.record.loop_duration
        self.exclude_keys = cfg.system.record.exclude_keys
        self.journal = cfg.system.record.journal

        self.init_runtime_variable()
        if cfg.system.record.auto_start:
            self.start()

        register_interface('frame.start_record', self.start)
        register_interface('frame.stop_record', self.stop)
        register_interface('frame.get_status', self.get_status)

    def init_runtime_variable(self):
        self.file_index = 0
        self.file_path = None
        self.success_num, self.lost_num, self.error_num = 0, 0, 0

    def get_log_files(self, disk_name):
        log_file_list = []
        try:
            record_path = disk_name + '/' + 'lp_log'
            file_list = os.listdir(record_path)
            for file in file_list:
                file_path = os.path.join(record_path, file)
                if os.path.isdir(file_path):
                    log_file_list.append(file_path)
        except Exception as e:
            self.logger.error(e)
        log_file_list.sort()
        return log_file_list

    def free_disk_space(self, disk_name):
        _, _, avail, _ = get_disk_info(disk_name)
        dir_files = self.get_log_files(disk_name)
        min_require_size = self.loop_duration * 10 * (3 * 1024)
        while avail < min_require_size and self.mode == 'loop' and len(dir_files) >= 2:
            delete_dir = dir_files[0]
            dir_files = dir_files[1:]
            run_cmd('rm -rf ' + delete_dir + ' &')
            time.sleep(0.1)
            _, _, avail, _ = get_disk_info(disk_name)
            self.logger.info('avail %d, min_req %d, delete the oldest log %s' % (avail, min_require_size, delete_dir))

    def start(self, directory):
        if self.is_start:
            return
        self.has_disk, self.disk_name = has_extension_disk()
        if not self.has_disk:
            return

        self.free_disk_space(self.disk_name)
        if not self.create_new_log(self.disk_name, directory):
            return

        self.start_wall_time = time.monotonic()
        self.input_queue = queue.Queue(maxsize = self.queue_max_size)
        self.is_start = True
        self.run_process = Thread(target=self._run, daemon=True)
        self.run_process.start()
        self.logger.info("Start Recording")

    def stop(self):
        if not self.is_start:
            return
        self.is_start = False
        try:
            self.input_queue.put_nowait(dict())
        except:
            pass
        self.run_process.join()
        if self.journal:
            run_cmd('mkdir -p {}/.journal && \
                     dmesg > {}/.journal/kernel.log && \
                     journalctl -u perception.service --no-pager --no-hostname > {}/.journal/service.log && \
                     chmod a+w+r {}/.journal &'.format(self.file_path, self.file_path, self.file_path, self.file_path))
        self.init_runtime_variable()
        run_cmd('sync &')
        self.logger.info("Stop Recording")

    def _run(self):
        self.logger.info('deamon %s starts' % self.name)
        while self.is_start:
            data_dict = self.input_queue.get()
            if not data_dict:
                continue
            self.sink(data_dict)
        self.logger.info('deamon %s stops' % self.name)

    def get_status(self):
        return dict(
            status       = self.is_start,
            frame_success= self.success_num,
            frame_lost   = self.lost_num,
            frame_error  = self.error_num,
        )

    def create_new_log(self, disk_name, directory = None):
        self.file_path = disk_name + '/' + ('lp_log/%s' % (datetime.datetime.now().strftime('%Y-%m-%d-%H-%M-%S') if directory is None else directory))
        success = do_mkdir(self.file_path)
        if success:
            self.logger.info('mkdir success: %s' % self.file_path)
        else:
            self.init_runtime_variable()
        call('config.dump_config', path=self.file_path + '/' + 'cfg.yaml', sync=False)
        os.chmod(self.file_path, 0o777)

        return success

    def disk_plug_out_dispatch(self):
        self.logger.warn('disk write error, maybe extension disk plug out')
        self.stop()
        self.init_runtime_variable()

    def prepare_data(self, data_dict):
        for item in self.exclude_keys:
            item_name = item.split('.', maxsplit=1)
            if len(item_name) == 1:
                data_dict.pop(item_name[0], None)
            elif len(item_name) == 2:
                if item_name[0] not in data_dict:
                    continue
                item_dict = data_dict[item_name[0]].copy()
                item_dict.pop(item_name[1], None)
                data_dict[item_name[0]] = item_dict

        return data_dict

    def enqueue(self, data_dict):
        if not self.is_start or self.is_free_disk:
            return
        if self.error_num >= 10:
            self.disk_plug_out_dispatch()

        if self.file_path is None:
            return

        try:
            data_dict['file_path'] = self.file_path + '/{0:06d}.pkl'.format(self.file_index)
            data_dict = self.prepare_data(data_dict)
            if self.run_mode == "online":
                self.input_queue.put_nowait(data_dict)
            else:
                self.input_queue.put(data_dict)
            self.file_index = self.file_index + 1
        except queue.Full:
            self.logger.warn('input queue is full, %s is too slow' % self.name)
            self.lost_num += 1

    def sink(self, data_dict):
        log_duration = time.monotonic() - self.start_wall_time
        if self.mode == 'loop' and log_duration > self.loop_duration:
            self.logger.info('frame sink rotate, duration %.1fs > %.1fs' % (log_duration, self.loop_duration))
            self.is_free_disk = True
            self.free_disk_space(self.disk_name)
            self.init_runtime_variable()
            self.create_new_log(self.disk_name)
            self.is_free_disk = False
            self.start_wall_time = time.monotonic()
            return

        data_dict['image'] = data_dict.pop('image_jpeg', dict())
        for name, img in data_dict['image'].items():
            data_dict['image'][name] = img if isinstance(img, bytes) else img.tobytes()

        log_file = data_dict.pop('file_path')
        try:
            os.umask(0)
            with open(log_file, mode='wb', buffering=10*1024*1024, opener=opener) as f:
                pickle.dump(data_dict, f, protocol=4)
            self.success_num += 1
        except Exception as e:
            self.logger.warn('FrameSink dump error: %s' % (e))
            self.error_num += 1
