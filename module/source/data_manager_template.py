from threading import Thread, Lock, Event
from util.period_calculator import PeriodCalculator
import sensor_driver.common_lib.cpp_utils as util

class DummyLogger():
    def __init__(self):
        pass
    def debug(self, *args, **kwargs):
        pass
    def info(self, *args, **kwargs):
        pass
    def warn(self, *args, **kwargs):
        pass
    def error(self, *args, **kwargs):
        pass

class DataManagerTemplate():
    def __init__(self, name, cfg, logger=None):
        self.name = name
        self.cfg = cfg
        self.mode = cfg.input.mode
        self.logger = logger if logger is not None else DummyLogger()
        if self.mode == "offline":
            self.offline_init()
        else:
            self.online_init()

    def online_init(self):
        pass

    def offline_init(self):
        pass

    def setup(self, cfg):
        self.cfg = cfg

    def start_loop(self, sensors):
        self.start = True
        self.acquire_thread, self.acquire_event, self.sensor_online, self.data_dict = dict(), dict(), dict(), dict()
        for sensor_name, sensor in sensors.items():
            sensor_event = Event()
            self.acquire_thread[sensor_name] = Thread(target=self.acquire_loop, args=(sensor, sensor_name, sensor_event,), daemon=True)
            self.acquire_event[sensor_name] = sensor_event
            self.sensor_online[sensor_name] = True
            self.acquire_thread[sensor_name].start()

    def stop_loop(self):
        self.start = False
        for acquire_thread in self.acquire_thread.values():
            acquire_thread.join()

    def acquire_loop(self, sensor, sensor_name, sensor_event):
        self.logger.info(f'{self.name}: {sensor_name}: acquire_loop start')
        util.set_thread_priority("Py-" + self.name[0] + sensor_name, 30)
        acquire_period = PeriodCalculator()
        while self.start:
            acquire_dict, valid = self.loop_run_once(sensor, sensor_name)
            acquire_fps = acquire_period.hit()
            if acquire_fps > 8.0:
                self.logger.debug(f'{self.name}: {sensor_name}, acquire FPS: {acquire_fps:.1f}')
            else:
                self.logger.warn (f'{self.name}: {sensor_name}, acquire FPS: {acquire_fps:.1f}')
            if not valid:
                continue
            self.data_lock.acquire()
            self.data_dict.update(acquire_dict)
            self.data_lock.release()
            sensor_event.set()
        sensor_event.set()
        self.logger.info(f'{self.name}: {sensor_name}: acquire_loop exit')

    def get_loop_data(self, timeout = 1.0):
        main_sensor_active, skip_sensor_wait = False, False
        for sensor_name in self.acquire_thread:
            last_sensor_state = self.sensor_online[sensor_name]
            if self.sensor_online[sensor_name] and (not skip_sensor_wait):
                self.sensor_online[sensor_name] = self.acquire_event[sensor_name].wait(timeout)
            else:
                self.sensor_online[sensor_name] = self.acquire_event[sensor_name].is_set()

            # check sensor is online
            if self.sensor_online[sensor_name] is True:
                # if main sensor's state is changed from inactive to active, to make main sensor as baseline, skip to wait for child sensors
                if main_sensor_active is False and last_sensor_state is False:
                    skip_sensor_wait = True
                # set main sensor state to True
                main_sensor_active = True

        for acuqire_event in self.acquire_event.values():
            acuqire_event.clear()

        self.data_lock.acquire()
        data_dict = self.data_dict.copy()
        self.data_dict = dict()
        self.data_lock.release()
        return data_dict

    def loop_run_once(self, sensor, sensor_name):
        raise NotImplementedError

    def start_capture(self):
        self.data_lock = Lock()

    def stop_capture(self):
        raise NotImplementedError

    def get_data(self, data_dict):
        if self.mode == "online":
            data = self.get_data_online(data_dict)
        else:
            data = self.get_data_offline(data_dict)
        return data

    def post_process_data(self, data_dict):
        return data_dict

    def get_data_online(self, data_dict):
        raise NotImplementedError

    def get_data_offline(self, data_dict):
        raise NotImplementedError

    def release(self):
        raise NotImplementedError