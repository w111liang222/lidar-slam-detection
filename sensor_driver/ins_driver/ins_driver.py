import ins_driver_ext as driver

class InsDriver():
    def __init__(self, port, device,
                 ex_param = [0, 0, 0, 0, 0, 0],
                 logger = None):
        self.port = port
        self.device = device
        self.ex_param = ex_param
        self.logger = logger

    def open(self):
        driver.create_ins()
        driver.set_external_param(self.ex_param[0], self.ex_param[1], self.ex_param[2],
                                  self.ex_param[5], self.ex_param[4], self.ex_param[3])

    def close(self):
        driver.destory_ins()

    def start(self):
        driver.start_capture(int(self.port), self.device)

    def stop(self):
        driver.stop_capture()

    def start_relay(self, dest_ip):
        driver.start_transfer(dest_ip)

    def stop_relay(self):
        driver.stop_transfer()

    def trigger(self, timestamp):
        data_dict = driver.trigger(timestamp)
        return data_dict

    def get_valid_message_count(self):
        return driver.get_valid_message_count()

    def get_receive_message_count(self):
        return driver.get_receive_message_count()

    def set_offline_mode(self):
        driver.set_offline_mode()

    def set_offline_data(self, data):
        driver.set_offline_data(data)

    def set_external_param(self, ex_param):
        self.ex_param = ex_param
        driver.set_external_param(self.ex_param[0], self.ex_param[1], self.ex_param[2],
                                  self.ex_param[5], self.ex_param[4], self.ex_param[3])
