import ins_driver_ext as driver

class InsDriver():
    def __init__(self, port, device, ins_type, mode, extrinsic_param, logger = None):
        self.port = port
        self.device = device
        self.ins_type = ins_type
        self.mode = mode
        self.extrinsic_param = extrinsic_param
        self.logger = logger

    def open(self):
        driver.create_ins(self.ins_type, self.mode)
        driver.set_extrinsic_param(self.extrinsic_param[0], self.extrinsic_param[1], self.extrinsic_param[2],
                                   self.extrinsic_param[5], self.extrinsic_param[4], self.extrinsic_param[3])

    def close(self):
        driver.destory_ins()

    def start(self):
        if self.mode == "online":
            driver.start_capture(int(self.port), self.device)

    def stop(self):
        if self.mode == "online":
            driver.stop_capture()

    def start_relay(self, dest_ip):
        driver.start_transfer(dest_ip)

    def stop_relay(self):
        driver.stop_transfer()

    def trigger(self, timestamp):
        return driver.trigger(timestamp)

    def set_offline_data(self, ins_data, imu_data):
        driver.set_offline_data(ins_data, imu_data)
