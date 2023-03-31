import socket
import pickle
from threading import Thread

class RemoteControl:
    def __init__(self, logger):
        self.status = dict(
            connection = False,
            steer_angle = 0,
            throttle = 0,
            brake = 0,
        )
        self.logger = logger

    def get_status(self):
        return self.status['connection']

    def set_certification(self, certification):
        self.certification = certification

    def start_daemon(self):
        self.daemon_start = True
        self.daemon_thread = Thread(target=self.run, daemon=True)
        self.daemon_thread.start()

    def stop_daemon(self):
        self.daemon_start = False
        self.daemon_thread.join()

    def run(self):
        server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        ip_port = ('127.0.0.1', 12660)
        server.settimeout(1)
        server.bind(ip_port)
        while self.daemon_start:
            try:
                data, peer = server.recvfrom(65536)
                self.status['connection'] = True
                data_dict = pickle.loads(data)
                self.send_commands(data_dict)
            except socket.timeout:
                if self.status['connection']:
                    self.status['connection'] = False
                    self.logger.warn('Disconnected with I-Cloud')

    def send_commands(self, data):
        if self.certification == data['certification']:
            self.logger.info(data)
        else:
            self.logger.warn('Certification failed, should be {}, but get {}'.format(self.certification, data['certification']))
        return
