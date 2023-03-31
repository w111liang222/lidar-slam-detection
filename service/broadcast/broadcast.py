import time
from threading import Thread
import socket

class BroadCast:
    def __init__(self):
        self.server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
        self.server.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.server.settimeout(0.2)

    def start_daemon(self):
        self.daemon_start = True
        self.daemon_thread = Thread(target=self.run, daemon=True)
        self.daemon_thread.start()

    def stop_daemon(self):
        self.daemon_start = False
        self.daemon_thread.join()

    def run(self):
        while self.daemon_start:
            try:
                self.server.sendto(b'IPU', ('<broadcast>', 37020))
            except Exception as e:
                print(e)
            time.sleep(1)


if __name__ == '__main__':
    broadcast_service = BroadCast()
    broadcast_service.start_daemon()

    while True:
        time.sleep(100)