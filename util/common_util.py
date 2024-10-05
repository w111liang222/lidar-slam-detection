import os
import ctypes
import ctypes.util
import time
import queue

from pathlib import Path
from threading import Thread

def run_cmd(cmd):
    return os.system('%s' % (cmd))

def opener(path, flags):
    return os.open(path, flags, 0o777)

def do_reboot():
    return run_cmd('reboot')

def do_mkdir(path):
    try:
        Path(path).mkdir(parents=True, exist_ok=True)
        success = True
    except Exception as e:
        print(e)
        success = False
    return success

DEFAULT_DISK_PATH = '/root/exchange'
def has_extension_disk():
    has_disk = False
    disk_name = ''
    with open('/proc/mounts') as f:
        for v in f:
            v = v.split()
            mount_name = v[1]
            if '/media' in mount_name and mount_name != '/media':
                has_disk = True
                disk_name = mount_name
                break

    # check the default path is exist
    if not has_disk and os.path.exists(DEFAULT_DISK_PATH):
        has_disk = True
        disk_name = DEFAULT_DISK_PATH

    return has_disk, disk_name

def get_disk_info(disk_name):
    try:
        vfs = os.statvfs(disk_name)
        total = vfs.f_bsize * vfs.f_blocks / 1024
        used = vfs.f_bsize * (vfs.f_blocks - vfs.f_bfree) / 1024
        avail = vfs.f_bsize * vfs.f_bavail / 1024
        used_percent = round(float(used) / float(used + avail) * 100, 2)
    except Exception as e:
        print(e)
        total, used, avail, used_percent = 0, 0, 0, 0

    return total, used, avail, used_percent

def get_disk_status():
    has_disk, disk_name = has_extension_disk()
    if has_disk:
        total, used, avail, used_percent = get_disk_info(disk_name)
    else:
        total, used, avail, used_percent = 0, 0, 0, 0

    return {'has_disk' : has_disk, 'disk_name' : Path(disk_name).name,
            'total': total, 'used_percent': used_percent}

def get_dir_files(directory):
    file_idx, file_dict = 0, dict()
    try:
        has_disk, disk_name = has_extension_disk()
        if not has_disk:
            return {}
        record_path = disk_name + '/' + directory
        file_list = os.listdir(record_path)
        file_list.sort()
        for file in file_list:
            try:
                file_path = os.path.join(record_path, file)
                if os.path.isdir(file_path):
                    file_dict[str(file_idx)] = [file_path, len(os.listdir(file_path))]
                    file_idx = file_idx + 1
            except Exception as e:
                print(e)
    except Exception as e:
        print(e)
    return file_dict

def set_system_time(peer_time):
    if peer_time is None:
        return

    peer_time = peer_time / 1000.0
    host_time = time.time()
    # only process when time diff > 60s
    if abs(peer_time - host_time) < 60:
        return

    # do not allow to backward
    if (host_time - peer_time) >= 60:
        print("system time is not allow to backward %f s" % (host_time - peer_time))
        return

    print("adjust device time to %f" % (peer_time))
    try:
        CLOCK_REALTIME = 0
        class timespec(ctypes.Structure):
            _fields_ = [("tv_sec", ctypes.c_long),
                        ("tv_nsec", ctypes.c_long)]

        librt = ctypes.CDLL(ctypes.util.find_library("rt"))

        ts = timespec()
        ts.tv_sec = int(peer_time)
        ts.tv_nsec = int((peer_time - ts.tv_sec) * 1000000000) # second to nanosecond

        # http://linux.die.net/man/3/clock_settime
        librt.clock_settime(CLOCK_REALTIME, ctypes.byref(ts))
    except Exception as e:
        print(e)

SIOCGIFADDR = 0x8915
SIOCGIFNETMASK = 0x891b
def get_network(iface, gw = True):
    import socket, struct, fcntl
    gateway = "0.0.0.0"
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            iface_bin = struct.pack('256s', bytes(iface, 'utf-8'))
            ip_addr = socket.inet_ntoa(fcntl.ioctl(s.fileno(), SIOCGIFADDR, iface_bin)[20:24])
            netmask = socket.inet_ntoa(fcntl.ioctl(s.fileno(), SIOCGIFNETMASK, iface_bin)[20:24])
        if gw is True:
            gateway = get_default_gateway(iface)
    except Exception as e:
        return None
    return ip_addr, netmask, gateway

def get_default_gateway(iface):
    gateway = "0.0.0.0"
    routes = os.popen("ip route | awk '/default/ { print $3 $5 }'").read().rstrip().split()
    for route in routes:
        if iface in route:
            gateway = route.replace(iface, "")
            break
    return gateway

class ThreadWorker():
    def __init__(self, name, task, num = 6):
        self.name = name
        self.task = task
        self.worker_num = num
        self.is_start = False

    def __del__(self):
        self.stop()

    def start(self):
        if not self.is_start:
            self.is_start = True
            self.start_worker()

    def stop(self):
        if self.is_start:
            self.is_start = False
            self.stop_worker()

    def worker_loop(self, worker_idx, input_queue, output_queue):
        print("%s, worker %d loop starts" % (self.name, worker_idx))
        while self.is_start:
            data = input_queue.get()
            if data is None:
                continue
            data = self.task(worker_idx, data)
            output_queue.put(data)
        print("%s, worker %d loop stop" % (self.name, worker_idx))

    def start_worker(self):
        self.worker_threads, self.input_queues, self.output_queues = [], [], []
        for i in range(self.worker_num):
            input_queue = queue.Queue(maxsize=3)
            output_queue = queue.Queue(maxsize=3)
            thread = Thread(target=self.worker_loop, args=(i, input_queue, output_queue,), name=self.name + "-" + str(i), daemon=True)
            thread.start()
            self.worker_threads.append(thread)
            self.input_queues.append(input_queue)
            self.output_queues.append(output_queue)

    def stop_worker(self):
        for input_queue in self.input_queues:
            input_queue.put(None)
        for thread in self.worker_threads:
            thread.join()

    def apply_task_sync(self, data):
        worker_idx = 0
        output_queue_dict = dict()
        # assign task
        for k, v in data.items():
            self.input_queues[worker_idx].put(v)
            output_queue_dict[k] = self.output_queues[worker_idx]
            worker_idx = worker_idx + 1
        # get result
        for k, output_queue in output_queue_dict.items():
            data[k] = output_queue.get()

        return data

is_encoder_init, encoders, encode_worker = False, None, None
def encode_image(worker_idx, img):
    global encoders
    if img.shape[-1] != 3:
        return encoders[worker_idx].encode_YUV(img)
    else:
        return encoders[worker_idx].encode(img)

def init_image_encoder():
    from third_party.turbojpeg import TurboJPEG
    global is_encoder_init, encoders, encode_worker
    encoders = [TurboJPEG() for i in range(16)]
    encode_worker = ThreadWorker('jpeg encoder', encode_image, 16)
    encode_worker.start()
    is_encoder_init = True

def encoder_image_jpeg(images):
    global is_encoder_init, encode_worker
    if not is_encoder_init:
        init_image_encoder()
    images = encode_worker.apply_task_sync(images)
    return images

is_decoder_init, decoders, decoder_worker = False, None, None
def decode_image(worker_idx, img):
    global decoders
    return decoders[worker_idx].decode(img)

def init_image_decoder():
    from third_party.turbojpeg import TurboJPEG
    global is_decoder_init, decoders, decoder_worker
    decoders = [TurboJPEG() for i in range(16)]
    decoder_worker = ThreadWorker('jpeg decoder', decode_image, 16)
    decoder_worker.start()
    is_decoder_init = True

def decode_image_jpeg(images):
    global is_decoder_init, decoder_worker
    if not is_decoder_init:
        init_image_decoder()
    images = decoder_worker.apply_task_sync(images)
    return images
