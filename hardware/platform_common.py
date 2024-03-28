
import os
import platform

def get_machine():
    machine = platform.machine()
    return machine

def get_version():
    if not os.path.exists('VERSION'):
        return 'Dev'
    else:
        f = open('VERSION', 'r')
        return f.read().replace("\n","")

def get_board():
    try:
        with open('/proc/device-tree/nvidia,dtsfilename',"r") as f:
            dts = f.read()
        if 'p3701' in dts:
            return "AGX-Orin-32GB"
        elif 'p3767' in dts:
            return "NX-Orin"
        raise Exception("Invalid nvidia board")
    except Exception as e:
        machine = get_machine()
        return machine

def get_jetpack():
    process = os.popen('''dpkg-query --showformat='${Version}' --show nvidia-l4t-core ''')
    l4t = process.read().split('-')[0]
    if l4t == "35.1.0":
        return "5.0.2"
    elif l4t == "35.4.1":
        return "5.1.2"

    return "20.04"

def is_jetson():
    board_name = get_board()
    if board_name in ["AGX-Orin-32GB", "NX-Orin"]:
        return True
    else:
        return False

def is_rockchip():
    process = os.popen('''uname -a''')
    uname = process.read().replace("\n","")
    if 'rk3588' in uname:
        return True, 'RK3588'

    return False, ''

BOARD_NAME = get_board()
MACHINE = get_machine()
JETPACK = get_jetpack()
VERSION = get_version()
