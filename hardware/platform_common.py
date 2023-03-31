
import os
import platform

def get_machine():
    machine = platform.machine()
    return machine

def get_board():
    try:
        with open('/proc/device-tree/nvidia,dtsfilename',"r") as f:
            dts = f.read()
        if 'p3668' in dts:
            return "Xavier-NX"
        elif 'p2822' in dts:
            return "Xavier-AGX"
        elif 'p3701' in dts:
            return "AGX-Orin-32GB"
    except Exception as e:
        machine = get_machine()
        if machine == "x86_64":
            return "IPC"
        else:
            return "Unknow"

def get_jetpack():
    process = os.popen('''dpkg-query --showformat='${Version}' --show nvidia-l4t-core ''')
    l4t = process.read().split('-')[0]
    if l4t == "32.7.1":
        return "4.6.1"
    elif l4t == "32.4.4" or l4t == "32.5.1":
        return "4.4.1"
    elif l4t == "35.1.0":
        return "5.0.2"

    if get_board() == "IPC":
        return "20.04"

    return "UnKnown"

def is_jetson():
    board_name = get_board()
    if board_name in ["Xavier-NX", "Xavier-AGX", "AGX-Orin-32GB"]:
        return True
    else:
        return False

BOARD_NAME = get_board()
MACHINE = get_machine()
JETPACK = get_jetpack()
