import re
import os
import numpy as np
import xml.etree.ElementTree as ElementTree
from easydict import EasyDict

def parse_xodr(file_path):
    trafficlight_list = []
    trafficlight_pos = np.zeros((0, 4))
    if not os.path.exists(file_path):
        return np.array(trafficlight_list), trafficlight_pos

    tree = ElementTree.parse(file_path)
    root = tree.getroot()
    for road in root.findall('road'):
        for signals in road.findall('signals'):
            for signal in signals.findall('signal'):
                if re.match('Signal_.Light', signal.attrib['name']):
                    trafficlight = EasyDict()
                    trafficlight.id = signal.attrib['id']
                    trafficlight.position = signal.find('positionInertial').attrib
                    for key, value in trafficlight.position.items():
                        trafficlight.position[key] = float(value)
                    trafficlight_pos = np.vstack((trafficlight_pos,
                                                  np.array([trafficlight.position.x, trafficlight.position.y, trafficlight.position.z, 1])))
                    for userData in signal.findall('userData'):
                        if userData.attrib['code'] == 'name':
                            trafficlight.name = userData.attrib['value']
                        elif userData.attrib['code'] == 'width':
                            trafficlight.width = float(userData.attrib['value'])
                        elif userData.attrib['code'] == 'height':
                            trafficlight.height = float(userData.attrib['value'])
                        elif userData.attrib['code'] == 'orientation':
                            trafficlight.orientation = userData.attrib['value']
                        elif userData.attrib['code'] == 'direction':
                            trafficlight.direction = userData.attrib['value']
                    trafficlight_list.append(trafficlight)
    trafficlight_list = np.array(trafficlight_list)
    return trafficlight_list, trafficlight_pos

def parse_anchor(file_path):
    if not os.path.exists(file_path):
        return None

    with open(file_path, 'r') as f:
        line = f.readline().split()
        origin = EasyDict()
        origin.lat   = float(line[0])
        origin.lon   = float(line[1])
        origin.alt   = float(line[2])
        origin.yaw   = float(line[3])
        origin.pitch = float(line[4])
        origin.roll  = float(line[5])

    return origin
