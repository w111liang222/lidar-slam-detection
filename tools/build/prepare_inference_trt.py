import os
import sys

sys.path.append(os.getcwd())

if 'BOARD_NAME' in os.environ and 'JETPACK' in os.environ:
    BOARD_NAME = os.environ['BOARD_NAME']
    JETPACK = os.environ['JETPACK']
    IS_JETSON = True
else:
    from hardware.platform_common import BOARD_NAME, MACHINE, JETPACK, is_jetson
    IS_JETSON = is_jetson()

JETSON_CMD_TEMPLATE = '''
git clone git@10.10.80.28:product/sensor_inference_trt.git
cd sensor_inference_trt
git checkout origin/{}
cd -
cp sensor_inference_trt/{}/* sensor_inference/
rm -rf sensor_inference_trt
'''

IPC_CMD_TEMPLATE = '''
git clone git@10.10.80.28:product/sensor_inference_trt.git
cd sensor_inference_trt
git checkout origin/{}
cd -
cp sensor_inference_trt/onnx/* sensor_inference/
rm -rf sensor_inference_trt
'''

def run_cmd(cmd):
    return os.system('%s' % (cmd))

def main():
    if IS_JETSON:
        print("Prepare TensorRT for {}, Jetpack {}".format(BOARD_NAME, JETPACK))
        cmd = JETSON_CMD_TEMPLATE.format(BOARD_NAME, JETPACK)
    elif BOARD_NAME == "IPC":
        print("Prepare ONNX for {}, Ubuntu {}".format(BOARD_NAME, JETPACK))
        cmd = IPC_CMD_TEMPLATE.format(BOARD_NAME)
    else:
        print("Invalid board {}".format(BOARD_NAME))
        return

    run_cmd(cmd)
    run_cmd("echo {} > /tmp/BOARD_NAME".format(BOARD_NAME))
    run_cmd("echo {} > /tmp/JETPACK".format(JETPACK))

if __name__ == '__main__':
    main()