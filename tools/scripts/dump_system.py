# sudo PYTHONPATH=$PYTHONPATH:/home/znqc/.local/lib/python3.8/site-packages python tools/scripts/dump_system.py
import zerorpc

perception = zerorpc.Client(heartbeat=None, timeout=300)
perception.connect("tcp://127.0.0.1:37040")

# set logging level
perception.set_runtime_config({"log_level": "INFO"})

# dump system snap
data = perception.dump()

# dump python thread stack
print('\033[92m' + "================= start dump thread =================")
for stack in data['stack']:
    print('\033[93m' + stack, end='')
print('\033[92m' + "================== dump thread ok ==================")