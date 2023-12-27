# sudo PYTHONPATH=$PYTHONPATH:/home/znqc/.local/lib/python3.8/site-packages python tools/scripts/dump_system.py
import zerorpc

perception = zerorpc.Client(heartbeat=None, timeout=300)
perception.connect("tcp://127.0.0.1:37040")

# set logging level
perception.set_runtime_config({"log_level": "INFO"})

args = dict(
    cutoff = 10,
    detail = 10,
)
data = perception.dump(args)

# dump perception memory
# print(data['mem'])

# dump python thread stack
# for stack in data['stack']:
#     print(stack, end='')
