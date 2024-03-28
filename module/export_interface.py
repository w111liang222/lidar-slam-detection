interface_dict = dict()

def register_interface(name, func):
    interface_dict[name] = func

def unregister_interface(name):
    interface_dict.pop(name, None)

def call(name, **kwargs):
    if name in interface_dict:
        return interface_dict[name](**kwargs)
    else:
        return None
