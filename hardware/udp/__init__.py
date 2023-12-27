from .linux.udp_driver import LinuxUdpDriver

__all__ = {
    'Linux': LinuxUdpDriver,
}

def build_udp(hardware, dest_ip, dest_port, logger):
    udp_device = __all__[hardware](logger=logger, dest_ip=dest_ip, dest_port=dest_port)
    return udp_device
