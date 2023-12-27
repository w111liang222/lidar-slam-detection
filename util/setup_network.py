TEMPLATE='''\
# interfaces(5) file used by ifup(8) and ifdown(8)

'''

STATIC_TEMPLATE='''\
nmcli con mod eth{} ipv4.method manual ipv4.addresses {}/{} ipv4.gateway {}
'''

DHCP_TEMPLATE='''\
nmcli con mod eth{} ipv4.method auto ipv4.gateway "" ipv4.addresses ""
'''

import socket
from ipaddress import IPv4Network

def is_valid_ipv4_address(address):
    try:
        socket.inet_pton(socket.AF_INET, address)
    except AttributeError:  # no inet_pton here, sorry
        try:
            socket.inet_aton(address)
        except socket.error:
            return False
        return address.count('.') == 3
    except socket.error:  # not a valid address
        return False

    return True

def is_valid_ipv4_mask(netmask):
    if not is_valid_ipv4_address(netmask):
        return False
    a, b, c, d = (int(octet) for octet in netmask.split("."))
    mask = a << 24 | b << 16 | c << 8 | d

    if mask == 0:
        return True

    m = mask & -mask
    right0bits = -1
    while m:
        m >>= 1
        right0bits += 1

    if mask | ((1 << right0bits) - 1) != 0xffffffff:
        return False
    return True

def network_validation(config):
    for network in config.board.network:
        valid_address = is_valid_ipv4_address(network.IP)
        if not valid_address:
            return False, 'Invalid Board Network IP address'
        valid_mask = is_valid_ipv4_mask(network.mask)
        if not valid_mask:
            return False, 'Invalid Board Network netmask'
        valid_gateway = is_valid_ipv4_address(network.gateway)

    if not valid_gateway:
        return False, 'Invalid Board Network gateway'
    valid_address = is_valid_ipv4_address(config.output.protocol.UDP.destination)
    if not valid_address:
        return False, 'Invalid UDP destination address'
    config.output.protocol.UDP.port = int(config.output.protocol.UDP.port)
    if config.output.protocol.UDP.port < 1024 or config.output.protocol.UDP.port > 49151:
        return False, 'Invalid UDP destination port'
    valid_address = is_valid_ipv4_address(config.output.point_cloud.destination)
    if not valid_address:
        return False, 'Invalid Point cloud transfer address'

    return True, ''

def init_network(config, ifaces = None):
    setup_network(config, ifaces)

def setup_network(config, ifaces):
    from util.common_util import run_cmd
    for idx in range(len(config)):
        run_cmd('''nmcli connection delete "Wired connection {}"'''.format(idx))

    for idx, cfg in enumerate(config):
        if ifaces is not None and str(idx) not in ifaces:
            continue
        print('''setup network interface {} '''.format(idx))
        run_cmd('''nmcli connection delete eth{}'''.format(idx))
        run_cmd('''nmcli connection add type ethernet con-name eth{} ifname eth{}'''.format(idx, idx))
        if cfg.DHCP is True:
            nmcli_cmd = DHCP_TEMPLATE.format(idx)
        else:
            prefixlen = IPv4Network('''0.0.0.0/{}'''.format(cfg.mask)).prefixlen
            nmcli_cmd = STATIC_TEMPLATE.format(idx, cfg.IP, prefixlen, cfg.gateway)
        run_cmd(nmcli_cmd)
        run_cmd('''nmcli con up eth{}'''.format(idx))

    return True

_Global_Flag = '/tmp/load_network_success'
_Config_Path = '/tmp/init_interface'

if __name__ == '__main__':
    import sys
    import os
    sys.path.append(os.getcwd())

    from pathlib import Path
    from easydict import EasyDict
    import yaml
    from util.common_util import run_cmd
    config = EasyDict(yaml.safe_load(Path('cfg/board_cfg_all.yaml').open()))

    if not os.path.exists(_Global_Flag):
        init_network(config.board.network)
        run_cmd('touch ' + _Global_Flag)
    elif os.path.exists(_Config_Path):
        with open(_Config_Path, 'r') as f:
            init_interface = f.read()
        init_network(config.board.network, init_interface)
        os.remove(_Config_Path)
