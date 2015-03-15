from walrus_system_configuration.util import *
from catkin.find_in_workspaces import find_in_workspaces
import sys
import os
from termcolor import colored
from catkin.find_in_workspaces import find_in_workspaces

DHCP_CONFIG_FILE = '/etc/dhcp/dhcpd.conf'

def config_file_to_install():
    config_path_list = find_in_workspaces(project='walrus_system_configuration', path='dhcpd.conf', first_matching_workspace_only=True)
    if len(config_path_list) == 0:
        error('Could not locate DHCP config file to install')
        return None
    return config_path_list[0]

def files_match(f1, f2):
    if not os.path.isfile(f1):
        return False
    if not os.path.isfile(f2):
        return False
    return open(f1).read() == open(f2).read()

def install():
    f = config_file_to_install()
    if f is None:
        return

    if os.path.isfile(DHCP_CONFIG_FILE):
        if files_match(DHCP_CONFIG_FILE, f):
            success('DHCP config file is installed')
        else:
            error('DHCP config already exists, but is not correct')
            if confirm('Backup old file and install config'):
                sudo_mv(DHCP_CONFIG_FILE, DHCP_CONFIG_FILE+'~')
                sudo_cp(f, DHCP_CONFIG_FILE)
    else:
        error('DHCP config file not installed')
        if confirm('Install config?'):
            sudo_cp(f, DHCP_CONFIG_FILE)

def status():
    f = config_file_to_install()
    if f is None:
        return

    if files_match(DHCP_CONFIG_FILE, f):
        success('DHCP config file is installed')
    else:
        error('DHCP config file not installed')

    if 'start/running' in sudo_call_output(['service', 'isc-dhcp-server', 'status'], output=False):
        success('DHCP Server is running')
    else:
        error('DHCP Server is not running')
