from walrus_system_configuration.util import *
from catkin.find_in_workspaces import find_in_workspaces
import sys
import os
from termcolor import colored

SYSINIT_CONFIG_FILE = '/etc/init/rc-sysinit.conf'

def wait_for_net():
    f = open(SYSINIT_CONFIG_FILE)
    found = False
    for line in f:
        if 'static-network-up' in line and line.strip().startswith('start'):
            found = True
    return found

def install():
    if wait_for_net():
        warn("Cannot fix boot without network, you must remove 'and static-network-up' from the start line in " + SYSINIT_CONFIG_FILE)
    else:
        success("Not waiting for network on boot")

def status():
    if wait_for_net():
        warn("Waiting for network on boot")
    else:
        success("Not waiting for network on boot")
