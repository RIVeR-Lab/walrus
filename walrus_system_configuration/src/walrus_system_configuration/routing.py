from walrus_system_configuration.util import *
import sys
import os
from termcolor import colored

SYSCTL_CONFIG_FILE = '/etc/sysctl.conf'
FORWARD_CONFIG_LINE = "net.ipv4.ip_forward=1"
IPTABLES_V4_CONFIG_FILE = '/etc/iptables/rules.v4'

def forwarding_set():
    f = open(SYSCTL_CONFIG_FILE)
    found = False
    for line in f:
        if line == FORWARD_CONFIG_LINE+'\n':
            found = True
    return found

def install():
    if forwarding_set():
        success("IPv4 forwarding setup")
    else:
        warn("Cannot configure IPv4 forwarding, you must add " + FORWARD_CONFIG_LINE + " to " + SYSCTL_CONFIG_FILE)

    install_package('iptables-persistent')

    install_catkin_file('walrus_system_configuration', 'rules.v4', IPTABLES_V4_CONFIG_FILE, "iptables config")


def status():
    if forwarding_set():
        success("IPv4 forwarding setup")
    else:
        warn("IPv4 forwarding is not setup")

    status_package('iptables-persistent')
    status_catkin_file('walrus_system_configuration', 'rules.v4', IPTABLES_V4_CONFIG_FILE, "iptables config")
