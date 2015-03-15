from walrus_system_configuration.util import *
from catkin.find_in_workspaces import find_in_workspaces
import sys
import os
from termcolor import colored

DHCP_CONFIG_FILE = '/etc/dhcp/dhcpd.conf'

def install():
    install_catkin_file('walrus_system_configuration', 'dhcpd.conf', DHCP_CONFIG_FILE, "DHCP config")

    install_package('isc-dhcp-server')

def status():
    status_catkin_file('walrus_system_configuration', 'dhcpd.conf', DHCP_CONFIG_FILE, "DHCP config")

    status_package('isc-dhcp-server')
    service_status('isc-dhcp-server')
