from walrus_system_configuration.util import *
import sys
import os
from termcolor import colored

BIND_NAMED_CONF_LOCAL_FILE = '/etc/bind/named.conf.local'
WALRUS_BIND_CONF_FILE = '/etc/bind/walrus'


def install():
    install_package('bind9')
    install_catkin_file('walrus_system_configuration', 'bind_named.conf.local', BIND_NAMED_CONF_LOCAL_FILE, "BIND named local config")
    install_catkin_file('walrus_system_configuration', 'walrus_bind_conf', WALRUS_BIND_CONF_FILE, "WALRUS BIND config")


def status():
    status_package('bind9')
    service_status('bind9')
    status_catkin_file('walrus_system_configuration', 'bind_named.conf.local', BIND_NAMED_CONF_LOCAL_FILE, "BIND named local config")
    status_catkin_file('walrus_system_configuration', 'walrus_bind_conf', WALRUS_BIND_CONF_FILE, "WALRUS BIND config")
