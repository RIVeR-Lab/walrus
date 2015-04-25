from walrus_system_configuration.util import *
from catkin.find_in_workspaces import find_in_workspaces
import sys
import os
from termcolor import colored

UPSTART_CONFIG_FILE = '/etc/init/ros.conf'

def install():
    install_catkin_file('walrus_system_configuration', 'upstart.conf', UPSTART_CONFIG_FILE, "Upstart config")

def status():
    status_catkin_file('walrus_system_configuration', 'upstart.conf', UPSTART_CONFIG_FILE, "Upstart config")
