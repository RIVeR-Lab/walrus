from walrus_system_configuration.util import *
from catkin.find_in_workspaces import find_in_workspaces
import sys
import os
from termcolor import colored

GRUB_CONFIG_FILE = '/etc/default/grub'
GRUB_BOOTFAIL_PARAMETER = 'GRUB_RECORDFAIL_TIMEOUT'

def bootfail_set():
    f = open(GRUB_CONFIG_FILE)
    found = False
    for line in f:
        if GRUB_BOOTFAIL_PARAMETER in line:
            found = True
    return found

def install():
    if bootfail_set():
        success("Bootfail parameter is set in grub config")
    else:
        warn("Cannot install boot fail parameter, you must add GRUB_" + GRUB_BOOTFAIL_PARAMETER + " to " + GRUB_CONFIG_FILE)

def status():
    if bootfail_set():
        success("Bootfail parameter is set in grub config")
    else:
        warn("Grub boot fail parameter is not set")
