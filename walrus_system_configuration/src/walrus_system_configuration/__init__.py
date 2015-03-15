from walrus_system_configuration.udev import *
from walrus_system_configuration.grub import *
from walrus_system_configuration.dhcp import *

def install():
    udev.install()
    grub.install()
    dhcp.install()
    return 0

def status():
    udev.status()
    grub.status()
    dhcp.status()
    return 0
