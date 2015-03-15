from walrus_system_configuration.udev import *
from walrus_system_configuration.grub import *
from walrus_system_configuration.dhcp import *
from walrus_system_configuration.routing import *

def install():
    udev.install()
    grub.install()
    dhcp.install()
    routing.install()
    return 0

def status():
    udev.status()
    grub.status()
    dhcp.status()
    routing.status()
    return 0
