from walrus_system_configuration.udev import *
from walrus_system_configuration.grub import *
from walrus_system_configuration.dhcp import *
from walrus_system_configuration.routing import *
from walrus_system_configuration.dns import *

def install():
    udev.install()
    grub.install()
    dhcp.install()
    routing.install()
    dns.install()
    return 0

def status():
    udev.status()
    grub.status()
    dhcp.status()
    routing.status()
    dns.status()
    return 0
