from walrus_system_configuration.udev import *
from walrus_system_configuration.grub import *

def install():
    udev.install()
    grub.install()
    return 0

def status():
    udev.status()
    grub.status()
    return 0
