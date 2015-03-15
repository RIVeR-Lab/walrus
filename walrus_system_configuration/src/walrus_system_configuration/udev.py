from walrus_system_configuration.util import *
from catkin.find_in_workspaces import find_in_workspaces
import os

UDEV_RULES_DIR = '/etc/udev/rules.d'

def enumerate_udev_files():
    udev_path_list = find_in_workspaces(project='walrus_system_configuration', path='udev', first_matching_workspace_only=True)
    if len(udev_path_list) == 0:
        error('Could not locate walrus udev rules to install')
        return
    udev_path = udev_path_list[0]
    udev_files= [os.path.join(udev_path, f) for f in os.listdir(udev_path)]

    epos_udev_file_list = find_in_workspaces(project='epos_hardware', path='90-ftd2xx.rules', first_matching_workspace_only=True)
    if len(epos_udev_file_list) == 0:
        error('Could not locate epos udev rules to install')
        return
    udev_files.append(epos_udev_file_list[0])

    return udev_files

def install():
    log('Installing udev rules')

    for f in enumerate_udev_files():
        name = os.path.basename(f)
        installed_udev_file = os.path.join(UDEV_RULES_DIR, name)
        if os.path.isfile(installed_udev_file):
            if os.path.realpath(installed_udev_file) != f:
                error(name + ' already exists, but is not correct\n', 'red')
                if confirm('Backup old file and install ' + name):
                    sudo_mv(installed_udev_file, installed_udev_file+'~')
                    sudo_symlink(f, installed_udev_file)
            else:
                success(name + ' already installed')
        else:
            warn(name + ' does not exist')
            if confirm('Install ' + name + '?'):
                log('Installing: ' + name)
                sudo_symlink(f, installed_udev_file)


def status():
    log('udev rules')
    udev_path_list = find_in_workspaces(project='walrus_system_configuration', path='udev', first_matching_workspace_only=True)
    if len(udev_path_list) == 0:
        error('Could not locate udev rules')
        return
    udev_path = udev_path_list[0]
    udev_file_names = os.listdir(udev_path)

    for f in enumerate_udev_files():
        name = os.path.basename(f)
        installed_udev_file = os.path.join(UDEV_RULES_DIR, name)
        if os.path.isfile(installed_udev_file):
            if os.path.realpath(installed_udev_file) != f:
                warn(name + ' exists, but is not correct')
            else:
                success(name + ' installed')
        else:
            error(name + ' not installed')
