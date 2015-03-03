import os
from catkin.find_in_workspaces import find_in_workspaces
import subprocess
import sys
from termcolor import colored

UDEV_RULES_DIR = '/etc/udev/rules.d'

def sudo_call(cmd):
    full_cmd = ['sudo'] + cmd
    print 'Calling: ' + ' '.join(full_cmd)
    subprocess.call(full_cmd)

def sudo_mv(src, dst):
    sudo_call(['mv', src, dst])

def sudo_symlink(src, name):
    sudo_call(['ln', '-s', src, name])

def confirm(message):
    while True:
        result = raw_input(message + ' (y/n): ')
        if result == 'y':
            return True
        elif result == 'n':
            return False

def enumerate_udev_files():
    udev_path_list = find_in_workspaces(project='walrus_system_configuration', path='udev', first_matching_workspace_only=True)
    if len(udev_path_list) == 0:
        print 'ERROR: Could not locate walrus udev rules to install'
        return
    udev_path = udev_path_list[0]
    udev_files= [os.path.join(udev_path, f) for f in os.listdir(udev_path)]

    epos_udev_file_list = find_in_workspaces(project='epos_hardware', path='90-ftd2xx.rules', first_matching_workspace_only=True)
    if len(epos_udev_file_list) == 0:
        print 'ERROR: Could not locate epos udev rules to install'
        return
    udev_files.append(epos_udev_file_list[0])

    return udev_files

def install_udev():
    print 'Installing udev rules'

    for f in enumerate_udev_files():
        name = os.path.basename(f)
        installed_udev_file = os.path.join(UDEV_RULES_DIR, name)
        if os.path.isfile(installed_udev_file):
            if os.path.realpath(installed_udev_file) != f:
                sys.stdout.write(colored(name + ' already exists, but is not correct\n', 'red'))
                if confirm('Backup old file and install ' + name):
                    sudo_mv(installed_udev_file, installed_udev_file+'~')
                    sudo_symlink(f, installed_udev_file)
            else:
                sys.stdout.write(colored(name + ' already installed\n', 'green'))
        else:
            sys.stdout.write(colored(name + ' does not exist\n', 'yellow'))
            if confirm('Install ' + name + '?'):
                sys.stdout.write('Installing: ' + name+'\n')
                sudo_symlink(f, installed_udev_file)


def udev_status():
    print 'udev rules'
    udev_path_list = find_in_workspaces(project='walrus_system_configuration', path='udev', first_matching_workspace_only=True)
    if len(udev_path_list) == 0:
        print 'ERROR: Could not locate udev rules'
        return
    udev_path = udev_path_list[0]
    udev_file_names = os.listdir(udev_path)

    for f in enumerate_udev_files():
        name = os.path.basename(f)
        installed_udev_file = os.path.join(UDEV_RULES_DIR, name)
        if os.path.isfile(installed_udev_file):
            if os.path.realpath(installed_udev_file) != f:
                sys.stdout.write(colored(name + ' exists, but is not correct\n', 'yellow'))
            else:
                sys.stdout.write(colored(name + ' installed\n', 'green'))
        else:
            sys.stdout.write(colored(name + ' not installed\n', 'red'))


def install():
    install_udev()

def status():
    udev_status()
