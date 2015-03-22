import subprocess
from termcolor import colored
import os
import sys
from catkin.find_in_workspaces import find_in_workspaces

def sudo_call(cmd):
    full_cmd = ['sudo'] + cmd
    print 'Calling: ' + ' '.join(full_cmd)
    return subprocess.call(full_cmd)

def sudo_call_output(cmd):
    full_cmd = ['sudo'] + cmd
    print 'Calling: ' + ' '.join(full_cmd)
    return subprocess.check_output(full_cmd)

def sudo_rm(f):
    sudo_call(['rm', f])

def sudo_mv(src, dst):
    sudo_call(['mv', src, dst])

def sudo_cp(src, dst):
    sudo_call(['cp', src, dst])

def sudo_symlink(src, name):
    sudo_call(['ln', '-s', src, name])

def confirm(message):
    while True:
        result = raw_input(message + ' (y/n): ')
        if result == 'y':
            return True
        elif result == 'n':
            return False

def log(msg):
    sys.stdout.write(msg+'\n')

def warn(msg):
    sys.stdout.write(colored('WARN: '+msg+'\n', 'yellow'))

def error(msg):
    sys.stdout.write(colored('ERROR: '+msg+'\n', 'red'))

def success(msg):
    sys.stdout.write(colored(msg+'\n', 'green'))


def files_match(f1, f2):
    if not os.path.isfile(f1):
        return False
    if not os.path.isfile(f2):
        return False
    return open(f1).read() == open(f2).read()


def install_file(src, dest, name):
    if os.path.isfile(dest):
        if files_match(dest, src):
            success(name+' file is installed')
        else:
            error(name+' already exists, but is not correct')
            if confirm('Backup old file and install?'):
                sudo_mv(dest, dest+'~')
                sudo_cp(src, dest)
    else:
        error(name+' file not installed')
        if confirm('Install?'):
            sudo_cp(src, dest)

def status_file(src, dest, name):
    if files_match(src, dest):
        success(name+' file is installed')
    else:
        error(name+' file not installed')


def catkin_find(package, path):
    path_list = find_in_workspaces(project=package, path=path, first_matching_workspace_only=True)
    if len(path_list) == 0:
        return None
    return path_list[0]

def install_catkin_file(package, path, dest, name):
    f = catkin_find(package, path)
    if f is not None:
        install_file(f, dest, name)
    else:
        error('Could not locate '+name+' file')

def status_catkin_file(package, path, dest, name):
    f = catkin_find(package, path)
    if f is not None:
        status_file(f, dest, name)
    else:
        error('Could not locate '+name+' file')

def install_delete_file(f, name):
    if os.path.isfile(f):
        error(name+' file exists')
        if confirm('Delete?'):
            sudo_rm(f)
        success(name+' file does not exist')

def status_delete_file(f, name):
    if os.path.isfile(f):
        error(name+' file exists')
    else:
        success(name+' file does not exist')





def is_package_installed(name):
    devnull = open(os.devnull, 'w')
    command = ['dpkg-query', '-Wf\'${db:Status-abbrev}\'', name]
    if subprocess.call(command, stdout=devnull, stderr=devnull) == 0:
        return subprocess.check_output(command) == '\'ii \''
    return False

def install_package(name):
    if not is_package_installed(name):
        error(name+' is not installed')
        if confirm('Install?'):
            sudo_call(['apt-get', 'install', '-y', name])
    else:
        success(name+' is installed')

def status_package(name):
    if not is_package_installed(name):
        error(name+' is not installed')
    else:
        success(name+' is installed')

def service_status(name):
    try:
        status = sudo_call_output(['service', name, 'status'])
    except subprocess.CalledProcessError:
        status = 'not running'
    if 'start/running' in status or 'is running' in status:
        success(name+' is running')
    elif 'unrecognized service' in status:
        error(name+' is not installed')
    else:
        error(name+' is not running')

