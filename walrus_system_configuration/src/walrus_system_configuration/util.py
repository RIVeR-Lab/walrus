import subprocess
from termcolor import colored
import sys

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

def log(msg):
    sys.stdout.write(msg+'\n')

def warn(msg):
    sys.stdout.write(colored('WARN: '+msg+'\n', 'yellow'))

def error(msg):
    sys.stdout.write(colored('ERROR: '+msg+'\n', 'red'))

def success(msg):
    sys.stdout.write(colored(msg+'\n', 'green'))
