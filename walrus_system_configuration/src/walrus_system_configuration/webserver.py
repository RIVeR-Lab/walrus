from walrus_system_configuration.util import *
from catkin.find_in_workspaces import find_in_workspaces
import sys
import os
from termcolor import colored

NGINX_CONFIG_FILE = '/etc/nginx/conf.d/walrus.conf'
DEFAULT_NGINX_SITE_FILE = '/etc/nginx/sites-enabled/default'

def install():
    install_catkin_file('walrus_system_configuration', 'nginx.conf', NGINX_CONFIG_FILE, "NGINX config")

    install_package('nginx')
    install_delete_file(DEFAULT_NGINX_SITE_FILE, 'default nginx site')

def status():
    status_catkin_file('walrus_system_configuration', 'nginx.conf', NGINX_CONFIG_FILE, "NGINX config")

    status_package('nginx')
    service_status('nginx')
    status_delete_file(DEFAULT_NGINX_SITE_FILE, 'default nginx site')
