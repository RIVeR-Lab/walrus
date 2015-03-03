from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    scripts=['install.py'],
    packages=['walrus_system_configuration'],
    package_dir={'': 'src'},
)

setup(**d)
