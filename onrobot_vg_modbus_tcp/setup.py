## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['onrobot_vg_modbus_tcp'],
    package_dir={'': 'src'},
    requires=['rospy', 'python-pymodbus'],
)

setup(**setup_args)