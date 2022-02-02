#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    packages=['hmi_ros', 'hmi_ros.outputs', 'hmi_ros.inputs', 'hmi_ros.msg'],
    package_dir={'': 'include', '': 'msg'},
)

setup(**setup_args)