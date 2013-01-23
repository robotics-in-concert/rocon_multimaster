#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['rocon_hub_client'],
    package_dir={'': 'src'},
    requires=['redis', 'rospy', 'rocon_gateway']
)

setup(**d)
