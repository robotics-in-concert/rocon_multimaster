#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['rocon_utilities'],
    package_dir={'': 'src'},
    scripts=['scripts/rocon_launch',
             ],
    requires=['rospy', 'rocon_std_msgs']
)

setup(**d)

