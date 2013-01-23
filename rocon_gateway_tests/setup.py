#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    #packages=['rocon_gateway_tests'],
    #package_dir={'': 'src'},
    requires=['rospy', 'rostest', 'rocon_gateway', 'roscpp_tutorials', 'rospy_tutorials', 'actionlib_tutorials', 'gateway_msgs']
)

setup(**d)
