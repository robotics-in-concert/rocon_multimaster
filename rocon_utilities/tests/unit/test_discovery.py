#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_multimaster/license/LICENSE
#

##############################################################################
# Imports
##############################################################################

from __future__ import print_function
import os
from rocon_utilities import package_index_from_package_path

##############################################################################
# Test Class
##############################################################################

def test_package_index_discovery():
    print('Testing package index discovery')
    ros_package_path = os.getenv('ROS_PACKAGE_PATH', '')
    ros_package_path = [x for x in ros_package_path.split(':') if x]
    package_index = package_index_from_package_path(ros_package_path)
    assert 'rocon_utilities' in package_index.keys() 
