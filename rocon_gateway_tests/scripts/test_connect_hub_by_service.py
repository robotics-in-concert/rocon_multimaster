#!/usr/bin/env python
#       
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_multimaster/hydro-devel/rocon_gateway_tests/LICENSE 
#
##############################################################################
# Imports
##############################################################################

import sys
import rospy
import rocon_console.console as console
from rocon_gateway import samples
import gateway_msgs.msg as gateway_msgs
import unittest
import rosunit

##############################################################################
# Main
##############################################################################

class TestConnectHubByService(unittest.TestCase):

    def setUp(self):
        rospy.init_node('test_connect_hub_by_service')

    def test_connect_hub_by_service(self):
        print("\n********************************************************************")
        print("* Sending Connect Hub Request")
        print("********************************************************************")
        result, unused_error_message = samples.connect_hub_by_service(raise_exception=False)
        self.assertEquals(gateway_msgs.ErrorCodes.SUCCESS, result)
        
    def tearDown(self):
        pass

NAME = 'test_connect_hub_by_service'
if __name__ == '__main__':
    rosunit.unitrun('test_connect_hub_by_service', NAME, TestConnectHubByService, sys.argv, coverage_packages=['rocon_gateway'])
