#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_multimaster/license/LICENSE
#

##############################################################################
# Imports
##############################################################################

# enable some python3 compatibility options:
# (unicode_literals not compatible with python2 uuid module)
from __future__ import absolute_import, print_function

import uuid
import unittest
import rosunit

# module being tested:
import rocon_utilities.platform_info
import rocon_std_msgs.msg as rocon_std_msgs

TEST_UUID_HEX = '0123456789abcdef0123456789abcdef'
TEST_UUID = uuid.UUID(hex=TEST_UUID_HEX)

##############################################################################
# Test Class
##############################################################################

class TestPlatformInfo(unittest.TestCase):
    """Unit tests for platform info manipulation.

    These tests do not require a running ROS core.
    """

    def test_message_to_string(self):
        msg = rocon_std_msgs.PlatformInfo()
        msg.os = 'linux'
        msg.version = rocon_std_msgs.PlatformInfo.VERSION_ANY
        msg.system = rocon_std_msgs.PlatformInfo.SYSTEM_ROS
        msg.platform = rocon_std_msgs.PlatformInfo.PLATFORM_TURTLEBOT
        msg.name = 'cybernetic_pirate'
        tuple = rocon_utilities.platform_info.to_string(msg)
        self.assertEqual(tuple, 'linux.*.ros.turtlebot.cybernetic_pirate')

    def test_string_to_message(self):
        tuple = 'linux.*.ros.turtlebot.cybernetic_pirate'
        msg = rocon_utilities.platform_info.to_msg(tuple)
        print("Msg: %s" %msg)
        self.assertEqual(rocon_std_msgs.PlatformInfo.VERSION_ANY, msg.version)
        self.assertEqual(rocon_std_msgs.PlatformInfo.SYSTEM_ROS, msg.system)
        self.assertEqual(rocon_std_msgs.PlatformInfo.PLATFORM_TURTLEBOT, msg.platform)
        self.assertEqual('cybernetic_pirate', msg.name)

    def test_set_name(self):
        tuple = 'linux.*.ros.turtlebot.unknown'
        new_tuple = rocon_utilities.platform_info.set_name(tuple, 'cybernetic_pirate')
        new_name = rocon_utilities.platform_info.get_name(new_tuple)
        self.assertEqual(new_tuple, 'linux.*.ros.turtlebot.cybernetic_pirate')
        self.assertEqual(new_name, 'cybernetic_pirate')


    def test_matches(self):
        tuple_a = 'linux.*.ros.turtlebot.cybernetic_pirate'
        tuple_b = 'linux.*.ros.*.cybernetic_pirate'
        msg_a = rocon_utilities.platform_info.to_msg(tuple_a)
        msg_b = rocon_utilities.platform_info.to_msg(tuple_b)
        self.assertEqual(rocon_utilities.platform_info.string_matches(tuple_a, tuple_b), True)
        self.assertEqual(rocon_utilities.platform_info.string_matches(tuple_b, tuple_a), False)
        self.assertEqual(rocon_utilities.platform_info.matches(msg_a, msg_b), True)
        self.assertEqual(rocon_utilities.platform_info.matches(msg_b, msg_a), False)
        
if __name__ == '__main__':
    rosunit.unitrun('rocon_utilities_platform_info',
                    'test_platform_info',
                    TestCommonModule)
