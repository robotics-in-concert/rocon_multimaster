#!/usr/bin/env python
#       
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_multimaster/hydro-devel/rocon_gateway_tests/LICENSE 
#
##############################################################################
# Imports
##############################################################################

import sys
import unittest
import rospy
import rocon_gateway
from std_msgs.msg import String
import rosunit
import rosgraph
from gateway_msgs.msg import Rule, ConnectionType

##############################################################################
# Aliases
##############################################################################

# Can't see an easier way to alias or import these
PUBLISHER = ConnectionType.PUBLISHER
SUBSCRIBER = ConnectionType.SUBSCRIBER
SERVICE = ConnectionType.SERVICE
ACTION_SERVER = ConnectionType.ACTION_SERVER
ACTION_CLIENT = ConnectionType.ACTION_CLIENT

##############################################################################
# Test
##############################################################################

class TestConnectionCache(unittest.TestCase):

    def setUp(self):
        master = rosgraph.Master(rospy.get_name())
        self.connection_cache = rocon_gateway.ConnectionCache(master.getSystemState)
        pass

    def test_update(self):
        new_system_state = {}
        new_system_state[PUBLISHER] = [['/chatter', ['/talker']]]
        new_system_state[SUBSCRIBER] = []
        new_system_state[SERVICE] = []
        self.connection_cache.update(new_system_state)
        new_system_state[PUBLISHER] = [['/chatter', ['/talker', '/babbler']]]
        new_connections, lost_connections = self.connection_cache.update(new_system_state)
        self.assertEquals("/babbler", new_connections[PUBLISHER][0].rule.node)
#        print("Node: %s" % new_connections[PUBLISHER][0].rule.node)
#        print("New Publishers:")
#        for connection in new_connections[PUBLISHER]:
#            print("  %s" % connection)

    def tearDown(self):
        pass

NAME = 'test_connection_cache'
if __name__ == '__main__':
    rosunit.unitrun('test_connection_cache', NAME, TestConnectionCache, sys.argv, coverage_packages=['rocon_gateway'])
