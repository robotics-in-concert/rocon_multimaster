#!/usr/bin/env python
#       
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_multimaster/license/LICENSE
#
##############################################################################
# Imports
##############################################################################

from __future__ import print_function

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

def test_connection_cache():
    master = rosgraph.Master(rospy.get_name())
    connection_cache = rocon_gateway.ConnectionCache(master.getSystemState)

    new_system_state = {}
    new_system_state[PUBLISHER] = [['/chatter', ['/talker']]]
    new_system_state[SUBSCRIBER] = []
    new_system_state[SERVICE] = []
    connection_cache.update(new_system_state)
    new_system_state[PUBLISHER] = [['/chatter', ['/talker', '/babbler']]]
    new_connections, lost_connections = connection_cache.update(new_system_state)
    assert "/babbler", new_connections[PUBLISHER][0].rule.node
