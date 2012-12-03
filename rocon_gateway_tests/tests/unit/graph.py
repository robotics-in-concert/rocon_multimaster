#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_multimaster/master/rocon_gateway_graph/LICENSE
#
##############################################################################
# Imports
##############################################################################

import roslib
roslib.load_manifest('rocon_gateway')
import rospy
import rocon_utilities.console as console
from rocon_gateway import Graph
from rocon_gateway import GatewayError

##############################################################################
# Main
##############################################################################

if __name__ == '__main__':
    rospy.init_node('test_graph')
    graph = Graph()
    rospy.sleep(1.0)
    graph.update()
    print graph._local_gateway
    print graph._remote_gateways
