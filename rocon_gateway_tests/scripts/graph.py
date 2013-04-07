#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_multimaster/hydro_devel/rocon_gateway_tests/LICENSE
#
##############################################################################
# Imports
##############################################################################

import rospy
import sys
import rocon_utilities.console as console
from rocon_gateway import Graph
from rocon_gateway import GatewayError
import unittest
import rosunit

##############################################################################
# Main
##############################################################################

class TestGraph(unittest.TestCase):

    def setUp(self):
        rospy.init_node('test_graph')
        self.graph = Graph()

    def test_graph(self):
        flips = None
        while not flips:
            rospy.loginfo("Got flips")
            self.graph.update()
            flips = self.graph._local_gateway.flip_watchlist
            rospy.sleep(0.2)
        print("********************************************************************")
        print("* Local Gateway")
        print("********************************************************************")
        print("%s" % self.graph._local_gateway)
        self.assertEquals("1", str(len(flips)))
        self.assertEquals("remote_gateway", flips[0].gateway)
        self.assertEquals("publisher", flips[0].rule.type)
        self.assertEquals("/chatter", flips[0].rule.name)
        
        print("********************************************************************")
        print("* Remote Gateway")
        print("********************************************************************")
        print("%s" % self.graph._remote_gateways)
        for remote_gateway in self.graph._remote_gateways:
            self.assertEquals("remote_gateway", remote_gateway.name)

    def tearDown(self):
        pass

NAME = 'test_graph'
if __name__ == '__main__':
    rosunit.unitrun('test_graph', NAME, TestGraph, sys.argv, coverage_packages=['rocon_gateway'])
        
