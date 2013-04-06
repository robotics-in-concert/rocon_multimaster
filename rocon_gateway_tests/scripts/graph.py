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
        rospy.sleep(1.0)
        self.graph.update()

    def test_graph(self):
        print("********************************************************************")
        print("* Local Gateway")
        print("********************************************************************")
        print self.graph._local_gateway
        print("********************************************************************")
        print("* Remote Gateway")
        print("********************************************************************")
        print self.graph._remote_gateways
        for remote_gateway in self.graph._remote_gateways:
            self.assertEquals("remote_gateway", remote_gateway.name)

    def tearDown(self):
        pass

NAME = 'test_graph'
if __name__ == '__main__':
    rosunit.unitrun('test_graph', NAME, TestGraph, sys.argv, coverage_packages=['rocon_gateway'])

        
