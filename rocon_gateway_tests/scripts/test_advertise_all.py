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
from rocon_gateway import samples
from rocon_gateway import GatewaySampleRuntimeError
from rocon_gateway import Graph
from rocon_gateway import GatewayError
import unittest
import rosunit

##############################################################################
# Main
##############################################################################

class TestGraph(unittest.TestCase):

    def setUp(self):
        rospy.init_node('test_advertise_all')
        self.graph = Graph()

    def test_advertise_all(self):
        print("********************************************************************")
        print("* Advertise All")
        print("********************************************************************")
        try:
            samples.advertise_all()
        except GatewaySampleRuntimeError as e:
            self.fail("Runtime error caught when advertising all connections.")
        public_interface = None
        while not public_interface:
            rospy.loginfo("Public Interface")
            self.graph.update()
            public_interface = self.graph._local_gateway.public_interface
            rospy.sleep(0.2)
        print("%s" % self.graph._local_gateway)
        self.assertEquals("2", str(len(public_interface)))
        for rule in public_interface:
            self.assertEquals("/chatter", rule.name)
            # Should probably assert rule.type and rule.node here as well.
        # Revert state
        try:
            samples.advertise_all(cancel=True) 
        except GatewaySampleRuntimeError as e:
            self.fail("Runtime error caught when unadvertising all connections.")

    def tearDown(self):
        pass

NAME = 'test_graph'
if __name__ == '__main__':
    rosunit.unitrun('test_graph', NAME, TestGraph, sys.argv, coverage_packages=['rocon_gateway'])
        
