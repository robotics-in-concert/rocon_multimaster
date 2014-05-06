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
import rocon_console.console as console
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
        rospy.init_node('test_advertisements')
        self.graph = Graph()

    def test_advertise_all(self):
        print("\n********************************************************************")
        print("* Advertise All")
        print("********************************************************************")
        try:
            samples.advertise_all()
        except GatewaySampleRuntimeError as e:
            self.fail("Runtime error caught when advertising all connections.")
        public_interface = self._wait_for_public_interface()
        #print("%s" % self.graph._local_gateway)
        self.assertEquals("2", str(len(public_interface)))
        for rule in public_interface:
            self.assertEquals("/chatter", rule.name)
            # Should probably assert rule.type and rule.node here as well.
        # Revert state
        try:
            samples.advertise_all(cancel=True)
        except GatewaySampleRuntimeError as e:
            self.fail("Runtime error caught when unadvertising all connections.")
        self._assert_cleared_public_interface()

    def test_advertise_tutorials(self):
        print("\n********************************************************************")
        print("* Advertise Tutorials")
        print("********************************************************************")
        try:
            samples.advertise_tutorials() 
        except GatewaySampleRuntimeError as e:
            self.fail("Runtime error caught when advertising tutorial connections.")
        public_interface = self._wait_for_public_interface()
        #print("%s" % self.graph._local_gateway)
        self.assertIn("/chatter", [rule.name for rule in public_interface])
        try:
            samples.advertise_tutorials(cancel=True) 
        except GatewaySampleRuntimeError as e:
            self.fail("Runtime error caught when unadvertising tutorial connections.")
        self._assert_cleared_public_interface()

    def test_advertise_regex_tutorials(self):
        print("\n********************************************************************")
        print("* Advertise Regex Tutorials")
        print("********************************************************************")
        try:
            samples.advertise_tutorials(regex_patterns=True) 
        except GatewaySampleRuntimeError as e:
            self.fail("Runtime error caught when advertising tutorial connections.")
        public_interface = self._wait_for_public_interface()
        print("%s" % self.graph._local_gateway)
        self.assertIn("/chatter", [rule.name for rule in public_interface])
        try:
            samples.advertise_tutorials(cancel=True, regex_patterns=True) 
        except GatewaySampleRuntimeError as e:
            self.fail("Runtime error caught when unadvertising tutorial connections.")
        self._assert_cleared_public_interface()
        
    def tearDown(self):
        pass

    ##########################################################################
    # Utility methods
    ##########################################################################

    def _wait_for_public_interface(self):
        public_interface = None
        while not public_interface:
            self.graph.update()
            public_interface = self.graph._local_gateway.public_interface
            rospy.rostime.wallsleep(0.2)
        return public_interface

    def _assert_cleared_public_interface(self):
        start_time = rospy.Time.now()
        while True:
            self.graph.update()
            public_interface = self.graph._local_gateway.public_interface
            if public_interface:
                result = "cleared"
                break
            else:
                rospy.rostime.wallsleep(0.2)
            if rospy.Time.now() - start_time > rospy.Duration(2.0):
                result = "timed out waiting for public interface to clear"
                break
        self.assertEqual("cleared", result)

NAME = 'test_graph'
if __name__ == '__main__':
    rosunit.unitrun('test_graph', NAME, TestGraph, sys.argv, coverage_packages=['rocon_gateway'])
        
