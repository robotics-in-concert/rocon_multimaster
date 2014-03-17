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

class TestFlips(unittest.TestCase):

    def setUp(self):
        rospy.init_node('test_pullments')
        self.graph = Graph()

    def test_flip_all(self):
        print("\n********************************************************************")
        print("* Flip All")
        print("********************************************************************")
        try:
            samples.flip_all()
        except GatewaySampleRuntimeError as e:
            self.fail("Runtime error caught when flipping all connections [%s]" % str(e))
        flipped_interface = self._wait_for_flipped_interface()
        #print("%s" % self.graph._local_gateway)
        self.assertEquals("2", str(len(flipped_interface)))
        for remote_rule in flipped_interface:
            self.assertEquals("/chatter", remote_rule.remote_rule.rule.name)
            # Should probably assert rule.type and rule.node here as well.
        # Revert state
        try:
            samples.flip_all(cancel=True)
        except GatewaySampleRuntimeError as e:
            self.fail("Runtime error caught when unflipping all connections [%s]" % str(e))
        self._assert_cleared_flipped_interface()

    def test_flip_tutorials(self):
        print("\n********************************************************************")
        print("* Flip Tutorials")
        print("********************************************************************")
        try:
            samples.flip_tutorials() 
        except GatewaySampleRuntimeError as e:
            self.fail("Runtime error caught when flipping tutorial connections.")
        flipped_interface = self._wait_for_flipped_interface()
        #print("%s" % self.graph._local_gateway)
        self.assertIn("/chatter", [remote_rule.remote_rule.rule.name for remote_rule in flipped_interface])
        try:
            samples.flip_tutorials(cancel=True) 
        except GatewaySampleRuntimeError as e:
            self.fail("Runtime error caught when unflipping tutorial connections.")
        self._assert_cleared_flipped_interface()

    def test_flip_regex_tutorials(self):
        print("\n********************************************************************")
        print("* Flip Regex Tutorials")
        print("********************************************************************")
        try:
            samples.flip_tutorials(regex_patterns=True) 
        except GatewaySampleRuntimeError as e:
            self.fail("Runtime error caught when flipping tutorial connections.")
        flipped_interface = self._wait_for_flipped_interface()
        print("%s" % self.graph._local_gateway)
        self.assertIn("/chatter", [remote_rule.remote_rule.rule.name for remote_rule in flipped_interface])
        try:
            samples.flip_tutorials(cancel=True, regex_patterns=True) 
        except GatewaySampleRuntimeError as e:
            self.fail("Runtime error caught when unflipping tutorial connections.")
        self._assert_cleared_flipped_interface()
        
    def tearDown(self):
        pass

    ##########################################################################
    # Utility methods
    ##########################################################################

    def _wait_for_flipped_interface(self):
        flipped_interface = None
        while not flipped_interface:
            self.graph.update()
            flipped_interface = self.graph._local_gateway.flipped_connections
            rospy.sleep(0.2)
        return flipped_interface

    def _assert_cleared_flipped_interface(self):
        start_time = rospy.Time.now()
        while True:
            self.graph.update()
            flipped_interface = self.graph._local_gateway.flipped_connections
            if flipped_interface:
                result = "cleared"
                break
            else:
                rospy.sleep(0.2)
            if rospy.Time.now() - start_time > rospy.Duration(1.0):
                result = "timed out waiting for flipped interface to clear"
                break
        self.assertEqual("cleared", result)

NAME = 'test_flips'
if __name__ == '__main__':
    rosunit.unitrun('test_flips', NAME, TestFlips, sys.argv, coverage_packages=['rocon_gateway'])
        
