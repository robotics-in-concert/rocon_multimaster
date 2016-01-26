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
from rocon_gateway import Graph
from rocon_gateway import GatewayError
import unittest
import rosunit
import rocon_gateway_utils


##############################################################################
# Logging
##############################################################################

def printtest(msg):
    print("[TEST] %s" % msg)


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
            printtest("Waiting for flips")
            self.graph.update()
            flips = self.graph._local_gateway.flip_watchlist
            rospy.rostime.wallsleep(0.2)
        printtest("********************************************************************")
        printtest("* Local Gateway")
        printtest("********************************************************************")
        printtest("%s" % self.graph._local_gateway)
        self.assertEquals("5", str(len(flips)))
        # TODO: this is currently returning the base name, is should be returning the hash name

        self.assertEquals(len([flip for flip in flips if flip.gateway == "remote_gateway" and flip.rule.name == "/add_two_ints" and flip.rule.type == "service"]), 1)
        self.assertEquals(len([flip for flip in flips if flip.gateway == "remote_gateway" and flip.rule.name == "/chatter" and flip.rule.type == "publisher"]), 1)
        self.assertEquals(len([flip for flip in flips if flip.gateway == "remote_gateway" and flip.rule.name == "/chatter" and flip.rule.type == "subscriber"]), 1)
        self.assertEquals(len([flip for flip in flips if flip.gateway == "remote_gateway" and flip.rule.name == "/fibonacci" and flip.rule.type == "action_server"]), 1)
        self.assertEquals(len([flip for flip in flips if flip.gateway == "remote_gateway" and flip.rule.name == "/fibonacci" and flip.rule.type == "action_client"]), 1)
        
        printtest("********************************************************************")
        printtest("* Remote Gateway")
        printtest("********************************************************************")
        printtest("%s" % self.graph._remote_gateways)
        for remote_gateway in self.graph._remote_gateways:
            self.assertEquals("remote_gateway", rocon_gateway_utils.gateway_basename(remote_gateway.name))

    def tearDown(self):
        pass

NAME = 'test_graph'
if __name__ == '__main__':
    rosunit.unitrun('test_graph', NAME, TestGraph, sys.argv, coverage_packages=['rocon_gateway'])
        
