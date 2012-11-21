#!/usr/bin/env python
#       
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_multimaster/master/rocon_gateway_tests/LICENSE 
#

PKG = 'rocon_gateway_tests'
import roslib; roslib.load_manifest(PKG)
import rospy
import rostest
from gateway_msgs.msg import *
from gateway_msgs.srv import *
import unittest
import std_msgs
import copy

class TestAdvertisementsLocally(unittest.TestCase):

    def assertAdvertiseAllCall(self, blacklist, expected_interface):
        req = AdvertiseAllRequest()
        req.cancel = False
        req.blacklist = copy.deepcopy(blacklist)

        # make call and ensure it succeeded
        resp = self.advertiseAll(req)
        self.assertEquals(resp.result, Result.SUCCESS)

        # ensure expected interfaces came up
        self.assertPublicInterface(expected_interface)

        # remove everything
        req.cancel = True
        resp = self.advertiseAll(req)
        self.assertEquals(resp.result, Result.SUCCESS)

        self.assertPublicInterface([])

    def assertAdvertiseCall(self, watchlist, expected_interface):

        # Make the call
        req = AdvertiseRequest()
        req.rules = copy.deepcopy(watchlist)
        req.cancel = False

        # ensure call succeeded
        resp = self.advertise(req)
        self.assertEquals(resp.result, Result.SUCCESS)
        self.assertEquals(len(resp.watchlist), len(watchlist))
        for rule in resp.watchlist:
            self.assertIn(rule, watchlist)

        # ensure public interface comes up as expected
        self.assertPublicInterface(expected_interface)

        # remove everything and check if everything has been removed
        req.cancel = True
        resp = self.advertise(req)
        self.assertEquals(resp.result, Result.SUCCESS)
        self.assertEquals(len(resp.watchlist), 0)
        self.assertPublicInterface([])

    def assertPublicInterface(self, rules):

        expected_rules = copy.deepcopy(rules)
        num_nodes = len(expected_rules)
        while True:
            rospy.sleep(1.0)
            resp = self.gatewayInfo()
            node_names = []
            for i in resp.public_interface:
                node_names.append(i.node)
            rospy.loginfo("TEST: Public Interface Nodes: %s"%str(node_names))
            if len(resp.public_interface) == num_nodes:
                break
            rospy.loginfo("TEST:   Waiting for watcher thread to load all nodes.")

        for i in range(num_nodes):
            self.assertIn(resp.public_interface[i], expected_rules)
            expected_rules.remove(resp.public_interface[i])

        self.assertEqual(len(expected_rules), 0);
        rospy.loginfo("TEST: Public interface found as expected")

    def setUp(self):
        '''
          Run at the start of every test. This function performs the following:
            1. Ensure that gateway services are available
            2. Make sure gateway is connected to the hub
          If something goes wrong and the setup can not complete successfully,
          the test will timeout with an error
        '''
        rospy.wait_for_service('/gateway/advertise')
        rospy.wait_for_service('/gateway/advertise_all')
        rospy.wait_for_service('/gateway/gateway_info')

        self.advertise = rospy.ServiceProxy('/gateway/advertise',Advertise)
        self.advertiseAll = rospy.ServiceProxy('/gateway/advertise_all',AdvertiseAll)
        self.gatewayInfo = rospy.ServiceProxy('/gateway/gateway_info',GatewayInfo)

        # Make sure we are connected to the gateway first!
        while True:
            rospy.sleep(1.0)
            resp = self.gatewayInfo()
            if resp.connected:
                break
            rospy.loginfo("TEST: Waiting for local gateway to be connected...")

        rospy.loginfo("TEST: Local gateway connected")

        # unit test property - show the difference when an assertion fails
        self.maxDiff = None

    def test_advertisePublisherByTopic(self):
        '''
          Tests advertising publishers. Also tests that multiple nodes are
          coming up as expected.
        '''
        # Test topic name directly
        watchlist = [Rule(ConnectionType.PUBLISHER, "/chatter", '')]
        expected_interface = list()
        expected_interface.append(Rule(ConnectionType.PUBLISHER, "/chatter", "/talker"))
        expected_interface.append(Rule(ConnectionType.PUBLISHER, "/chatter", "/talker2"))
        self.assertAdvertiseCall(watchlist, expected_interface)

        # Test topic name using regex
        watchlist = [Rule(ConnectionType.PUBLISHER, "/chat.*", '')]
        self.assertAdvertiseCall(watchlist, expected_interface)

        # Test bogus regex
        watchlist = [Rule(ConnectionType.PUBLISHER, "/chattt.*", '')]
        self.assertAdvertiseCall(watchlist, [])

    def test_advertisePublisherByNode(self):
        '''
          Tests advertising publishers. Also tests that multiple nodes are
          coming up as expected.
        '''
        watchlist = [Rule(ConnectionType.PUBLISHER, "/chatter", "/talker")]
        expected_interface = list()
        expected_interface.append(Rule(ConnectionType.PUBLISHER, "/chatter", "/talker"))
        self.assertAdvertiseCall(watchlist, expected_interface)

        # Test using regex
        watchlist = [Rule(ConnectionType.PUBLISHER, "/chat.*", ".*ker")]
        self.assertAdvertiseCall(watchlist, expected_interface)

    def test_advertiseDifferentConnections(self):
        '''
          Makes sure that every connection type is being detected and advertised
          appropriately.
        '''
        topics = {}
        topics[ConnectionType.PUBLISHER] = "/chatter"
        topics[ConnectionType.SUBSCRIBER] = "/chatter"
        topics[ConnectionType.SERVICE] = "/add_two_ints"
        topics[ConnectionType.ACTION_SERVER] = "/averaging_server/"
        topics[ConnectionType.ACTION_CLIENT] = "/fibonacci/"
        nodes = {}
        nodes[ConnectionType.PUBLISHER] = ["/talker","/talker2"]
        nodes[ConnectionType.SUBSCRIBER] = ["/listener"]
        nodes[ConnectionType.SERVICE] = ["/add_two_ints_server"]
        nodes[ConnectionType.ACTION_SERVER] = ["/averaging_server"]
        nodes[ConnectionType.ACTION_CLIENT] = ["/fibonacci_client"]

        watchlist = [Rule(type, topics[type], '') for type in topics]
        expected_interface = [Rule(type, topics[type], node) for type in topics for node in nodes[type]]
        self.assertAdvertiseCall(watchlist, expected_interface)

    def test_advertiseAll(self):

        topics = {}
        topics[ConnectionType.PUBLISHER] = "/chatter"
        topics[ConnectionType.SUBSCRIBER] = "/chatter"
        topics[ConnectionType.SERVICE] = "/add_two_ints"
        topics[ConnectionType.ACTION_SERVER] = "/averaging_server/"
        topics[ConnectionType.ACTION_CLIENT] = "/fibonacci/"
        nodes = {}
        nodes[ConnectionType.PUBLISHER] = ["/talker","/talker2"]
        nodes[ConnectionType.SUBSCRIBER] = ["/listener"]
        nodes[ConnectionType.SERVICE] = ["/add_two_ints_server"]
        nodes[ConnectionType.ACTION_SERVER] = ["/averaging_server"]
        nodes[ConnectionType.ACTION_CLIENT] = ["/fibonacci_client"]

        # no blacklist
        blacklist = []
        expected_interface = [Rule(type, topics[type], node) for type in topics for node in nodes[type]]
        expected_interface.append(Rule(ConnectionType.SUBSCRIBER, "/random_number", "/averaging_server"))
        self.assertAdvertiseAllCall(blacklist, expected_interface)

        # 2 items in blacklist
        blacklist = []
        blacklist.append(Rule(ConnectionType.PUBLISHER, "/chatter", ""))
        blacklist.append(Rule(ConnectionType.SUBSCRIBER, "/random_number", "/averaging_server"))
        expected_interface = [Rule(type, topics[type], node) for type in topics for node in nodes[type]]
        expected_interface[:] = [e for e in expected_interface if e.type != ConnectionType.PUBLISHER]
        self.assertAdvertiseAllCall(blacklist, expected_interface)

        # test regex in blacklist
        blacklist = []
        blacklist.append(Rule(ConnectionType.SUBSCRIBER, "/random.*", ""))
        expected_interface = [Rule(type, topics[type], node) for type in topics for node in nodes[type]]
        self.assertAdvertiseAllCall(blacklist, expected_interface)

        blacklist = []
        blacklist.append(Rule(ConnectionType.SUBSCRIBER, "/random2.*", "")) 
        expected_interface = [Rule(type, topics[type], node) for type in topics for node in nodes[type]]
        expected_interface.append(Rule(ConnectionType.SUBSCRIBER, "/random_number", "/averaging_server"))
        self.assertAdvertiseAllCall(blacklist, expected_interface)

        blacklist = []
        blacklist.append(Rule(ConnectionType.SUBSCRIBER, "/random.*", ".*raging.*"))
        expected_interface = [Rule(type, topics[type], node) for type in topics for node in nodes[type]]
        self.assertAdvertiseAllCall(blacklist, expected_interface)

    def tearDown(self):
        '''
          Called at the end of every test to ensure that all the advertisements
          are removed
        '''
        req = AdvertiseAllRequest()
        req.cancel = True
        self.advertiseAll(req)
        self.assertPublicInterface([])

if __name__ == '__main__':
    rospy.init_node('multimaster_test')
    rostest.rosrun(PKG, 'test_advertisements_locally', TestAdvertisementsLocally) 
