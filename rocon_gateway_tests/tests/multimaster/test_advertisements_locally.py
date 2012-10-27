#!/usr/bin/env python
#       
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_multimaster/master/rocon_gateway_tests/LICENSE 
#

PKG = 'rocon_gateway_tests'
import roslib; roslib.load_manifest(PKG)
import rospy
import rostest
from gateway_comms.msg import *
from gateway_comms.srv import *
import unittest
import std_msgs
import copy

class TestAdvertisementsLocally(unittest.TestCase):

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
        # Request adding the /chatter topic, 
        # This should add 2 nodes /talker and /talker2
        req = AdvertiseRequest()
        rule = Rule()
        rule.type = ConnectionType.PUBLISHER
        rule.name = "/chatter"
        req.rules.append(rule)
        req.cancel = False

        resp = self.advertise(req)
        self.assertEquals(resp.result, Result.SUCCESS)
        self.assertEquals(len(resp.watchlist), 1)
        self.assertEquals(resp.watchlist[0], rule)

        # Ensure the two nodes in the test suite got added
        expected_interface = list()
        expected_interface.append(Rule(ConnectionType.PUBLISHER, "/chatter", "/talker"))
        expected_interface.append(Rule(ConnectionType.PUBLISHER, "/chatter", "/talker2"))
        self.assertPublicInterface(expected_interface)

        # Test removal
        req.cancel = True
        resp = self.advertise(req)
        self.assertEquals(resp.result, Result.SUCCESS)
        self.assertEquals(len(resp.watchlist), 0)
        self.assertPublicInterface([])

        # Test topic name using regex
        req.rules[0].name="/chat.*"
        req.cancel = False

        resp = self.advertise(req)
        self.assertEquals(resp.result, Result.SUCCESS)
        self.assertEquals(len(resp.watchlist), 1)
        self.assertEquals(resp.watchlist[0], rule)

        self.assertPublicInterface(expected_interface)

        # Test removal
        req.cancel = True
        resp = self.advertise(req)
        self.assertEquals(resp.result, Result.SUCCESS)
        self.assertEquals(len(resp.watchlist), 0)
        self.assertPublicInterface([])

    def test_advertisePublisherByNode(self):
        '''
          Tests advertising publishers. Also tests that multiple nodes are
          coming up as expected.
        '''
        req = AdvertiseRequest()
        rule = Rule()
        rule.type = ConnectionType.PUBLISHER
        rule.name = "/chatter"
        rule.node = "/talker"
        req.rules.append(rule)
        req.cancel = False

        resp = self.advertise(req)
        self.assertEquals(resp.result, Result.SUCCESS)
        self.assertEquals(len(resp.watchlist), 1)
        self.assertEquals(resp.watchlist[0], rule)

        # Ensure the two nodes in the test suite got added
        expected_interface = list()
        expected_interface.append(Rule(ConnectionType.PUBLISHER, "/chatter", "/talker"))
        self.assertPublicInterface(expected_interface)

        # Test removal
        req.cancel = True
        resp = self.advertise(req)
        self.assertEquals(resp.result, Result.SUCCESS)
        self.assertEquals(len(resp.watchlist), 0)
        self.assertPublicInterface([])

        # Test using regex
        req.rules[0].name="/chat.*"
        req.rules[0].node=".*ker"
        req.cancel = False

        resp = self.advertise(req)
        self.assertEquals(resp.result, Result.SUCCESS)
        self.assertEquals(len(resp.watchlist), 1)
        self.assertEquals(resp.watchlist[0], rule)

        self.assertPublicInterface(expected_interface)

        # Test removal
        req.cancel = True
        resp = self.advertise(req)
        self.assertEquals(resp.result, Result.SUCCESS)
        self.assertEquals(len(resp.watchlist), 0)
        self.assertPublicInterface([])
        

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

        expected_rules = [Rule(type, topics[type], node) for type in topics for node in nodes[type]]

        req = AdvertiseRequest()
        for type in topics:
            rule = Rule()
            rule.type = type
            rule.name = topics[type]
            req.rules.append(rule)
        req.cancel = False
        resp = self.advertise(req)

        self.assertEquals(resp.result, Result.SUCCESS)
        self.assertEquals(len(resp.watchlist), len(topics))
        for rule in resp.watchlist:
            self.assertIn(rule, resp.watchlist)

        # Ensure all the nodes come as advertised.
        self.assertPublicInterface(expected_rules)
        
        # Remove everything and assert null list
        req.cancel = True
        resp = self.advertise(req)
        self.assertEquals(resp.result, Result.SUCCESS)
        self.assertEquals(len(resp.watchlist), 0)
        self.assertPublicInterface([])

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
