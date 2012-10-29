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

    def assertRemotePublicInterface(self, gateway_name, rules):

        expected_rules = copy.deepcopy(rules)
        num_nodes = len(expected_rules)
        while True:
            rospy.sleep(1.0)
            resp = self.remoteGatewayInfo()
            remote_gateway = None
            for gateway in resp.gateways:
                if gateway == gateway_name:
                    remote_gateway = gateway
                    break
            self.assertIsNotNone(remote_gateway)
            for i in remote_gateway.public_interface:
                node_names.append(i.node)
            rospy.loginfo("TEST: Public Interface Nodes (Gateway: %s): %s"%(gateway_name, str(node_names)))
            if len(remote_gatway.public_interface) == num_nodes:
                break
            rospy.loginfo("TEST:   Waiting for watcher thread to load all nodes.")

        for i in range(num_nodes):
            self.assertIn(remote_gatway.public_interface[i], expected_rules)
            expected_rules.remove(remote_gateway.public_interface[i])

        self.assertEqual(len(expected_rules), 0);
        rospy.loginfo("TEST: Public interface for %s found as expected"%gateway_name)

    def setUp(self):
        '''
          Run at the start of every test. This function performs the following:
            1. Ensure that gateway services are available
            2. Make sure gateway is connected to the hub
          If something goes wrong and the setup can not complete successfully,
          the test will timeout with an error
        '''
        rospy.wait_for_service('/gateway/gateway_info')
        rospy.wait_for_service('/gateway/remote_gateway_info')

        self.gatewayInfo = rospy.ServiceProxy('/gateway/gateway_info', GatewayInfo)
        self.remoteGatewayInfo = rospy.ServiceProxy('/gateway/remote_gateway_info', RemoteGatewayInfo)

        # Make sure we are connected to the gateway first!
        while True:
            rospy.sleep(1.0)
            resp = self.gatewayInfo()
            if resp.connected:
                break
            rospy.loginfo("TEST: Waiting for local gateway to be connected...")

        rospy.loginfo("TEST: Local gateway connected")

        # Wait for remote gateway to come up
        req = RemoteGatewayInfoRequest()
        req.gateways = []
        while True:
            rospy.sleep(1.0)
            resp = self.remoteGatewayInfo(req)
            self.assertLessEqual(len(resp.gateways), 1)
            if len(resp.gateways) == 0:
                rospy.loginfo("TEST: Waiting for remote gateway to come up...")
            else:
                self.remote_gateway_name = resp.gateways[0].name

        rospy.loginfo("TEST: Remote gateway connected")
        # unit test property - show the difference when an assertion fails
        self.maxDiff = None

    def test_checkRemotePublicInterface(self):
        '''
          Makes sure that every connection type is being detected and advertised
          appropriately.
        '''
        expected_interface = []
        expected_interface.append(Rule(ConnectionType.PUBLISHER, "/chatter", "/talker"))
        expected_interface.append(Rule(ConnectionType.PUBLISHER, "/chatter", "/talker2"))
        expected_interface.append(Rule(ConnectionType.SUBSCRIBER, "/chatter", "/listener"))
        expected_interface.append(Rule(ConnectionType.SUBSCRIBER, "/random_number", "/averaging_server"))
        expected_interface.append(Rule(ConnectionType.SERVICE, "/add_two_ints", "/add_two_ints_server"))
        expected_interface.append(Rule(ConnectionType.ACTION_SERVER, "/averaging_server/", "/averaging_server"))
        expected_interface.append(Rule(ConnectionType.ACTION_CLIENT, "/fibonacci_client/", "/fibonacci"))
        self.assertRemotePublicInterface(self.remote_gateway_name, expected_interface)

    def tearDown(self):
        '''
          Called at the end of every test to ensure that all the advertisements
          are removed
        '''
        pass

if __name__ == '__main__':
    rospy.init_node('multimaster_test')
    rostest.rosrun(PKG, 'test_advertisements_locally', TestAdvertisementsLocally) 
