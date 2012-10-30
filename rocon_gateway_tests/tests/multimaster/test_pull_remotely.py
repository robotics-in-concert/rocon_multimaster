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
from rocon_gateway.master_api import LocalMaster
import unittest
import std_msgs
import copy

class TestPullRemotely(unittest.TestCase):

    def assertRemotePublicInterface(self, gateway_name, rules):

        expected_rules = copy.deepcopy(rules)
        num_nodes = len(expected_rules)
        while True:
            rospy.sleep(1.0)
            resp = self.remoteGatewayInfo()
            remote_gateway = None
            for gateway in resp.gateways:
                if gateway.name == gateway_name:
                    remote_gateway = gateway
                    break
            self.assertIsNotNone(remote_gateway)
            node_names = []
            for i in remote_gateway.public_interface:
                node_names.append(i.node)
            rospy.loginfo("TEST: Public Interface Nodes (Gateway: %s): %s"%(gateway_name, str(node_names)))
            if len(remote_gateway.public_interface) == num_nodes:
                break
            rospy.loginfo("TEST:   Waiting for watcher thread to load all nodes.")

        for i in range(num_nodes):
            self.assertIn(remote_gateway.public_interface[i], expected_rules)
            expected_rules.remove(remote_gateway.public_interface[i])

        self.assertEqual(len(expected_rules), 0);
        rospy.loginfo("TEST: Public interface for %s found as expected"%gateway_name)

    def assertMasterState(self, expected_interface):
        while True:
            rospy.sleep(1.0)
            connection_state = self.master.getConnectionState()
            all_rules_found = True
            rules_not_found = []
            for rule in expected_interface:
                rule_type = rule[0]
                rule_name = rule[1]
                rule_count = rule[2]
                num_rules_found = 0
                for connection in connection_state[rule_type]:
                    if connection.rule.name == rule_name:
                        num_rules_found = num_rules_found + 1
                if num_rules_found != rule_count:
                    rules_not_found.append(rule.name)
                    all_rules_found = False
            if all_rules_found:
                break
            rospy.log_info("TEST: Following rules found with incorrect number of instances: %s"%str(rules_not_found))

        rospy.loginfo("TEST: Master state found as expected")

    def assertPullCall(self, watchlist, expected_interface):

        zero_interface = []
        for i in expected_interface:
            zero_interface.append((i[0],i[1],0))

        self.assertMasterState(zero_interface)

        for rule in watchlist:
            req = RemoteRequest()
            req.remote = rule
            req.cancel = False

            resp = self.pull(req)
            self.assertEquals(resp.result, Result.SUCCESS)

        self.assertMasterState(expected_interface)
            
        for rule in watchlist:
            req = RemoteRequest()
            req.remote = rule
            req.cancel = True

            resp = self.pull(req)
            self.assertEquals(resp.result, Result.SUCCESS)

        self.assertMasterState(zero_interface)

    def setUp(self):
        '''
          Run at the start of every test. This function performs the following:
            1. Ensure that gateway services are available
            2. Make sure gateway is connected to the hub
          If something goes wrong and the setup can not complete successfully,
          the test will timeout with an error
        '''
        rospy.wait_for_service('/gateway/pull')
        rospy.wait_for_service('/gateway/pull_all')
        rospy.wait_for_service('/gateway/gateway_info')
        rospy.wait_for_service('/gateway/remote_gateway_info')

        self.pull = rospy.ServiceProxy('/gateway/pull', Remote)
        self.pullAll = rospy.ServiceProxy('/gateway/pull_all', RemoteAll)
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
                break

        rospy.loginfo("TEST: Remote gateway connected")

        self.master = LocalMaster()
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
        expected_interface.append(Rule(ConnectionType.ACTION_CLIENT, "/fibonacci/", "/fibonacci_client"))
        self.assertRemotePublicInterface(self.remote_gateway_name, expected_interface)

    def test_pullPublisherByTopic(self):
        watchlist = [RemoteRule(self.remote_gateway_name, Rule(ConnectionType.PUBLISHER, "/chatter", ''))]
        expected_interface = [(ConnectionType.PUBLISHER, "/chatter", 2)]
        self.assertPullCall(watchlist, expected_interface)

        # Test topic name using regex
        watchlist = [RemoteRule(self.remote_gateway_name, Rule(ConnectionType.PUBLISHER, "/chat.*", ''))]
        self.assertPullCall(watchlist, expected_interface)

    def test_pullPublisherByNode(self):
        watchlist = [RemoteRule(self.remote_gateway_name, Rule(ConnectionType.PUBLISHER, "/chatter", "/talker"))]
        expected_interface = [(ConnectionType.PUBLISHER, "/chatter", 1)]
        self.assertPullCall(watchlist, expected_interface)

        # Test using regex
        watchlist = [RemoteRule(self.remote_gateway_name, Rule(ConnectionType.PUBLISHER, "/chat.*", ".*ker"))]
        self.assertPullCall(watchlist, expected_interface)

    def tearDown(self):
        '''
          Called at the end of every test to ensure that all the advertisements
          are removed
        '''
        pass

if __name__ == '__main__':
    rospy.init_node('multimaster_test')
    rostest.rosrun(PKG, 'test_pull_remotely', TestPullRemotely) 
