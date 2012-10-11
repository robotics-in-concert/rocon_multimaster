#!/usr/bin/env python
#       
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_multimaster/master/multimaster_client/rocon_gateway_tests/LICENSE 
#

PKG = 'rocon_gateway_tests'
import roslib; roslib.load_manifest(PKG)
import rospy
import rostest
from gateway_comms.msg import *
from gateway_comms.srv import *
import unittest
import std_msgs
import random

class TestAdvertisementsLocally(unittest.TestCase):

    def setUp(self):
        '''
          Run at the start of every test. This function performs the following:
            1. Ensure that gateway services are available
            2. Make sure gateway is connected to the hub
          If something goes wrong and the setup can not complete successfully,
          the test will timeout with an error
        '''
        self.log_open()

        rospy.wait_for_service('/gateway/advertise')
        rospy.wait_for_service('/gateway/advertise_all')
        rospy.wait_for_service('/gateway/gateway_info')

        self.advertise = rospy.ServiceProxy('/gateway/advertise',Advertise)
        self.advertiseAll = rospy.ServiceProxy('/gateway/advertise_all',AdvertiseAll)
        self.gatewayInfo = rospy.ServiceProxy('/gateway/gateway_info',GatewayInfo)

        # Make sure we are connected to the gateway first!
        while True:
            resp = self.gatewayInfo()
            if resp.connected:
                break
            rospy.sleep(3.0)
            self.log("TEST : Waiting for gateway to be connected...")

        self.maxDiff = None

    def log_open(self):
        #self.log_file = open('/tmp/' + PKG + '_test_advertisements_locally_' + str(random.randint(1,100000)), 'w')
        pass

    def log(self,text):
        #self.log_file.write('[LOG] ' + text + '\n')
        pass

    def log_close(self):
        #self.log_file.close()
        pass

    def test_advertisePublisherByTopic(self):
        '''
          Tests advertising publishers. Also tests that multiple nodes are
          coming up as expected.
        '''
        # Request adding the /chatter topic, 
        # This should add 2 nodes /talker and /talker2
        req = AdvertiseRequest()
        rule = Rule()
        rule.type = Rule.PUBLISHER
        rule.name = "/chatter"
        req.rules.append(rule)
        req.cancel = False
        resp = self.advertise(req)
        
        self.assertEquals(resp.result, Result.SUCCESS)
        self.assertEquals(len(resp.watchlist), 1)
        self.assertEquals(resp.watchlist[0], rule)

        # Now wait for the 2 nodes to be added. This may take some time as the 
        # nodes may not be up yet. If something has gone wrong, then the test 
        # will timeout with a failure
        num_nodes = 2
        while True:
            resp = self.gatewayInfo()
            if len(resp.public_interface) == num_nodes:
                break
            rospy.sleep(3.0)
            self.log("TEST : Waiting for watcher thread to load nodes.")
            node_names = []
            for i in resp.public_interface:
                node_names.append(i.name)
            self.log("TEST :   Current Nodes: %s"%str(node_names))

        actual_node_names = []
        for i in range(num_nodes):
            self.assertEquals(resp.public_interface[i].type,Rule.PUBLISHER)
            self.assertEquals(resp.public_interface[i].name,"/chatter")
            actual_node_names.append(resp.public_interface[i].node)
        expected_node_names = ["/talker","/talker2"]
        self.assertListEqual(sorted(actual_node_names), sorted(expected_node_names))

        #Now lets manually add a third publisher, and see if it gets added
        pub = rospy.Publisher("/chatter", std_msgs.msg.String)
        pub.publish("hello")

        # Wait for 3 nodes to be available with all the correct information
        num_nodes = 3
        while True:
            resp = self.gatewayInfo()
            if len(resp.public_interface) == num_nodes:
                break
            rospy.sleep(3.0)
            self.log("TEST : Waiting for watcher thread to load nodes.")
            node_names = []
            for i in resp.public_interface:
                node_names.append(i.name)
            self.log("TEST :   Current Nodes: %s"%str(node_names))

        actual_node_names = []
        for i in range(num_nodes):
            self.assertEquals(resp.public_interface[i].type,Rule.PUBLISHER)
            self.assertEquals(resp.public_interface[i].name,"/chatter")
            actual_node_names.append(resp.public_interface[i].node)
        expected_node_names = ["/talker","/talker2",rospy.get_name()]
        self.assertListEqual(sorted(actual_node_names), sorted(expected_node_names))

    def test_advertiseAll(self):
        '''
          Makes sure that every connection type is being detected and advertised
          appropriately.
        '''
        topics = {}
        topics[Rule.PUBLISHER] = "/chatter"
        topics[Rule.SUBSCRIBER] = "/chatter"
        topics[Rule.SERVICE] = "/add_two_ints"
        topics[Rule.ACTION_SERVER] = "/averaging_server/"
        topics[Rule.ACTION_CLIENT] = "/fibonacci/"
        nodes = {}
        nodes[Rule.PUBLISHER] = ["/talker","/talker2"]
        nodes[Rule.SUBSCRIBER] = ["/listener"]
        nodes[Rule.SERVICE] = ["/add_two_ints_server"]
        nodes[Rule.ACTION_SERVER] = ["/averaging_server"]
        nodes[Rule.ACTION_CLIENT] = ["/fibonacci_client"]
        num_nodes = 6

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
        #self.assertListEqual(sorted(resp.watchlist), sorted(req.rules))

        # Ensure all the nodes come as advertised. First wait for all the nodes
        while True:
            resp = self.gatewayInfo()
            if len(resp.public_interface) == num_nodes:
                break
            rospy.sleep(3.0)
            self.log("TEST : Waiting for watcher thread to load nodes.")
            node_names = []
            for i in resp.public_interface:
                node_names.append(i.name)
            self.log("TEST :   Current Nodes: %s"%str(node_names))

        # Now make sure all data is correct
        for i in range(num_nodes):
            type = resp.public_interface[i].type
            node = resp.public_interface[i].node
            name = resp.public_interface[i].name
            self.assertEquals(name, topics[type])
            self.assertIn(node, nodes[type])
            nodes[type].remove(node)

    def tearDown(self):
        '''
          Called at the end of every test to ensure that all the advertisements
          are removed
        '''
        self.log_close()
        req = AdvertiseAllRequest()
        req.cancel = True
        self.advertiseAll(req)

if __name__ == '__main__':
    rospy.init_node('multimaster_test')
    rostest.rosrun(PKG, 'test_advertisements_locally', TestAdvertisementsLocally) 
