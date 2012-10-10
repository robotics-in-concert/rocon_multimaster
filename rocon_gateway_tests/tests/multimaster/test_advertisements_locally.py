#!/usr/bin/env python
#       
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_multimaster/master/multimaster_client/rocon_gateway_tests/LICENSE 
#

PKG = 'rocon_gateway_tests'
import roslib; roslib.load_manifest(PKG)
import rospy
import rosgraph
from gateway_comms.msg import *
from gateway_comms.srv import *
import unittest
import std_msgs

class TestAdvertisementsLocally(unittest.TestCase):

    def setUp(self):

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

    def test_AdvertisePublisherByTopic(self):
        
        rospy.init_node('multimaster_test')

        # Request adding the /chatter topic, this should add 2 nodes /talker and /talker2
        req = AdvertiseRequest()
        rule = PublicRule()
        rule.connection.type = Connection.PUBLISHER
        rule.connection.name = "/chatter"
        req.rules.append(rule)
        req.cancel = False
        resp = self.advertise(req)
        
        self.assertEquals(resp.result, Result.SUCCESS)
        self.assertEquals(len(resp.watchlist), 1)
        self.assertEquals(resp.watchlist[0], rule)

        # Now wait for the 2 nodes to be added. This may take some time as the nodes may not be up yet
        # If something has gone wrong, then the test will timeout with a failure
        num_nodes = 2
        while True:
            resp = self.gatewayInfo()
            if len(resp.public_interface) == num_nodes:
                break
            rospy.sleep(3.0)

        actual_node_names = []
        for i in range(num_nodes):
            self.assertEquals(resp.public_interface[i].connection.type,Connection.PUBLISHER)
            self.assertEquals(resp.public_interface[i].connection.name,"/chatter")
            actual_node_names.append(resp.public_interface[i].connection.node)
        expected_node_names = ["/talker","/talker2"]
        self.assertItemsEqual(actual_node_names, expected_node_names)

        #Now lets manually add a third publisher, and see if it gets added
        pub = rospy.Publisher("/chatter", std_msgs.msg.String)
        pub.publish("hello")

        # Wait for 3 nodes to be available
        num_nodes = 3
        while True:
            resp = self.gatewayInfo()
            if len(resp.public_interface) == 3:
                break
            rospy.sleep(3.0)

        actual_node_names = []
        for i in range(num_nodes):
            self.assertEquals(resp.public_interface[i].connection.type,Connection.PUBLISHER)
            self.assertEquals(resp.public_interface[i].connection.name,"/chatter")
            actual_node_names.append(resp.public_interface[i].connection.node)
        expected_node_names = ["/talker","/talker2",rospy.get_name()]
        self.assertItemsEqual(actual_node_names, expected_node_names)

    def tearDown(self):
        req = AdvertiseAllRequest()
        req.cancel = True
        self.advertiseAll(req)

if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'test_advertisements_locally', TestAdvertisementsLocally) 
