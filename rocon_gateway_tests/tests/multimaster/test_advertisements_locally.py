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

class TestAdvertisementsLocally(unittest.TestCase):

    def setUp(self):
        #wait for the test suite to become available
        all_nodes_up = False
        master = rosgraph.Master(rospy.get_name())
        while not all_nodes_up:
            try:
                master.lookupNode("/talker")
                master.lookupNode("/talker2")
                master.lookupNode("/listener")
                master.lookupNode("/add_two_ints_server")
                master.lookupNode("/fibonacci_client")
                master.lookupNode("/averaging_server")
                all_nodes_up = True
            except:
                rospy.loginfo("Setup : Still waiting for nodes to come up")
                rospy.sleep(1.0)
                if rospy.is_shutdown():
                    self.fail("Unable to find all test nodes in suite")

        rospy.wait_for_service('/gateway/advertise')
        rospy.wait_for_service('/gateway/advertise_all')

        self.advertise = rospy.ServiceProxy('/gateway/advertise',Advertise)
        self.advertiseAll = rospy.ServiceProxy('/gateway/advertise_all',AdvertiseAll)

        while True:
            req = AdvertiseAllRequest()
            req.cancel = True
            resp = self.advertiseAll(req)
            if resp.result == Result.SUCCESS:
                break
            rospy.sleep(5.0)

    def test_AdvertisePublisherByTopic(self):
        req = AdvertiseRequest()
        rule = PublicRule()
        rule.connection.type = Connection.PUBLISHER
        rule.connection.name = "/chatter"
        req.rules.append(rule)
        req.cancel = False
        resp = self.advertise(req)
        
        self.assertEquals(resp.result, Result.SUCCESS, "Advertise call not successful");
        self.assertEquals(len(resp.watchlist), 1, "Unexpected public interface watchlist length"); # only one rule was added
        self.assertEquals(resp.watchlist[0], rule, "Added rule different from requested"); # the rule we requested was added
        self.assertEquals(len(resp.public_interface), 2, "Number of nodes added should be 2, but found to be %s"%str(len(resp.public_interface))); # both nodes are available

        self.assertEquals(resp.public_interface[0].connection.type,Connection.PUBLISHER, "added node has incorrect type [%s], expected publisher"%resp.public_interface[0].connection.type);
        self.assertEquals(resp.public_interface[1].connection.type,Connection.PUBLISHER, "added node has incorrect type [%s], expected publisher"%resp.public_interface[0].connection.type);
        self.assertEquals(resp.public_interface[0].connection.name,"/chatter", "expected topic name chatter, found %s"%resp.public_interface[0].connection.name);
        self.assertEquals(resp.public_interface[1].connection.name,"/chatter", "expected topic name chatter, found %s"%resp.public_interface[0].connection.name);
        self.assertItemsEqual([resp.public_interface[0].connection.node,resp.public_interface[1].connection.node],["/talker","/talker2"], "node names do not match, expected /talker,/talker2, found [%s,%s]"%(resp.public_interface[0].connection.node,resp.public_interface[1].connection.node)) 

    def tearDown(self):
        req = AdvertiseAllRequest()
        req.cancel = True
        self.advertiseAll(req)

if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'test_advertisements_locally', TestAdvertisementsLocally) 
