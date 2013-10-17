#!/usr/bin/env python

import sys
import unittest
import rospy
from std_msgs.msg import String
import rosunit

class TestListener(unittest.TestCase):

    def setUp(self):
        rospy.init_node('listener', anonymous=True)
        rospy.Subscriber("chatter", String, self.listener_callback)
        self.received_data = None

    def listener_callback(self, talker):
        rospy.loginfo("Listener: I heard %s" % talker.data)
        self.received_data = talker.data

    def test_receive_data(self):
        start_time = rospy.Time.now()
        timeout = rospy.Duration(5.0)
        timed_out = False
        while not rospy.is_shutdown() and not self.received_data and not (rospy.Time.now() - start_time > timeout):
            rospy.loginfo("Listener: waiting for data")
            rospy.rostime.wallsleep(0.2)
        self.assertEquals("dude", self.received_data)

    def tearDown(self):
        pass

NAME = 'test_listener'
if __name__ == '__main__':
    rosunit.unitrun('test_listener', NAME, TestListener, sys.argv, coverage_packages=['rocon_test'])
