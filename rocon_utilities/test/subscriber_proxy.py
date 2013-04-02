#!/usr/bin/env python

import sys
import unittest
import rospy
import rocon_utilities
from std_msgs.msg import String
import rosunit

class TestSubscriberProxy(unittest.TestCase):

    def setUp(self):
        rospy.init_node('listener', anonymous=True)
        pass

    def test_receive_data(self):
        talker_data = rocon_utilities.SubscriberProxy('chatter', String)()
        self.assertEquals("dude", talker_data.data)

    def tearDown(self):
        pass

NAME = 'test_subscriber_proxy'
if __name__ == '__main__':
    rosunit.unitrun('test_subscriber_proxy', NAME, TestSubscriberProxy, sys.argv, coverage_packages=['rocon_utilities'])
