#!/usr/bin/env python

""" Testing the service pair timeout functionality """

# enable some python3 compatibility options:
from __future__ import absolute_import, print_function, unicode_literals

import unittest
import rospy
import rocon_service_pair_msgs.msg as rocon_service_pair_msgs
import rocon_service_pairs
import rostest
import unique_id
import threading


def generate_request_message():
    request = rocon_service_pair_msgs.TestiesRequest()
    request.data = "hello dude"
    return request

class TestServicePairTimeouts(unittest.TestCase):

    def __init__(self, *args):
        super(TestServicePairTimeouts, self).__init__(*args)
        self.testies = rocon_service_pairs.ServicePairClient('testies_broken', rocon_service_pair_msgs.TestiesPair)
        self.response_message = "I heard ya dude"
        rospy.sleep(0.5)  # rospy hack to give publishers time to setup
        self._error_msg_id = None
        self._error_response = None
        self._error_event = None

    def test_non_blocking_call_with_timeout(self):
        self._error_event = threading.Event()
        msg_id = self.testies(generate_request_message(), timeout=rospy.Duration(1.0), callback=self.callback, error_callback=self.error_callback)
        result = self._error_event.wait(2.0)
        self.assertTrue(result, "Did not receive an error message")

    def test_wait_for_service_pair_server(self):
        wait_failed = False
        with self.assertRaises(rospy.ROSException):
            self.testies.wait_for_service_pair_server(1.0)

    def callback(self, msg_id, msg):
        """ User callback to feed into non-blocking requests.

          @param msg_id : id of the request-response pair.
          @type uuid_msgs.UniqueID
          
          @param msg : message response received
          @type <name>Response
        """
        pass

    def error_callback(self, msg_id, error_message):
        """ User callback to feed into non-blocking requests.

          @param msg_id : id of the request-response pair.
          @type uuid_msgs.UniqueID
          
          @param error_message : error string received
          @type str
        """
        #rospy.loginfo("Error Callback: %s" % error_message)
        self._error_event.set()

if __name__ == '__main__':
    rospy.init_node("test_service_pair_timeouts")
    rostest.rosrun('rocon_service_pairs',
                   'test_service_pair_timeouts',
                   TestServicePairTimeouts) 