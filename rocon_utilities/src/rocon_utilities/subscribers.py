#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_multimaster/master/rocon_utilities/LICENSE
#

##############################################################################
# Imports
##############################################################################

import rospy

##############################################################################
# Support Classes
##############################################################################

class SubscriberListener():
    '''
      Support class for wait_for_subscriber.
    '''
    def __init__(self, topic, msg_type, timeout):
        '''
          @param topic : the topic name to subscriber to
          @type str
          @param msg_type : any ros message type (typical arg for the subscriber)
          @type msg
          @param timeout : timeout on the wait operation (None = /infty)
          @type rospy.Duration()
          @return msg type data or None
        '''
        rospy.Subscriber(topic, msg_type, self._callback)
        self._data = None
        self._timeout = timeout

    def wait_for_data(self):
        r = rospy.Rate(10)
        start_time = rospy.get_time()
        while not rospy.is_shutdown() and self._data == None:
            r.sleep()
            if self._timeout:
                if rospy.get_time() - start_time > self._timeout:
                    break
        return self._data
        
    def _callback(self, data):
        self._data = data

##############################################################################
# Methods
##############################################################################

def wait_for_subscriber(topic, msg_type, timeout = None):
    '''
      Use this to effect a fake service. This is usually used with a latched
      subscriber to provide service updates with zero latency.
    '''
    listener = SubscriberListener(topic, msg_type, timeout)
    return listener.wait_for_data()
