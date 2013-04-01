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


class SubscriberProxy():
    '''
      Works like a service proxy, but using a subscriber instead.
    '''
    def __init__(self, topic, msg_type):
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

    def __call__(self, timeout=None):
        '''
          Returns immediately with the latest data or waits for
          incoming data.
        '''
        r = rospy.Rate(10)
        start_time = rospy.get_time()
        while not rospy.is_shutdown() and self._data == None:
            r.sleep()
            if timeout:
                if rospy.get_time() - start_time > timeout:
                    break
        return self._data

    def wait_for_next(self, timeout=None):
        '''
          Makes sure any current data is cleared and waits for new data.
        '''
        self._data = None
        return self.__call__(timeout)

    def _callback(self, data):
        self._data = data
