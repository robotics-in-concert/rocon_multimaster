#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_multimaster/license/LICENSE
#

##############################################################################
# Imports
##############################################################################

import rospy

##############################################################################
# Classes
##############################################################################

#class SAService(object):


class SAServiceProxy(object):
    '''
      Works like a service proxy, but for a non-blocking (sync over async)
      publisher-subscriber pair that represent both ends of a typical
      request-response service.
    '''
    __slots__ = [
            '_publisher',
            '_subscriber',
        ]

    def __init__(self, name, msg_type):
        '''
          @param name : the service name to subscriber to
          @type str
          @param msg_type : any ros message type (typical arg for the subscriber)
          @type msg
          @param timeout : timeout on the wait operation (None = /infty)
          @type rospy.Duration()
          @return msg type data or None
        '''
        self._subscriber = rospy.Subscriber(name + "/request", msg_type, self._callback)
        self._publisher = rospy.Publisher(name + "/response", msg_type)

    def __call__(self, timeout=None):
        '''
          Returns immediately with the latest data or waits for
          incoming data.

          @param timeout : time to wait for data, polling at 10Hz.
          @type rospy.Duration
          @return latest data or None
        '''
        r = rospy.Rate(10)
        start_time = rospy.Time.now()
        while not rospy.is_shutdown() and self._data == None:
            r.sleep()
            if timeout:
                if rospy.Time.now() - start_time > timeout:
                    return None
        return self._data

    def wait_for_next(self, timeout=None):
        '''
          Makes sure any current data is cleared and waits for new data.
        '''
        self._data = None
        return self.__call__(timeout)

    def wait_for_publishers(self):
        '''
          Blocks until publishers are seen.

          @raise rospy.exceptions.ROSInterruptException if we are in shutdown.
        '''
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self._subscriber.get_num_connections() != 0:
                return
            else:
                r.sleep()
        # we are shutting down
        raise rospy.exceptions.ROSInterruptException

    def _callback(self, data):
        self._data = data

    def unregister(self):
        '''
          Unregister the subscriber so future instantiations of this class can pull a
          fresh subscriber (important if the data is latched).
        '''
        self._subscriber.unregister()
