#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_multimaster/license/LICENSE
#
##############################################################################
# Imports
##############################################################################

import threading
import rospy
import unique_id

# Local imports

          @param callback : callback function invoked when a non-blocking call returns with a reply.
          @type method with argument of type given by msg_type
##############################################################################
# Role Manager
##############################################################################


class ServicePairProxy(object):
    '''
      Manages connectivity information provided by services and provides this
      for human interactive agent (aka remocon) connections.
    '''
    __slots__ = [
            '_publisher',
            '_subscriber',
            '_callback',  # external callback invoked upon receiving a reply
            '_request_events'  # list of request id's to associate with returning replies [uuid_msgs/UniqueId].
        ]

    ##########################################################################
    # Initialisation
    ##########################################################################

    def __init__(self, name, service_pair_type):
        '''
          @param name : the service pair name
          @type str
          @param service_pair_type : the .pair message type (shortened name without the Request/Response suffix)
          @type str
          @param timeout : timeout on the wait operation (None = /infty)
          @type rospy.Duration()
          @return msg type data or None
        '''
        self._subscriber = rospy.Subscriber(name + "/request", service_pair_type + "Request", self._internal_callback)
        self._publisher = rospy.Publisher(name + "/response", service_pair_type + "Response")
        self._callback = callback
        self._request_events = {}  # [uuid_msgs/UniqueId]

    def __call__(self, msg, timeout=None, callback = None):
        '''
          Blocking call.

          @param msg : the request message
          @type 'service_pair_type'Request (as specified in __init__)

          @param timeout : time to wait for data
          @type rospy.Duration

          @return data
        '''
        id = unique_id.toHexString(msg.id)
        self._request_events[id] = threading.Event()  # uuid_msgs/UniqueId
        self._publisher.publish(msg)
        if timeout is None:
            self._request_events[id].wait()
        else:
            self._request_events[id].wait(timeout.to_sec())
        del self._request_events[id]

    def _internal_callback(self, msg):
        # Check if it is a blocking call that has requested it.
        if msg.id in self._request_events.keys():
            self._request_events[msg.id].set()
            return
