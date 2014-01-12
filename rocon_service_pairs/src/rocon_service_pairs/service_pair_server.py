#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_multimaster/license/LICENSE
#
##############################################################################
# Imports
##############################################################################

#import threading
import rospy
import unique_id
#import copy
#import functools

# Local imports
from .exceptions import ServicePairException

##############################################################################
# Server Class
##############################################################################


class ServicePairServer(object):
    '''
      The server side of a pubsub service pair.
    '''
    __slots__ = [
            '_publisher',
            '_subscriber',
            '_callback',
            #'_request_handlers',  # initiate, track and execute requests with these { hex string ids : dic of RequestHandler objects (Blocking/NonBlocking) }
            'ServicePairSpec',
            'ServicePairRequest',
            'ServicePairResponse',
        ]

    ##########################################################################
    # Initialisation
    ##########################################################################

    def __init__(self, name, callback, ServicePairSpec):
        '''
          @param name : resource name of service pair (e.g. testies for pair topics testies/request, testies/response)
          @type str
          @param callback : function invoked when a request arrives
          @param ServicePairSpec : the pair type (e.g. rocon_service_pair_msgs.msg.TestiesPair)
          @type str
        '''
        self._callback = callback
        try:
            p = ServicePairSpec()
            self.ServicePairSpec = ServicePairSpec
            self.ServicePairRequest = type(p.pair_request)
            self.ServicePairResponse = type(p.pair_response)
        except AttributeError:
            raise ServicePairException("Type is not an pair spec: %s" % str(ServicePairSpec))
        self._subscriber = rospy.Subscriber(name + "/request", self.ServicePairRequest, self._internal_callback)
        self._publisher = rospy.Publisher(name + "/response", self.ServicePairResponse)

    ##########################################################################
    # Public Methods
    ##########################################################################

    def reply(self, request_id, msg):
        '''
          @param request_id : the request id to associate with this response.
          @type uuid_msgs.UniqueID

          @param msg : the response
          @type ServiceResponse
        '''
        pair_response = self.ServicePairResponse()
        pair_response.id = request_id
        pair_response.response = msg
        self._publisher.publish(pair_response)

    ##########################################################################
    # Callbacks
    ##########################################################################

    def _internal_callback(self, msg):
        '''
          @param msg : message returned from the server (with pair id etc)
          @type self.ServicePairRequest
        '''
        # Check if it is a blocking call that has requested it.
        self._callback(msg.id, msg.request)
