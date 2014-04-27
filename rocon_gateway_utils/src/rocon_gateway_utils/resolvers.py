#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_multimaster/license/LICENSE
#
##############################################################################
# Imports
##############################################################################

import re
import time
import rospy
import gateway_msgs.msg as gateway_msgs
import rocon_python_comms
import rosservice

##########################################################################
# Gateway Existence
##########################################################################


def resolve_local_gateway(timeout=None):
    '''
      @param timeout : timeout on checking for the gateway, if None, it just makes a single attempt.
      @type rospy.rostime.Duration

      @raise rocon_python_comms.NotFoundException: if no remote gateways or no matching gateways available.
    '''
    #master = rosgraph.Master(rospy.get_name())
    gateway_namespace = None
    service_names = []
    timeout_time = time.time() + timeout.to_sec() if timeout is not None else None
    while not rospy.is_shutdown():
        if timeout_time is not None and time.time() > timeout_time:
            break
        service_names = rosservice.rosservice_find("gateway_msgs/RemoteGatewayInfo")
        if not service_names or len(service_names) > 1:
            gateway_namespace = None
        elif service_names[0] == '/remote_gateway_info':
            gateway_namespace = "/"
        else:
            gateway_namespace = re.sub(r'/remote_gateway_info', '', service_names[0])
        if gateway_namespace is not None or timeout is None:
            break
        else:
            rospy.rostime.wallsleep(0.1)
    if not gateway_namespace:
        if not service_names:
            raise rocon_python_comms.NotFoundException("no gateway found attached to this local master - did you start it?")
        else:
            raise rocon_python_comms.NotFoundException("found more than one gateway connected to this master, this is an invalid configuration.")
    #console.debug("Found a local gateway at %s"%gateway_namespace)
    return gateway_namespace


def resolve_gateway_info(gateway_namespace=None):
    '''
      @param the local topic namespace to prepend to the 'gateway_info' identifier. Uses
      resolve_local_gateway if none is specified.
      @type str

      @return the local gateway info in all its gory detail.
      @rtype gateway_msgs.GatewayInfo

      @raise rocon_gateway.GatewayError: if no remote gateways or no matching gateways available.
    '''
    if gateway_namespace == None:
        gateway_namespace = resolve_local_gateway()
    gateway_info = rocon_python_comms.SubscriberProxy(gateway_namespace + '/gateway_info', gateway_msgs.GatewayInfo)()
    return gateway_info
