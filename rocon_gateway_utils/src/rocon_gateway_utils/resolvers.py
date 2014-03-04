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
import rosgraph
import rospy
import gateway_msgs.msg as gateway_msgs
import rocon_python_comms

##########################################################################
# Gateway Existence
##########################################################################


def resolve_local_gateway(timeout=rospy.rostime.Duration(1.0)):
    '''
      @param timeout : timeout on checking for the gateway.
      @type rospy.rostime.Duration

      @raise rocon_python_comms.NotFoundException: if no remote gateways or no matching gateways available.
    '''
    master = rosgraph.Master(rospy.get_name())
    gateway_namespace = None
    timeout_time = time.time() + timeout.to_sec()
    while not rospy.is_shutdown() and time.time() < timeout_time:
        unused_publishers, unused_subscribers, services = master.getSystemState()
        for service in services:
            service_name = service[0]  # second part is the node name
            if re.search(r'remote_gateway_info', service_name):
                if service_name == '/remote_gateway_info':
                    gateway_namespace = "/"
                    break
                else:
                    gateway_namespace = re.sub(r'/remote_gateway_info', '', service_name)
                    break
        if gateway_namespace is not None:
            break
        else:
            rospy.rostime.wallsleep(0.1)
    if not gateway_namespace:
        raise rocon_python_comms.NotFoundException("could not find a local gateway - did you start it?")
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
