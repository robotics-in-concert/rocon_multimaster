#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_multimaster/hydro_devel/rocon_gateway/LICENSE
#
##############################################################################
# Imports
##############################################################################

import rospy
import gateway_msgs.srv as gateway_srvs

# local imports
from .exceptions import GatewaySampleRuntimeError

##############################################################################
# Constants
##############################################################################

gateway_namespace = '/gateway'

##############################################################################
# Methods
##############################################################################
#
# Notes about these methods:
#  Ros is already running (i.e. rospy.init_node has been called
#  Use the gateway namespace above (could probably make it smarter by hunting)
#
##############################################################################


def advertise_all(cancel=False, ns=gateway_namespace):
    '''
      Sends a rule for advertising everything except the default blacklist.
    '''
    advertise_all = rospy.ServiceProxy('/gateway/advertise_all', gateway_srvs.AdvertiseAll)
    req = gateway_srvs.AdvertiseAllRequest()
    req.cancel = cancel
    req.blacklist = []
    if cancel:
        action_text = "cancelling"
    else:
        action_text = "advertising"
    rospy.loginfo("Advertise All : %s all." % action_text)
    resp = advertise_all(req)
    if resp.result != 0:
        raise GatewaySampleRuntimeError("failed to advertise all (todo: no error message yet)")
