#!/usr/bin/env python
#       
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_multimaster/master/rocon_gateway_hub/LICENSE 
#

import roslib; roslib.load_manifest('rocon_gateway_hub')
import rospy

###############################################################################
# Functions
###############################################################################

def load():
    '''
    Returns the gateway parameters from the ros param server.
    Most of these should be fairly self explanatory.
    '''
    param = {}
    
    param['name'] = rospy.get_param('~name','Gateway Hub')
    param['port'] = rospy.get_param('~port','6380')
    param['zeroconf'] = rospy.get_param("~zeroconf",True)
    
    return param

