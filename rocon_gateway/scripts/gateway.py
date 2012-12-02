#!/usr/bin/env python
#       
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_multimaster/master/rocon_gateway/LICENSE 
#

##############################################################################
# Imports
##############################################################################

import roslib; roslib.load_manifest('rocon_gateway')
import rospy
from rocon_gateway import Gateway

##############################################################################
# Launch point
##############################################################################

if __name__ == '__main__':
    
    rospy.init_node('gateway')
    gateway = Gateway()
    gateway.spin()
    rospy.loginfo("Gateway : shutting down.")
