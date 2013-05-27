#!/usr/bin/env python
#       
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_multimaster/master/rocon_gateway/LICENSE 
#

##############################################################################
# Imports
##############################################################################

import rospy
import sys
import rocon_gateway

class Flags(object):
    advertise = 'advertise'
    cancel = 'cancel'

##############################################################################
# Functions
##############################################################################
    

##############################################################################
# Main
##############################################################################

if __name__ == '__main__':
    
    rospy.init_node('master_connections')
    master = rocon_gateway.LocalMaster()
    publishers, subscribers, services = master.get_system_state()
    
    for p in publishers:
      print(str(p))

