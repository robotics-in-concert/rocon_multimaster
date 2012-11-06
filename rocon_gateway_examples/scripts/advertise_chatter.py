#!/usr/bin/env python
#       
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_multimaster/master/rocon_gateway_examples/LICENSE 
#
##############################################################################
# Imports
##############################################################################

import roslib; roslib.load_manifest('rocon_gateway_examples')
import rospy
import rocon_gateway
from gateway_comms.srv import Advertise,AdvertiseRequest
from gateway_comms.msg import ConnectionType, Rule

"""
    Advertise /chatter Publisher

    Usage:
        1> roslaunch --port=11311 rocon_gateway_hub pirate.launch           # starts hub

        2a> roslaunch --port=11312 rocon_gateway pirate_tutorials.launch    # launches gateway on master_uri 11312 and starts /chatter publisher  
        2b> rosrun rocon_gateway_examples advertise_chatter.py __master=http://localhost:11312 # advertise /chatter

        2c> rosservice call /gateway/gateway_info       # check if chatter is advertised properly

        3a> roslaunch --port=11313 rocon_gateway pirate.launch  # launches gateway on master_uri 11313.
        3b> rosservice call /gateway/remote_gateway_info []     # check if /chatter is visible
"""
        
if __name__ == '__main__':

    rospy.init_node('advertise_chatter') 

    #  Setup gateway service
    advertise_srv = rospy.ServiceProxy('/gateway/advertise',Advertise)
   
    # Create Service Request Object
    req = AdvertiseRequest()
    req.cancel = False          # if true, it unadvertise the exisiting topic

    rule = Rule()
    rule.name = '/chatter'               # Topic name
    rule.type = ConnectionType.PUBLISHER # Advertise publisher
    rule.node = ''                       # If specified, it advertise only a particular node's publisher
    req.rules = []
    req.rules.append(rule)
    
    # Call service
    rospy.loginfo("Advertise : %s [%s,%s,%s]."%('advertise',rule.type, rule.name, rule.node or 'None')) 
    resp = advertise_srv(req)

    # Print response
    rospy.loginfo(str(resp))


