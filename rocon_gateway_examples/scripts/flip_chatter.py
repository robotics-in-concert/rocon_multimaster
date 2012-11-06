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
from gateway_comms.srv import Remote,RemoteRequest
from gateway_comms.msg import ConnectionType

"""
    Flipping chatter. It flips /chatter topic from master_uri 11312 to 11313

    Usage:
        1 > roslaunch --port=11311 rocon_gateway_hub priate.launch           // starts hub

        2a > roslaunch --port=11312 rocon_gateway pirate_tutorials.launch     // launches gateway on master_uri 11312 its gateway name is 'gateway' and starts /chatter publisher
        3a > roslaunch --port=11313 rocon_gateway priate.launch               // launches gateway on master_uri 11313 its gateway name is 'gateway2'

        2b > rosrun rocon_gateway_examples flip_chatter.py  __master:=http://localhost:11312  // flips /chatter topic in 11312
        3b > rostopic list                                                                    // check if the topic exists in 11313
"""

if __name__ == '__main__':
    
    rospy.init_node('flip_chatter')
    rospy.loginfo("Fliping /chatter topic")

    # Gateway flip service
    flip_service = rospy.ServiceProxy('/gateway/flip',Remote)

    # Gateway name where the topic is flip-in to it may differ
    remote_gateway_name = 'gateway1'

    # Create Request for flip service
    req = RemoteRequest()
    req.remote.gateway = remote_gateway_name 
    req.cancel = False          # if true, attempt to cancel existing /chatter
    req.remote.rule.name = "/chatter"
    req.remote.rule.type = ConnectionType.PUBLISHER     # Indicates that it is attempting to flip publisher
    req.remote.rule.node = '' # Optional. If specified, it flips a particular node's publisher. It is useful if there is a multiple publisher and want to flip just ones

    # Call service and receive the response
    rospy.loginfo("Flip : %s [%s,%s,%s,%s]."%('flipping', req.remote.gateway, req.remote.rule.type, req.remote.rule.name, req.remote.rule.node or 'None')) 
    resp = flip_service(req)

    # Print response
    rospy.loginfo(str(resp))

