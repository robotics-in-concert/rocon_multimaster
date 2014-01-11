#!/usr/bin/env python
#       
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_multimaster/master/rocon_gateway/LICENSE 
#

##############################################################################
# Imports
##############################################################################

import rospy
import unique_id
import rocon_service_pair_msgs.msg as rocon_service_pair_msgs 

##############################################################################
# Classes
##############################################################################

class Jagi(object):
    __slots__ = [
        'pub',
        'sub',
    ]

    def __init__(self):
        self.pub = rospy.Publisher('testies/response', rocon_service_pair_msgs.TestiesPairResponse)
        self.sub = rospy.Subscriber("testies/request", rocon_service_pair_msgs.TestiesPairRequest, self.callback)

    def callback(self, msg):
        rospy.loginfo("received '%s'" % msg.request.data)
        response = rocon_service_pair_msgs.TestiesResponse()
        response.data = "I heard ya dude" 
        pair_response = rocon_service_pair_msgs.TestiesPairResponse()
        pair_response.id = msg.id
        pair_response.response = response
        self.pub.publish(pair_response)

##############################################################################
# Main
##############################################################################

# To send a message to this subscriber from the command line:
#
# > rostopic pub /testies/request rocon_service_pair_msgs/TestiesRequest "id: 
#    uuid: "[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]"
#    data: 'hello'"
#    
#    

if __name__ == '__main__':
    
    rospy.init_node('example_rocon_pair_server')
    jagi = Jagi()
    rospy.spin()
