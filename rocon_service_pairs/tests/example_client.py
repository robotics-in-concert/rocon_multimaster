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
import rocon_service_pairs

##############################################################################
# Classes
##############################################################################

class Jagi(object):
    __slots__ = [
        'testies',
    ]

    def __init__(self):
        self.testies = rocon_service_pairs.ServicePairClient('testies', rocon_service_pair_msgs.TestiesPair)
        request = rocon_service_pair_msgs.TestiesRequest()
        request.data = "hello dude"
        rospy.sleep(0.5)  # rospy hack to give publishers time to setup
        response = self.testies(request, timeout=rospy.Duration(10.0))
        if response is not None:
            rospy.loginfo("Test: %s" % response.data)
        else:
            rospy.loginfo("Test return 'None'")

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
    
    rospy.init_node('example_rocon_service_pair_client')
    jagi = Jagi()
#     publisher = rospy.Publisher('testies/request', rocon_service_pair_msgs.TestiesPairRequest)
#     testies = rocon_service_pair_msgs.TestiesPairRequest()
#     testies.request = rocon_service_pair_msgs.TestiesRequest()
#     testies.request.data = "hello dude"
#     rospy.sleep(1.0)
#     publisher.publish(testies)
    rospy.spin()
