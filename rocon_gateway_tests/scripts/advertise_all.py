#!/usr/bin/env python
#       
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_multimaster/hydro-devel/rocon_gateway_tests/LICENSE 
#
##############################################################################
# Imports
##############################################################################

import rospy
import rocon_gateway
import gateway_msgs.srv as gateway_srvs
import argparse
import sys

##############################################################################
# Main
##############################################################################

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Advertise all connections (unadvertise if using --cancel')
    parser.add_argument('--cancel', action='store_true', help='cancel the advertisements')
    args = parser.parse_args(rospy.myargv(sys.argv)[1:])
    if args.cancel:
        action_text = "cancelling"
    else:
        action_text = "advertising"

    rospy.init_node('advertise_all')
    advertise_all = rospy.ServiceProxy('/gateway/advertise_all', gateway_srvs.AdvertiseAll)
    req = gateway_srvs.AdvertiseAllRequest() 
    req.cancel = args.cancel
    req.blacklist = []

    rospy.loginfo("Advertise All : %s all."%action_text) 
    resp = advertise_all(req)
    if resp.result != 0:
        rospy.logerr("Advertise All : error occured (todo: no error message yet)")
        #rospy.logerr("Advertise : %s"%resp.error_message)

