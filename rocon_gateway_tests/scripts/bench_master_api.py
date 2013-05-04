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
import rocon_utilities.console as console
import sys

##############################################################################
# Main
##############################################################################

if __name__ == '__main__':
    rospy.init_node('bench_master_api')
    master = rocon_gateway.LocalMaster()
    while not rospy.is_shutdown():
        start_time = rospy.get_time()  # float version
        connections = master.get_connection_state()
        finish_time = rospy.get_time() - start_time
        print("Benchmarks \n" + console.cyan + "  LocalMaster.get_connection_state: " + console.yellow + "%s" % finish_time + console.reset)
