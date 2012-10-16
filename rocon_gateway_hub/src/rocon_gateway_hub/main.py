#!/usr/bin/env python
#       
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_multimaster/master/multimaster_server/rocon_gateway_hub/LICENSE 
#

import sys

# Ros imports
import roslib; roslib.load_manifest('rocon_gateway_hub')
import rospy

# Local imports
from . import utils
from . import redis_server
from . import ros_parameters
from . import zeroconf

##############################################################################
# Main
##############################################################################

def main():
    # Ros
    while not utils.check_master():
        rospy.logerr("Unable to communicate with master!")
        rospy.sleep(1.0)
        if rospy.is_shutdown():
            sys.exit(utils.red_string("Unable to communicate with master!"))
    rospy.init_node('hub')
    param = ros_parameters.load()

    # Redis
    utils.check_if_executable_available('redis-server')  # aborts if redis-server not installed
    config = redis_server.parse_system_configuration()
    redis_server.initialise(int(config["port"]), param['name'])

    # Zeroconf
    if param['zeroconf']:
        utils.check_if_executable_available('avahi-daemon') # aborts if avahi-daemon not installed
        zeroconf.advertise_port_to_avahi(config, param['name']) # aborts if avahi-daemon not running
        
    rospy.spin()
    redis_server.clear(int(config["port"]))
    