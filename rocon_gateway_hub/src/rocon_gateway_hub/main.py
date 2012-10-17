#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_multimaster/master/multimaster_server/rocon_gateway_hub/LICENSE
#

import sys

# Ros imports
import roslib
roslib.load_manifest('rocon_gateway_hub')
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

    # Installation checks - both abort the process if not installed.
    utils.check_if_executable_available('redis-server')
    if param['zeroconf']:
        utils.check_if_executable_available('avahi-daemon')

    if param['zeroconf']:
        zeroconf.advertise_port_to_avahi(param['port'], param['name'])  # aborts if avahi-daemon not running

    redis = redis_server.RedisServer(param)
    redis.start()
    rospy.spin()
    redis.shutdown()
