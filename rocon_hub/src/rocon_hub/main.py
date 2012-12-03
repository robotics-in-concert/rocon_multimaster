#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_multimaster/master/multimaster_server/rocon_hub/LICENSE
#

import sys

# Ros imports
import roslib
roslib.load_manifest('rocon_hub')
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

    # Installation checks - sys exits if the process if not installed.
    utils.check_if_executable_available('redis-server')
    if param['zeroconf']:
        utils.check_if_executable_available('avahi-daemon')

    redi = redis_server.RedisServer(param)
    redi.start()  # sys exits if server connection is unavailable

    if param['zeroconf']:
        zeroconf.advertise_port_to_avahi(param['port'], param['name'])  # sys exits if running avahi-daemon not found

    rospy.spin()
    redi.shutdown()
