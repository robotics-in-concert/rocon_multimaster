#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_multimaster/license/LICENSE
#
##############################################################################
# Imports
##############################################################################

import sys

# Ros imports
import rospy
import std_srvs.srv as std_srvs

# Local imports
from . import utils
from . import redis_server
from . import ros_parameters
from . import watcher
from . import zeroconf

##############################################################################
# Variables
##############################################################################

redi = None
timeout = 15


def shutdown():
    global redi
    if redi is not None:
        rospy.loginfo("Hub : shutting down.")
        redi.shutdown()
        redi = None


##############################################################################
# Main
##############################################################################


def main():
    global redi
    global timeout
    while not utils.check_master():
        rospy.logerr("Unable to communicate with master!")
        rospy.rostime.wallsleep(1.0)
        if rospy.is_shutdown():
            sys.exit(utils.red_string("Unable to communicate with master!"))
    rospy.init_node('hub')
    param = ros_parameters.load()

    # Installation checks - sys exits if the process if not installed.
    utils.check_if_executable_available('redis-server')
    if param['zeroconf']:
        utils.check_if_executable_available('avahi-daemon')

    redi = redis_server.RedisServer(param)
    redi.start()  # sys exits if server connection is unavailable or incorrect version

    if param['zeroconf']:
        zeroconf.advertise_port_to_avahi(param['port'], param['name'])  # sys exits if running avahi-daemon not found

    watcher_thread = watcher.WatcherThread('localhost', param['port'])
    watcher_thread.start()
    rospy.spin()
    shutdown()
