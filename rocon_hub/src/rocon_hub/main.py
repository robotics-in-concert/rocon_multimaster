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

##############################################################################
# Shutdown Handlers
##############################################################################
#
# This lets the hub have a controlled shutdown from an external party
# (in our special case of interest, from the conductor).


def ros_service_shutdown(unused_request):
    shutdown()
    return std_srvs.EmptyResponse()


def shutdown():
    global redi
    if redi is not None:
        rospy.loginfo("Hub : shutting down.")
        redi.shutdown()
        redi = None


def wait_for_shutdown():
    '''
      Shutdown hook - we wait here for an external shutdown via ros service
      (at which point redi is None)
      timing out after a reasonable time if we need to.
    '''
    global redi
    global timeout
    count = 0.0
    while count < timeout:
        if redi is None:
            return
        else:
            count += 0.5
            rospy.rostime.wallsleep(0.5)  # human time
    rospy.logwarn("Hub : timed out waiting for external shutdown by ros service, forcing shutdown now.")
    shutdown()


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
    if param['external_shutdown']:
        timeout = param['external_shutdown_timeout']
        rospy.on_shutdown(wait_for_shutdown)
        unused_shutdown_service = rospy.Service('~shutdown', std_srvs.Empty, ros_service_shutdown)

    redi = redis_server.RedisServer(param)
    redi.start()  # sys exits if server connection is unavailable or incorrect version

    if param['zeroconf']:
        zeroconf.advertise_port_to_avahi(param['port'], param['name'])  # sys exits if running avahi-daemon not found

    watcher_thread = watcher.WatcherThread('localhost', param['port'])
    watcher_thread.start()
    rospy.spin()
    if not param['external_shutdown']:
        # do it here, don't wait for the ros service to get triggered
        shutdown()
