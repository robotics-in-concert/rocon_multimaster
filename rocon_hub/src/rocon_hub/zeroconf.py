#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_multimaster/license/LICENSE
#
##############################################################################
# Imports
##############################################################################

import subprocess
import sys

# Delete this once we upgrade (hopefully anything after precise)
# Refer to https://github.com/robotics-in-concert/rocon_multimaster/issues/248
import threading
threading._DummyThread._Thread__stop = lambda x: 42

# Ros imports
import rospy

# Local imports
from . import utils
##############################################################################
# Functions
##############################################################################


def advertise_port_to_avahi(port, hub_name):
    '''
      Check if avahi-daemon is around and publish the redis server ip:port.
    '''
    # Check - assuming ubuntu here, robustify later
    proc = subprocess.Popen(["pidof", "avahi-daemon"], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    if proc.stdout.read() == "":
        sys.exit(utils.logfatal("Hub : could not find the avahi-daemon - is it running?"))
        sys.exit(utils.logfatal("    : if your isp is misbehaving and avahi not autostarting"))
        sys.exit(utils.logfatal("    : you may need to set AVAHI_DAEMON_DETECT_LOCAL=0"))
        sys.exit(utils.logfatal("    : in /etc/default/avahi-daemon"))

    # if you don't specify  stdout/stderr streams, then it will automatically go to the background
    # avahi-publish is a blocking call - it has to go to the background
    # also note, we don't worry about cleaning it up as it will be killed with the parent process
    subprocess.Popen(["avahi-publish", "-s", hub_name, "_ros-multimaster-hub._tcp", str(port)])
    rospy.loginfo("Hub : advertising '" + hub_name + "' on zeroconf [_ros-multimaster-hub._tcp, port " + str(port) + "]")
