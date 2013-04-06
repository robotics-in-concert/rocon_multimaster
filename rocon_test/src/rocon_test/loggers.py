#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_multimaster/hydro-devel/rocon_test/LICENSE
#

##############################################################################
# Imports
##############################################################################

from __future__ import print_function

import sys
import roslaunch.core
import logging
import rospkg
from rostest.rostestutil import rostest_name_from_path


##############################################################################
# Variables
##############################################################################

logger = logging.getLogger('rocon_test')

##############################################################################
# Methods
##############################################################################


def generate_log_name(package, filename):
    '''
      Generates an underscored pkg_dir relative name
      (e.g. launch/pirate_chatter.multilaunch -> launch_pirate_chatter
    '''
    roslaunch.core.add_printlog_handler(logger.info)
    roslaunch.core.add_printerrlog_handler(logger.error)
    r = rospkg.RosPack()
    pkg_dir = r.get_path(package)
    log_name = rostest_name_from_path(pkg_dir, filename)
    return log_name


def printlog(msg, *args):
    if args:
        msg = msg % args
    logger.info(msg)
    print("[ROCON_TEST] " + msg)


def printlogerr(msg, *args):
    if args:
        msg = msg % args
    logger.error(msg)
    print >> sys.stderr, "[ROCON_TEST] " + msg

##############################################################################
# Test
##############################################################################

if __name__ == '__main__':
    printlog(" Where is it dude?")
