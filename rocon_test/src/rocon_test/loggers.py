#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_multimaster/license/LICENSE
#

##############################################################################
# Imports
##############################################################################

from __future__ import print_function

import os
import sys
import roslaunch.core
import logging
import socket
import rosgraph
import rospkg
import rosunit
from rostest.rostestutil import rostest_name_from_path


##############################################################################
# Methods
##############################################################################

def xml_results_file(package, results_log_name):
    '''
      Generate the absolute results log file name. This will be located in the
      cmake build directory, prefixed with 'rocon_test' and using the specially
      generated underscore separated log name.
    '''
    results_log_file = rosunit.xml_results_file(package, results_log_name, is_rostest=True)
    # rostest's results_file is prefixed with 'rostest' - switch it to our 'rocon_test'
    return results_log_file.replace('rostest', 'rocon_test', 1)

def configure_logging(package, filename):
    '''
      Configures the logger and generates an underscored pkg_dir relative name
      (e.g. launch/pirate_chatter.multilaunch -> launch_pirate_chatter
    '''
    roslaunch.core.add_printlog_handler(logging.getLogger('rocon_test').info)
    roslaunch.core.add_printerrlog_handler(logging.getLogger('rocon_test').error)
    r = rospkg.RosPack()
    pkg_dir = r.get_path(package)
    results_log_name = rostest_name_from_path(pkg_dir, filename)
    log_basename = 'rocon_test-%s-%s.log' % (socket.gethostname(), os.getpid())
    unused_log_name = rosgraph.roslogging.configure_logging('rocon_test', filename=log_basename)
    results_log_file = xml_results_file(package, results_log_name)
    return results_log_name, results_log_file


def printlog(msg, *args):
    if args:
        msg = msg % args
    logging.getLogger('rocon_test').info(msg)
    print("[ROCON_TEST] " + msg)


def printlogerr(msg, *args):
    if args:
        msg = msg % args
    logging.getLogger('rocon_test').error(msg)
    print >> sys.stderr, "[ROCON_TEST] " + msg

##############################################################################
# Test
##############################################################################

if __name__ == '__main__':
    printlog(" Where is it dude?")
