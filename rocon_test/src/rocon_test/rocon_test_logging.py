#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_multimaster/hydro-devel/rocon_test/LICENSE
#

##############################################################################
# Imports
##############################################################################

from __future__ import print_function

import roslaunch.core
import logging
import rospkg
from rostest.rostestutil import rostest_name_from_path


##############################################################################
# Methods
##############################################################################

#class Logger():
#    '''
#      Logging functionalities we hide under the hood.
#    '''
def generate_log_name(package, filename):
    logger = logging.getLogger('rocon_test')
    roslaunch.core.add_printlog_handler(logger.info)
    roslaunch.core.add_printerrlog_handler(logger.error)
    r = rospkg.RosPack()
    pkg_dir = r.get_path(package)
    log_name = rostest_name_from_path(pkg_dir, filename)  # underscored pkg_dir relative name (e.g. launch/pirate_chatter.multilaunch -> launch_pirate_chatter
    return (logger, log_name)
