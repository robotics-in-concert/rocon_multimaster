#!/usr/bin/env python

#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_multimaster/master/rocon_utilities/LICENSE
#

##############################################################################
# Imports
##############################################################################

import rospy
import time

##############################################################################
# Wall Rate (Uses system time, independent of ros /clock)
##############################################################################

class WallRate():

    def __init__(self, rate):
        '''
          @param rate : rate in hertz. If rate = 0, then won't sleep
          @type float
        '''
        self.rate = rate
        self.period = 1.0 / rate if rate > 0.0 else 0.0
        self.recorded_time = time.time() 

    def sleep(self):
        current_time = time.time()
        elapsed = current_time - self.recorded_time
        if self.period - elapsed > 0:
            rospy.rostime.wallsleep(self.period - elapsed)
        self.recorded_time = time.time()
