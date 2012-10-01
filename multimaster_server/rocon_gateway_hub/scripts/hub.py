#!/usr/bin/env python
#       
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_multimaster/master/multimaster_server/rocon_gateway_hub/LICENSE 
# Copyright (c) 2012, Yujin Robot, Daniel Stonier
#

import roslib; roslib.load_manifest('rocon_gateway_hub')
import rospy
import rocon_gateway_hub

if __name__ == "__main__":
    rocon_gateway_hub.launch()
