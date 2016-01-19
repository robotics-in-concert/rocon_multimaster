#!/usr/bin/env python
from __future__ import absolute_import

# ROS SETUP if needed

import multiprocessing
import threading
import time
import cProfile



try:
    import rospy
    import rosgraph
    import roslaunch
    import rocon_gateway
    import pyros_setup
except ImportError as exc:
    import os
    import pyros_setup
    import sys
    sys.modules["pyros_setup"] = pyros_setup.delayed_import_auto(base_path=os.path.join(os.path.dirname(__file__), '..', '..', '..', '..', '..'))
    import rospy
    import rosgraph
    import roslaunch
    import rocon_gateway

roscore_process = None
master = None

if not rosgraph.masterapi.is_online():
    master, roscore_process = pyros_setup.get_master()
    assert master.is_online()

# Start roslaunch
launch = roslaunch.scriptapi.ROSLaunch()
launch.start()

os.environ["ROS_NAMESPACE"] = "/rocon"
rospy.init_node('gateway', )
rospy.set_param("~hub_uri", "http://localhost:6380")
rospy.set_param("~disable_zeroconf", "true")

gw_node = rocon_gateway.GatewayNode()

def wait_just_a_bit():
    count = 255
    start = time.time()
    while count > 0:
        # time is ticking
        now = time.time()
        timedelta = now - start
        start = now

        time.sleep(0.2)

        count -= 1

    rospy.signal_shutdown('timer expired')

timer = threading.Thread(target= wait_just_a_bit())
timer.start()

cProfile.run('gw_node.spin()')

rospy.signal_shutdown('test complete')

if roscore_process is not None:
    roscore_process.terminate()  # make sure everything is stopped