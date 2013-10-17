#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def talker():
    pub = rospy.Publisher('chatter', String, latch=True)
    rospy.init_node('talker')
    while not rospy.is_shutdown():
        str = "dude"
        rospy.loginfo(str)
        pub.publish(String(str))
        rospy.rostime.wallsleep(1.0)

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
