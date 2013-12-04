#!/usr/bin/env python

import rospy

if __name__ == '__main__':
    rospy.init_node('args_test')
    dude = rospy.get_param('~dude')
    rospy.loginfo("Dude is %s" % dude)
    rospy.spin()