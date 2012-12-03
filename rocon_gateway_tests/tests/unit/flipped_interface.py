#!/usr/bin/env python
#       
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_multimaster/master/rocon_gateway_tests/LICENSE 
#

import roslib
roslib.load_manifest('rocon_gateway_tests')
import rospy
from rocon_gateway import FlippedInterface, createEmptyConnectionTypeDictionary, Connection
from gateway_msgs.msg import Rule, ConnectionType, RemoteRule
import argparse

"""
  Tests various functions in the flipped interface class.
  
  Usage:
    > rosrun rocon_gateway_tests flipped_interface
"""

def print_flips(new_flips, old_flips):
    print ""
    print "NEW FLIPS:"
    print ""
    print new_flips
    print ""
    print "OLD FLIPS:"
    print ""
    print old_flips
    print ""

if __name__ == '__main__':

    parser = argparse.ArgumentParser(description='Tests the update method in the flipped interface.')
#  parser.add_argument("gateway", help="gateway string identifier", type=str)
#  parser.add_argument('-c','--clients',metavar='<Client name>',type=str,nargs='+',help='Client\'s unique name on hub')
#  parser.add_argument('-m','--message',metavar='<Topic triple>',type=str,nargs='+',help='<Topic triple>="<topic name>,<topic type>,<node uri>"')
#  args = parser.parse_args()

    rospy.init_node('flipped_interface')
    flipped_interface = FlippedInterface(default_rule_blacklist=[])

    ##############################
    # Initial Connections
    ##############################
    connection_rule = Rule()
    connection_rule.type=ConnectionType.PUBLISHER
    connection_rule.name="/foo"
    connection_rule.node="/bar"
    system_connections = createEmptyConnectionTypeDictionary()
    connection = Connection(connection_rule,"std_msgs/String","http://localhost:5321")
    system_connections[ConnectionType.PUBLISHER].append(connection)
    
    flip_rule = RemoteRule()
    flip_rule.gateway='gateway1'
    flip_rule.rule.name='/chatter'
    flip_rule.rule.type=ConnectionType.PUBLISHER
    
    rospy.loginfo("************* Update after a new flip rule *************")
    rospy.loginfo("Expected Result: the system connection is not yet present")
    rospy.loginfo("so should return empty results (no matches) from the update.")
    flipped_interface.addRule(flip_rule)
    new_flips, old_flips = flipped_interface.update(system_connections)
    # should return empty dictionaries
    print_flips(new_flips,old_flips)

    
    rospy.loginfo("************* Update after system connection acquired *************")
    rospy.loginfo("Expected Result: should see a match and a new item in new_flips.")

    connection_rule = Rule()
    connection_rule.type=ConnectionType.PUBLISHER
    connection_rule.name="/chatter"
    connection_rule.node="/talker"
    system_connections = createEmptyConnectionTypeDictionary()
    connection = Connection(connection_rule,"std_msgs/String","http://localhost:5321")
    system_connections[ConnectionType.PUBLISHER].append(connection)
    
    new_flips, old_flips = flipped_interface.update(system_connections)
    print_flips(new_flips,old_flips)
    
    rospy.loginfo("************* Update after flip rule removed acquired *************")
    rospy.loginfo("Expected Result: should see a match existing in old flips.")
    
    if not flipped_interface.removeRule(flip_rule):
        rospy.logerr("Failed to remove an existing rule from the flip rules.")
    
    new_flips, old_flips = flipped_interface.update(system_connections)
    print_flips(new_flips,old_flips)
    rospy.loginfo("************* Done *************")
    
