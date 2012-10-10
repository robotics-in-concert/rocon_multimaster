#!/usr/bin/env python
#       
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_multimaster/master/multimaster_client/rocon_gateway_tests/LICENSE 
#

import roslib; roslib.load_manifest('rocon_gateway_tests')
import rospy
from rocon_gateway import FlippedInterface, createEmptyConnectionTypeDictionary
from gateway_comms.msg import FlipRule, Connection
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
    flipped_interface = FlippedInterface()

    ##############################
    # Initial Connections
    ##############################
    connections = createEmptyConnectionTypeDictionary()
    connection = Connection()
    connection.type=Connection.PUBLISHER
    connection.name="/foo"
    connection.node="/bar"
    connections[Connection.PUBLISHER].append(connection)
    
    flip_rule = FlipRule()
    flip_rule.gateway='gateway1'
    flip_rule.connection.name='/chatter'
    flip_rule.connection.type=Connection.PUBLISHER
    
    rospy.loginfo("************* Update Tests *************")

    rospy.loginfo("Testing update for a new flip rule")
    flipped_interface.addRule(flip_rule)
    new_flips, old_flips = flipped_interface.update(connections)
    # should return empty dictionaries
    
    connection = Connection()
    connection.type=Connection.PUBLISHER
    connection.name="/chatter"
    connection.node="/talker"

    connections[Connection.PUBLISHER].append(connection)
    new_flips, old_flips = flipped_interface.update(connections)
    print_flips(new_flips,old_flips)
    
    if not flipped_interface.removeRule(flip_rule):
        rospy.logerr("Failed to remove an existing rule from the flip rules.")
    
    new_flips, old_flips = flipped_interface.update(connections)
    print_flips(new_flips,old_flips)
    
    import re
    try:
        print re.match("dude","fat").group()
    except AttributeError as e:
        print "dude"