#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_multimaster/master/rocon_gateway/LICENSE
#

###############################################################################
# Imports
###############################################################################

import roslib
roslib.load_manifest('rocon_gateway')
import rospy
import zeroconf_msgs.srv

###############################################################################
# Constants
###############################################################################

gateway_hub_service = "_ros-multimaster-hub._tcp"

###############################################################################
# Functions
###############################################################################

def resolve_address(msg):
    '''
      Resolves a zeroconf address into ip/port portions.
      @var msg : zeroconf_msgs.DiscoveredService 
      @return (string,int) : ip, port pair.
    '''
    ip = "localhost"
    if not msg.is_local:
        ip = msg.ipv4_addresses[0]
    return (ip,msg.port)
    
def setup_ros_services():
    '''
      Looks to see if it can find the zeroconf services that
      will help it auto-discover a hub. If it finds them, 
      it hooks up the required ros services with the zeroconf node.
      
      @return success of the hookup
      @rtype bool
    '''
    zeroconf_services = {}
    zeroconf_timeout = 5 # Amount of time to wait for the zeroconf services to appear
    rospy.loginfo("Gateway : looking to see if zeroconf services are available...")
    try:
        rospy.wait_for_service("zeroconf/add_listener", timeout=zeroconf_timeout)
        zeroconf_services["add_listener"] = rospy.ServiceProxy("zeroconf/add_listener",zeroconf_msgs.srv.AddListener)
        zeroconf_services["list_discovered_services"] = rospy.ServiceProxy("zeroconf/list_discovered_services",zeroconf_msgs.srv.ListDiscoveredServices)
        if not zeroconf_services["add_listener"](service_type = gateway_hub_service):
            zeroconf.services = {} # failure
    except rospy.ROSException:
        rospy.logwarn("Gateway : timed out waiting for zeroconf services to become available.")
    return zeroconf_services
