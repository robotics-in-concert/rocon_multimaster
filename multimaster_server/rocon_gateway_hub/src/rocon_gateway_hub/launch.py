#!/usr/bin/env python
#       
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_multimaster/master/multimaster_server/rocon_gateway_hub/LICENSE 
#

import sys
import subprocess

try:
    import redis
except ImportError:
    sys.exit("\n[ERROR] No python-redis found - hint 'rosdep install rocon_gateway_hub'\n")

# Ros imports
import roslib; roslib.load_manifest('rocon_gateway_hub')
import rospy

# Local imports
import utils

##############################################################################
# Config Parser
##############################################################################

def parse_redis_configuration():
    '''
      Pushes the redis server configuration file into a variable.
      Primarily used at the moment to find the redis server is running on.
      
      @filename : filename for the redis server configuration file.
    '''
    filename = '/etc/redis/redis.conf'
    f = open(filename,'r')
    settings = {}

    for line in f:
        kv = line.split()
        if len(kv) > 1:
            settings[kv[0]]= kv[1]
    return settings

##############################################################################
# Check Package availability
##############################################################################

def check_if_package_available(name):
    '''
      Ensure a particular executable is available on the system.
      
      Could use package names and python-apt here to find if the package is
      available, but more reliable and general - just check if program binary
      is available.
    '''
    if utils.which(name) is None:
        sys.exit(utils.logfatal("Hub : " + name + " not installed - hint 'rosdep install rocon_gateway_hub'."))

##############################################################################
# Initialize redis server 
##############################################################################
import re
def initialize_redis_server(port, hub_name):
    '''
      Connect, delete all rocon:xxx variables and reinitialise with specified values."
      
      Aborts the program if the connection fails.
    '''
    try:
        pool = redis.ConnectionPool(host='localhost', port=port, db=0)
        server = redis.Redis(connection_pool=pool)
        rocon_keys = server.keys("rocon:*")
        pattern = re.compile("rocon:*")
        keys_to_delete = []
        for key in rocon_keys:
            if pattern.match(key):
                keys_to_delete.append(key)
        pipe = server.pipeline()
        if len(keys_to_delete) != 0:
            pipe.delete(*keys_to_delete) # * unpacks the list args - http://stackoverflow.com/questions/2921847/python-once-and-for-all-what-does-the-star-operator-mean-in-python
        pipe.set("rocon:hub:index",0)
        pipe.set("rocon:hub:name",hub_name)
        pipe.execute()
    except redis.exceptions.ConnectionError:
        sys.exit(utils.logfatal("Hub : could not connect to the redis server - is it running?"))
    rospy.loginfo("Hub : reset all rocon:xxx variables on the redis server.")


##############################################################################
# avahi advertisement
##############################################################################

def advertise_port_to_avahi(config, hub_name):
    '''
      Check if avahi-daemon is around and publish the redis server ip:port.
    '''
    # Check - assuming ubuntu here, robustify later
    proc = subprocess.Popen(["pidof","avahi-daemon"],stdout=subprocess.PIPE,stderr=subprocess.PIPE)
    if proc.stdout.read() == "":
        sys.exit(utils.logfatal("Hub : could not find the avahi-daemon - is it running?"))
        sys.exit(utils.logfatal("    : if your isp is misbehaving and avahi not autostarting"))
        sys.exit(utils.logfatal("    : you may need to set AVAHI_DAEMON_DETECT_LOCAL=0"))
        sys.exit(utils.logfatal("    : in /etc/default/avahi-daemon"))

    port = config["port"]
    # if you don't specify  stdout/stderr streams, then it will automatically go to the background
    # avahi-publish is a blocking call - it has to go to the background
    # also note, we don't worrry about cleaning it up as it will be killed with the parent process
    subprocess.Popen(["avahi-publish","-s",hub_name,"_ros-gateway-hub._tcp",str(port)])
    rospy.loginfo("Hub : advertising '"+hub_name+"' on zeroconf [_ros-gateway-hub._tcp, port "+str(port)+"]")

##############################################################################
# Main
##############################################################################

def launch():
    if not utils.check_master():
        sys.exit(utils.red_string("Unable to communicate with master!"))

    # Parameters
    rospy.init_node('hub')
    zeroconf_flag = rospy.get_param("~zeroconf",True)
    hub_name = rospy.get_param("~name","Gateway Hub")

    # Redis
    check_if_package_available('redis-server')  # aborts if redis-server not installed
    config = parse_redis_configuration()
    initialize_redis_server(int(config["port"]), hub_name)

    # Zeroconf
    if zeroconf_flag:
        check_if_package_available('avahi-daemon') # aborts if avahi-daemon not installed
        advertise_port_to_avahi(config, hub_name) # aborts if avahi-daemon not running
    rospy.spin()
