#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2012, Yujin Robot, Daniel Stonier , Jihoon Lee
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#  notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#  copyright notice, this list of conditions and the following
#  disclaimer in the documentation and/or other materials provided
#  with the distribution.
#  * Neither the name of Yujin Robot nor the names of its
#  contributors may be used to endorse or promote products derived
#  from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import os
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
    devnull = open(os.devnull,"w")
    #retval = subprocess.call(["dpkg","-s",package_name],stdout=devnull,stderr=subprocess.STDOUT)
    retval = subprocess.call(["which",name],stdout=devnull,stderr=subprocess.STDOUT)
    devnull.close()
    if retval != 0:
        sys.exit(utils.logerror("[ERROR] " + name + " not installed - hint 'rosdep install rocon_gateway_hub'\n"))

##############################################################################
# Initialize redis server 
##############################################################################

def initialize_redis_server(port, hub_name):
    # DJS Todo: check for success here
    pool = redis.ConnectionPool(host='localhost', port=port, db=0)
    server = redis.Redis(connection_pool=pool)

    pipe = server.pipeline()
    # DJS Todo: don't flush other programs use of the hub.
    pipe.flushall()
    pipe.set("rocon:index",0)
    pipe.set("rocon:hub_name",hub_name)
    pipe.execute()
    print "Clean up all database. set \"index\" 0"


##############################################################################
# avahi advertisement
##############################################################################

def advertise_port_to_avahi(config, hub_name):
    port = config["port"]
    # if you don't specify  stdout/stderr streams, then it will automatically go to the background
    # note, it will be also killed if the parent process itself is killed later on.
    retval = subprocess.Popen(["avahi-publish","-s",hub_name,"_ros-gateway-hub._tcp",str(port)])
    print("avahi-publish -s "+hub_name+" _ros-gateway-hub._tcp "+str(port))
    #retval = subprocess.call("avahi-publish -s '"+hub_name+"' _ros-gateway-hub._tcp "+str(port),shell=True)
    print("Return value: " + str(retval))
    rospy.loginfo("advertising '"+hub_name+"' on zeroconf [_ros-gateway-hub._tcp, port "+str(port)+"]")

##############################################################################
# Main
##############################################################################

def launch():
    if not utils.check_master():
        sys.exit(utils.red_string("Unable to communicate with master!"))

    rospy.init_node('hub')
    zeroconf_flag = rospy.get_param("~zeroconf",True)
    hub_name = rospy.get_param("~name","Gateway Hub")

    # These abort if not found 
    check_if_package_available('redis-server')
    if zeroconf_flag:
        check_if_package_available('avahi-daemon')

    # check if the daemons (redis and avahi) are running by testing
    # their connection and functionality explicitly

    # flush all the previous data. and set unique key for indexing clients
    config = parse_redis_configuration()
    initialize_redis_server(int(config["port"]), hub_name)

    # Try to autodetect the system and start redis appropriately
    # Try to autodetect the system and start zeroconf appropriately
    # Linux
    # TODO: If port is zero, find a free port here before advertising
    # Might need to track this one so we can kill it when the program 
    # terminates
    if zeroconf_flag:
        advertise_port_to_avahi(config, hub_name)
    print("SPIIIIIIIIIIIIIINNNINNNNNNNNNNNNNG")
    rospy.spin()
    # Need to kill avahipublish here
