#!/usr/bin/env python
#       
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_multimaster/master/multimaster_server/rocon_gateway_hub/LICENSE 
#
##############################################################################
# Imports
##############################################################################

import sys
import re
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
# Functions
##############################################################################

def parse_system_configuration():
    '''
      Pushes the system redis server configuration file into a variable.
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

def initialise(port, hub_name):
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
        rospy.loginfo("Hub : reset hub variables on the redis server.")
    except redis.exceptions.ConnectionError:
        sys.exit(utils.logfatal("Hub : could not connect to the redis server - is it running?"))

def clear(port):
    '''
      Clears rocon: keys on the server.
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
        pipe.execute()
        rospy.logdebug("Hub : clearing hub variables on the redis server.")
    except redis.exceptions.ConnectionError:
        sys.exit(utils.logfatal("Hub : could not connect to the redis server - is it running?"))

def instantiate_template(template, port, pid_file, max_memory):
    '''
      Variable subsitution in a template file.
      
      This inserts the labelled variables into the template wherever the corresponding
      %(port) etc are found.
      
      @param port : port on which the server will run
      @type int
      @param pid_file : pathname to where the pid file will be stored
      @type string
      @param max_memory: how much memory to allocate to the redis server in bytes
      @type string (e.g. 10mb) 
    '''
    return template%locals()

import os
import rospkg

if __name__ == "__main__":

    ros_home = rospkg.get_ros_home()  
    hub_name = 'Gateway Hub'.lower().replace(" ","_")
    port = 6380
    pid_file = os.path.join(ros_home,'redis',hub_name+'.pid')
    max_memory = '10mb'
    template = utils.read_template('../../redis/redis.conf.template')
    print("Ros Home: %s"%ros_home)
    print("Hub Id  : %s"%hub_name)
    print("Port    : %s"%port)
    print("Pid File: %s"%pid_file)
    template = instantiate_template(template, port, pid_file, max_memory)
    print template
