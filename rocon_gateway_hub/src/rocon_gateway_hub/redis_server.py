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
import shutil
try:
    import redis
except ImportError:
    sys.exit("\n[ERROR] No python-redis found - 'rosdep install rocon_gateway_hub'\n")

# Ros imports
import roslib
roslib.load_manifest('rocon_gateway_hub')
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
    f = open(filename, 'r')
    settings = {}

    for line in f:
        kv = line.split()
        if len(kv) > 1:
            settings[kv[0]] = kv[1]
    return settings


def initialise(port, hub_name):
    '''
      Connect, delete all rocon:xxx variables and reinitialise with
      specified values.

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
            pipe.delete(*keys_to_delete)  # * unpacks the list args - http://stackoverflow.com/questions/2921847/python-once-and-for-all-what-does-the-star-operator-mean-in-python
        pipe.set("rocon:hub:index", 0)
        pipe.set("rocon:hub:name", hub_name)
        pipe.execute()
        rospy.loginfo("Hub : reset hub variables on the redis server.")
    except redis.exceptions.ConnectionError:
        sys.exit(utils.logfatal("Hub : could not connect to the redis server - is it running?"))


def start_server(parameters):
    home_dir = os.path.join(rospkg.get_ros_home(), 'redis', parameters['hub_name'].lower().replace(" ", "_"))
    if os.path.isdir(home_dir):
        shutil.rmtree(home_dir)
    os.makedirs(home_dir)
    rospack = rospkg.RosPack()
    redis_conf_file = os.path.join(rospack.get_path('rocon_gateway_hub'), 'redis', 'redis.conf')
    redis_local_file = os.path.join(rospack.get_path('rocon_gateway_hub'), 'redis', 'redis.conf.local')
    target_redis_conf_file = os.path.join(home_dir, 'redis.conf')
    target_redis_local_file = os.path.join(home_dir, 'redis.conf.local')
    redis_conf_template = utils.read_template(redis_conf_file)
    redis_conf_template = instantiate_redis_conf_template(redis_conf_template, target_redis_local_file)
    redis_local_template = utils.read_template(redis_local_file)
    redis_local_template = instantiate_local_conf_template(redis_local_template, parameters['port'], parameters['max_memory'])
    try:
        f = open(target_redis_conf_file, 'w')
        f.write(redis_conf_template.encode('utf-8'))
    finally:
        f.close()
    try:
        f = open(target_redis_local_file, 'w')
        f.write(redis_local_template.encode('utf-8'))
    finally:
        f.close()


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
            pipe.delete(*keys_to_delete)  # * unpacks the list args - http://stackoverflow.com/questions/2921847/python-once-and-for-all-what-does-the-star-operator-mean-in-python
        pipe.execute()
        rospy.logdebug("Hub : clearing hub variables on the redis server.")
    except redis.exceptions.ConnectionError:
        sys.exit(utils.logfatal("Hub : could not connect to the redis server - is it running?"))


def instantiate_redis_conf_template(template, local_conf_filename):
    '''
      Variable substitution in a template file.

      @param local_conf_filename : where to find the local redis configuration file
      @type string
    '''
    return template % locals()


def instantiate_local_conf_template(template, port, max_memory):
    '''
      Variable substitution in a template file.

      @param port : port on which the server will run
      @type int
      @param pid_file : pathname to where the pid file will be stored
      @type string
      @param max_memory: how much memory to allocate to the redis server in bytes
      @type string (e.g. 10mb)
    '''
    return template % locals()

import os
import rospkg

if __name__ == "__main__":

    parameters = {}
    parameters['hub_name'] = 'Pirate Hub'
    parameters['port'] = 6380
    parameters['max_memory'] = '10mb'
    start_server(parameters)
