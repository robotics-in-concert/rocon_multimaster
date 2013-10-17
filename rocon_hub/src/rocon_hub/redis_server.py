#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_multimaster/master/multimaster_server/rocon_hub/LICENSE
#
##############################################################################
# Imports
##############################################################################

import os
import sys
import re
import shutil
import subprocess

# Ros imports
import rospy
import rospkg
try:
    import redis
except ImportError:
    # actually unused right now while we use redis as a ros package
    sys.exit("\n[ERROR] No python-redis found - 'rosdep install rocon_hub'\n")

# Local imports
import utils

##############################################################################
# Redis Server
##############################################################################


class RedisServer:
    def __init__(self, parameters):
        self._parameters = parameters
        self._process = None
        self._home_dir = os.path.join(rospkg.get_ros_home(), 'redis', self._parameters['name'].lower().replace(" ", "_"))
        self._files = {}
        self._files['redis_conf'] = os.path.join(self._home_dir, 'redis.conf')
        self._files['redis_conf_local'] = os.path.join(self._home_dir, 'redis.conf.local')
        self._files['redis_server_log'] = os.path.join(self._home_dir, 'redis-server.log')
        self._server = None

        self._setup()

    def _setup(self):
        '''
          Clear and configure redis conf, log files in the ros home
          directories under a subdirectory styled by the name of this hub.
        '''
        if os.path.isdir(self._home_dir):
            shutil.rmtree(self._home_dir)
        os.makedirs(self._home_dir)
        rospack = rospkg.RosPack()
        redis_conf_template = utils.read_template(os.path.join(rospack.get_path('rocon_hub'), 'redis', 'redis.conf'))
        redis_conf_template = instantiate_redis_conf_template(redis_conf_template, self._files['redis_conf_local'])
        redis_local_template = utils.read_template(os.path.join(rospack.get_path('rocon_hub'), 'redis', 'redis.conf.local'))
        redis_local_template = instantiate_local_conf_template(redis_local_template,
                                                               self._parameters['port'],
                                                               self._parameters['max_memory'],
                                                               self._files['redis_server_log'])
        try:
            f = open(self._files['redis_conf'], 'w')
            f.write(redis_conf_template.encode('utf-8'))
        finally:
            f.close()
        try:
            f = open(self._files['redis_conf_local'], 'w')
            f.write(redis_local_template.encode('utf-8'))
        finally:
            f.close()

    def start(self):
        '''
          Start the server. Also connect, delete all rocon:xxx
          variables and reinitialise with specified values.

          Aborts the program if the connection fails.
        '''
        self._process = subprocess.Popen(["redis-server", self._files['redis_conf']])
        pool = redis.ConnectionPool(host='localhost', port=int(self._parameters['port']), db=0)
        no_attempts = 5
        count = 0
        while count < no_attempts:
            try:
                self._server = redis.Redis(connection_pool=pool)
                rocon_keys = self._server.keys("rocon:*")
                pattern = re.compile("rocon:*")
                keys_to_delete = []
                for key in rocon_keys:
                    if pattern.match(key):
                        keys_to_delete.append(key)
                pipe = self._server.pipeline()
                if len(keys_to_delete) != 0:
                    pipe.delete(*keys_to_delete)  # * unpacks the list args - http://stackoverflow.com/questions/2921847/python-once-and-for-all-what-does-the-star-operator-mean-in-python
                pipe.set("rocon:hub:name", self._parameters['name'])
                pipe.execute()
                rospy.loginfo("Hub : reset hub variables on the redis server.")
                break
            except redis.ConnectionError:
                count += 1
                if count == no_attempts:
                    self.shutdown()
                    sys.exit(utils.logfatal("Hub : could not connect to the redis server - is it running?"))
                else:
                    rospy.rostime.wallsleep(0.1)

    def shutdown(self):
        '''
          Clears rocon: keys on the server.
        '''
        try:
            rocon_keys = self._server.keys("rocon:*")
            pattern = re.compile("rocon:*")
            keys_to_delete = []
            for key in rocon_keys:
                if pattern.match(key):
                    keys_to_delete.append(key)
            pipe = self._server.pipeline()
            if len(keys_to_delete) != 0:
                pipe.delete(*keys_to_delete)  # * unpacks the list args - http://stackoverflow.com/questions/2921847/python-once-and-for-all-what-does-the-star-operator-mean-in-python
            pipe.execute()
            rospy.logdebug("Hub : clearing hub variables on the redis server.")
        except redis.ConnectionError:
            pass
        self._process.terminate()

##############################################################################
# Functions
##############################################################################


def instantiate_redis_conf_template(template, local_conf_filename):
    '''
      Variable substitution in a template file.

      @param local_conf_filename : where to find the local redis configuration file
      @type string
    '''
    return template % locals()


def instantiate_local_conf_template(template, port, max_memory, logfile):
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

if __name__ == "__main__":
    pool = redis.ConnectionPool(host='localhost', port='6380', db=0)
    try:
        print "dude"
    except redis.exceptions.ConnectionError:
        print "err"
