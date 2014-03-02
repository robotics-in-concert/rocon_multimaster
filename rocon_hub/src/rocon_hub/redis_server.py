#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_multimaster/license/LICENSE
#
##############################################################################
# Imports
##############################################################################

import os
import sys
import re
import shutil
import subprocess
import signal

# Delete this once we upgrade (hopefully anything after precise)
# Refer to https://github.com/robotics-in-concert/rocon_multimaster/issues/248
import threading
threading._DummyThread._Thread__stop = lambda x: 42

import rospy
import rospkg
try:
    import rocon_python_redis as redis
except ImportError:
    # actually unused right now while we use redis as a ros package
    sys.exit("\n[ERROR] No python-redis found - 'rosdep install rocon_hub'\n")
import rocon_semantic_version as semantic_version

from . import utils

##############################################################################
# Redis Server
##############################################################################


class RedisServer:
    def __init__(self, parameters):
        self._parameters = parameters
        self._process = None
        self._home_dir = os.path.join(rospkg.get_ros_home(), 'redis', self._parameters['name'].lower().replace(" ", "_"))
        # clean out old redis information
        if os.path.isdir(self._home_dir):
            shutil.rmtree(self._home_dir)
        self._files = {}
        self._version_extension = self._introspect_redis_server_version()
        self._files['redis_conf'] = os.path.join(self._home_dir, 'redis-%s.conf' % self._version_extension)
        self._files['redis_conf_local'] = os.path.join(self._home_dir, 'redis-%s.conf.local' % self._version_extension)
        self._files['redis_server_log'] = os.path.join(self._home_dir, 'redis-server.log')
        self._server = None
        self._setup()

    def _introspect_redis_server_version(self):
        '''
          Sniff the version in major.minor format for decision making elsewhere (patch we disregard since our
          decisions don't depend on it).

          @return version extension in 'major.minor' format.
          @rtype str
        '''
        process = subprocess.Popen(["redis-server", "--version"], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        output, unused_error = process.communicate()
        try:
            version_string = re.search('v=([0-9.]+)', output).group(1)  # 2.6+ pattern
        except AttributeError:
            version_string = re.search('version ([0-9.]+)', output).group(1)  # 2.2 pattern
        version = semantic_version.Version(version_string)
        rospy.loginfo("Hub : version %s" % (version_string))
        return str(version.major) + "." + str(version.minor)

    def _setup(self):
        '''
          Clear and configure redis conf, log files in the ros home
          directories under a subdirectory styled by the name of this hub.

          Also check that we have support for the redis server - i.e. check if we
          have a .conf file for that version and exit this script if not found.
        '''
        if os.path.isdir(self._home_dir):
            shutil.rmtree(self._home_dir)
        os.makedirs(self._home_dir)
        rospack = rospkg.RosPack()
        package_redis_conf_file = os.path.join(rospack.get_path('rocon_hub'), 'redis', 'redis-%s.conf' % self._version_extension)
        package_redis_conf_local_file = os.path.join(rospack.get_path('rocon_hub'), 'redis', 'redis-%s.conf.local' % self._version_extension)

        # Checks
        if not os.path.isfile(package_redis_conf_file):
            utils.logfatal("Hub : the version of the redis server you have installed is not supported by rocon.")
            sys.exit(utils.logfatal("Hub : please submit a ticket at https://github.com/robotics-in-concert/rocon_multimaster"))

        redis_conf_template = utils.read_template(package_redis_conf_file)
        redis_conf_template = instantiate_redis_conf_template(redis_conf_template, self._files['redis_conf_local'])  # drop the local file path to use into the settings
        redis_local_template = utils.read_template(package_redis_conf_local_file)
        redis_local_template = instantiate_local_conf_template(redis_local_template,
                                                               self._parameters['port'],
                                                               self._parameters['max_memory'],
                                                               self._files['redis_server_log'],
                                                               self._home_dir)
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
        # Launch as a separate process group so we can control when it gets shut down.
        self._process = subprocess.Popen(["redis-server", self._files['redis_conf']], preexec_fn=os.setpgrp)
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
            #rospy.loginfo("Hub : clearing hub variables on the redis server.")
        except redis.ConnectionError:
            pass
        try:
            # because we start the redis sserver as a separate process group, we need to handle its shutdown
            # as roslaunch knows nothing of it.
            self._process.send_signal(signal.SIGINT)
            self._process.wait()
        except OSError:
            pass  # process already shut down


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


def instantiate_local_conf_template(template, port, max_memory, logfile, working_dir):
    '''
      Variable substitution in a template file.

      @param port : port on which the server will run
      @type int
      @param pid_file : pathname to where the pid file will be stored
      @type string
      @param max_memory: how much memory to allocate to the redis server in bytes
      @type string (e.g. 10mb)
      @param logfile
      @type string
      @param working_dir : filesystem which redis uses to dump (can we turn this off in 2.6?)
      @type string
    '''
    return template % locals()

if __name__ == "__main__":
    pool = redis.ConnectionPool(host='localhost', port='6380', db=0)
    try:
        print "dude"
    except redis.exceptions.ConnectionError:
        print "err"
