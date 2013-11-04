#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_multimaster/master/rocon_utilities/LICENSE
#

##############################################################################
# Imports
##############################################################################

import rospy
import rospkg
import roslib
import roslib.names

##############################################################################
# Resources
##############################################################################


def find_resource_from_string(resource):
    '''
      Convenience wrapper around roslib to find a resource (file) inside
      a package. This function passes off the work to find_resource
      once the input string is split.

      @param package : ros package
      @param resource : string resource identifier of the form package/filename

      @raise IOError : raised if the resource string couldn't be validated into package/remainder.
    '''
    package, filename = roslib.names.package_resource_name(resource)
    if not package:
        raise IOError("resource could not be split with a valid leading package name [%s]" % (resource))
    return find_resource(package, filename)


def find_resource(package, filename):
    '''
      Convenience wrapper around roslib to find a resource (file) inside
      a package. It checks the output, and provides the appropriate
      error if there is one.

      @param package : ros package
      @param filename : some file inside the specified package
      @return str : absolute path to the file

      @raise IOError : raised if there is nothing found or multiple objects found.
    '''
    try:
        resolved = roslib.packages.find_resource(package, filename)
        if not resolved:
            raise IOError("cannot locate [%s] in package [%s]" % (filename, package))
        elif len(resolved) == 1:
            return resolved[0]
        elif len(resolved) > 1:
            raise IOError("multiple resources named [%s] in package [%s]:%s\nPlease specify full path instead" % (filename, package, ''.join(['\n- %s' % r for r in resolved])))
    except rospkg.ResourceNotFound:
        raise IOError("[%s] is not a package or launch file name" % package)
    return None


def is_absolute_name(name):
    '''
      Checks if the name begins with a leading slash which validates it
      either as an absolute or relative name in the ros world.
    '''
    return name[:1] == '/'

##############################################################################
# Subscriber Proxy
##############################################################################


class SubscriberProxy():
    '''
      Works like a service proxy, but using a subscriber instead.
    '''
    def __init__(self, topic, msg_type):
        '''
          @param topic : the topic name to subscriber to
          @type str
          @param msg_type : any ros message type (typical arg for the subscriber)
          @type msg
          @param timeout : timeout on the wait operation (None = /infty)
          @type rospy.Duration()
          @return msg type data or None
        '''
        self._subscriber = rospy.Subscriber(topic, msg_type, self._callback)
        self._data = None

    def __call__(self, timeout=None):
        '''
          Returns immediately with the latest data or waits for
          incoming data.

          @param timeout : time to wait for data, polling at 10Hz.
          @type rospy.Duration
          @return latest data or None
        '''
        r = rospy.Rate(10)
        start_time = rospy.Time.now()
        while not rospy.is_shutdown() and self._data == None:
            r.sleep()
            if timeout:
                if rospy.Time.now() - start_time > timeout:
                    return None
        return self._data

    def wait_for_next(self, timeout=None):
        '''
          Makes sure any current data is cleared and waits for new data.
        '''
        self._data = None
        return self.__call__(timeout)

    def wait_for_publishers(self):
        '''
          Blocks until publishers are seen.

          @raise rospy.exceptions.ROSInterruptException if we are in shutdown.
        '''
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self._subscriber.get_num_connections() != 0:
                return
            else:
                r.sleep()
        # we are shutting down
        raise rospy.exceptions.ROSInterruptException

    def _callback(self, data):
        self._data = data

    def unregister(self):
        '''
          Unregister the subscriber so future instantiations of this class can pull a
          fresh subscriber (important if the data is latched).
        '''
        self._subscriber.unregister()
