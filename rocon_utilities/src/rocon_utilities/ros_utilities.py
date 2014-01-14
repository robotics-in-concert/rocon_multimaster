#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_multimaster/license/LICENSE
#

##############################################################################
# Imports
##############################################################################

import os
import time
import rospy
import rospkg
import roslib.names
from catkin_pkg.packages import find_packages

##############################################################################
# Resources
##############################################################################


def find_resource_from_string(resource, rospack=None, extension=None):
    '''
      Convenience wrapper around roslib to find a resource (file) inside
      a package. This function passes off the work to find_resource
      once the input string is split.

      @param package : ros package
      @param resource : string resource identifier of the form package/filename

      @param extension : file name extension to look for/expect
      @type string

      @raise IOError : raised if the resource is not found or has an inappropriate extension.
    '''
    if extension is not None:
        filename_extension = os.path.splitext(resource)[-1]
        if filename_extension == '':  # no ext given
            resource += ".%s" % extension
        elif filename_extension != "." + extension and filename_extension != extension:
            raise IOError("resource with invalid filename extension specified [%s][%s]" % (resource, extension))
    package, filename = roslib.names.package_resource_name(resource)
    if not package:
        raise IOError("resource could not be split with a valid leading package name [%s]" % (resource))
    return find_resource(package, filename, rospack)


def find_resource(package, filename, rospack=None):
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
        resolved = roslib.packages.find_resource(package, filename, rospack=rospack)
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

      Note : this is redundant with roslib.names.is_global(name)

      https://github.com/ros/ros/blob/hydro-devel/core/roslib/src/roslib/names.py#L113
    '''
    return name[:1] == '/'


def package_index_from_package_path(package_paths):
    """Find all packages on the given list of paths

    Iterates over the given list of paths in reverse order so that packages
    found in the paths at the beginning of the list get overlaid onto packages
    with the same name which were found in paths farther back in the list.

    The resulting dictionary is keyed by the package name (so packages with
    duplicate names are overlaid) and the values are the
    :py:class:`catkin_pkg.package.Package` class

    @note Is this actually implemented as a function in a general ros package?

    :param ros_package_path: list of paths to search
    :type ros_package_path: list
    :returns: dictionary of package objects keyed by name of the package
    :rtype: dict
    """
    result = {}
    for path in reversed(package_paths):
        for unused_package_path, package in find_packages(path).items():
            result[package.name] = package
    return result

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
        if timeout:
            # everything in floating point calculations
            timeout_time = time.time() + timeout.to_sec()
        while not rospy.is_shutdown() and self._data == None:
            rospy.rostime.wallsleep(0.1)
            if timeout:
                if time.time() > timeout_time:
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
