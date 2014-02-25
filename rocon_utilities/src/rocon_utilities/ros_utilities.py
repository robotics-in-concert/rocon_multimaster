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
from rosservice import rosservice_find

from .exceptions import ServiceNotFoundException
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

      @return full pathname to the resource
      @rtype str

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


def find_service(service_type, timeout=rospy.rostime.Duration(5.0)):
    service_name = None
    timeout_time = time.time() + timeout.to_sec()
    while not rospy.is_shutdown() and time.time() < timeout_time and not service_name:
        service_names = rosservice_find(service_type)
        if len(service_names) > 1:
            raise ServiceNotFoundException("multiple services found %s." % service_names)
        elif len(service_names) == 1:
            service_name = service_names[0]
        else:
            rospy.rostime.wallsleep(0.1)
    if service_name is None:
        raise ServiceNotFoundException("timed out")
    return service_name
