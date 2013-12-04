#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_multimaster/license/LICENSE
#

##############################################################################
# Imports
##############################################################################

import rocon_std_msgs.msg as rocon_std_msgs

from .exceptions import InvalidPlatformInfoString

##############################################################################
# Resources
##############################################################################


def to_string(msg):
    '''
      Convert a PlatformInfo message to a string tuple with '.' separators.

      @param msg : platform info message to convert.
      @type rocon_std_msgs.PlatformInfo

      @return string tuple representation of the platform info
      @rtype string
    '''
    return '.'.join([msg.os, msg.version, msg.system, msg.platform, msg.name])


def to_msg(platform_info_string_tuple):
    '''
      Converts a platform info string tuple (e.g. ubuntu.precise.ros.turtlebot.Dude)
      into it's message equivalent.

      @param platform_info_string_tuple : string representation
      @type str

      @return platform info message
      @rtype rocon_std_msgs.PlatformInfo

      @raise InvalidPlatformInfoString
    '''
    platform_info_tuple_list = platform_info_string_tuple.split('.')
    if len(platform_info_tuple_list) != 5:
        raise InvalidPlatformInfoString("Platform info string tuple not of the correct length [%s != 5]" % len(platform_info_tuple_list))
    msg = rocon_std_msgs.PlatformInfo()
    msg.os = platform_info_tuple_list[0]
    msg.version = platform_info_tuple_list[1]
    msg.system = platform_info_tuple_list[2]
    msg.platform = platform_info_tuple_list[3]
    msg.name = platform_info_tuple_list[4]
    return msg


def matches(platform_info_a, platform_info_b):
    '''
      Checks if first argument matches the second. If you want equality,
      be sure to check the result both ways, e.g.

        if matches(a,b) and matches(b,a):

      If you want to compare strings instead of messages, apply the previous
      functions on the arguments, e.g.:

        if matches(to_msg(a), to_msg(b))

      or use the string_matches method. Note that this accomodates
      matching against wildcard characters ('*'). Later it would be good to even
      introduce regex matching.

      @param platform_info_a
      @type rocon_std_msgs.PlatformInfo

      @param platform_info_b
      @type rocon_std_msgs.PlatformInfo

      @result true if the first argument matches the second's specification
      @rtype boolean
    '''
    if platform_info_a.os != platform_info_b.os and \
            platform_info_b.os != rocon_std_msgs.PlatformInfo.OS_ANY:
        return False
    if platform_info_a.version != platform_info_b.version and \
            platform_info_b.version != rocon_std_msgs.PlatformInfo.VERSION_ANY:
        return False
    if platform_info_a.system != platform_info_b.system and \
            platform_info_b.system != rocon_std_msgs.PlatformInfo.SYSTEM_ANY:
        return False
    if platform_info_a.platform != platform_info_b.platform and \
            platform_info_b.platform != rocon_std_msgs.PlatformInfo.PLATFORM_ANY:
        return False
    if platform_info_a.name != platform_info_b.name and \
            platform_info_b.name != rocon_std_msgs.PlatformInfo.NAME_ANY:
        return False
    return True


def string_matches(platform_info_a, platform_info_b):
    '''
      A tuple string equivalent of the matches command.

      @param platform_info_a in tuple string format.
      @type string

      @param platform_info_b in tuple string format.
      @type string

      @result true if the first argument matches the second's specification
      @rtype boolean
    '''
    return matches(to_msg(platform_info_a), to_msg(platform_info_b))
