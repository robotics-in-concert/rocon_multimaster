#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_multimaster/license/LICENSE
#

##############################################################################
# Imports
##############################################################################

import rocon_std_msgs.msg as rocon_std_msgs

from .exceptions import InvalidPlatformTuple

##############################################################################
# Conversions
##############################################################################


def to_string(msg):
    '''
      Convert a PlatformTuple message to a string tuple with '.' separators.

      @param msg : platform info message to convert.
      @type rocon_std_msgs.PlatformTuple

      @return string tuple representation of the platform tuple
      @rtype string
    '''
    return '.'.join([msg.os, msg.version, msg.system, msg.platform, msg.name])


def to_msg(platform_tuple_string):
    '''
      This fills out the platform tuple (e.g. ubuntu.precise.ros.turtlebot.Dude)
      fields of a rocon_std_msgs/PlatformTuple message.

      @param platform_tuple_string : string representation
      @type str

      @return platform tuple message
      @rtype rocon_std_msgs.PlatformTuple

      @raise InvalidPlatformTuple
    '''
    try:
        assert_valid(platform_tuple_string)
    except InvalidPlatformTuple as e:
        raise e
    platform_tuple_list = platform_tuple_string.split('.')
    msg = rocon_std_msgs.PlatformTuple()
    msg.os = platform_tuple_list[0]
    msg.version = platform_tuple_list[1]
    msg.system = platform_tuple_list[2]
    msg.platform = platform_tuple_list[3]
    msg.name = platform_tuple_list[4]
    return msg

##############################################################################
# Setters
##############################################################################


def set_name(platform_tuple_string, name):
    '''
      Replaces the name part of the tuple with the provided name.

      @param platform_tuple_string
      @type string

      @param name
      @type string
    '''
    # Check for valid name
    (platform_part, unused_separator, unused_name) = platform_tuple_string.rpartition('.')
    return platform_part + '.' + name


def get_name(platform_tuple_string):
    '''
      Returns the name part of a platform info string.

      @param platform_tuple_string
      @type string

      @return name
      @rtype string
    '''
    # Check for valid name
    (unused_platform_part, unused_separator, name) = platform_tuple_string.rpartition('.')
    return name


##############################################################################
# Validation
##############################################################################


def assert_valid(platform_tuple_string):
    '''
      Make various checks to ensure the string tuple is valid. Throws exceptions
      if not.

      @param platform_tuple_string
      @type string

      @raise InvalidPlatformTuple
    '''
    platform_tuple_list = platform_tuple_string.split('.')
    if len(platform_tuple_list) != 5:
        raise InvalidPlatformTuple("platform tuple string not of the correct length [%s != 5]" % len(platform_tuple_list))
    # Check for no illegal characters
    # Check for correct regular expression wildcard combinations

##############################################################################
# Matching
##############################################################################


def is_compatible(platform_tuple_a, platform_tuple_b):
    '''
      Checks if the platform_tuples are compatible. Especially used
      for checking whether a rapp is compatible with its platform.

      If you want to compare platform_tuple strings instead of messages,
      apply the previous functions on the arguments, e.g.:

        if is_compatible(to_msg(a), to_msg(b))

      Note that this accomodates matching against wildcard characters ('*').
      Later it would be good to introduce regex matching.

      @param platform_tuple_a
      @type rocon_std_msgs/PlatformTuple

      @param platform_tuple_b
      @type rocon_std_msgs/PlatformTuple

      @result true if the first tuple is compatible with the second
      @rtype boolean
    '''
    if platform_tuple_a.os != platform_tuple_b.os and \
       platform_tuple_a.os != rocon_std_msgs.PlatformTuple.OS_ANY and \
       platform_tuple_b.os != rocon_std_msgs.PlatformTuple.OS_ANY:
        return False
    if platform_tuple_a.version != platform_tuple_b.version and \
       platform_tuple_a.version != rocon_std_msgs.PlatformTuple.VERSION_ANY and \
       platform_tuple_b.version != rocon_std_msgs.PlatformTuple.VERSION_ANY:
        return False
    if platform_tuple_a.system != platform_tuple_b.system and \
       platform_tuple_a.system != rocon_std_msgs.PlatformTuple.SYSTEM_ANY and \
       platform_tuple_b.system != rocon_std_msgs.PlatformTuple.SYSTEM_ANY:
        return False
    if platform_tuple_a.platform != platform_tuple_b.platform and \
       platform_tuple_a.platform != rocon_std_msgs.PlatformTuple.PLATFORM_ANY and \
       platform_tuple_b.platform != rocon_std_msgs.PlatformTuple.PLATFORM_ANY:
        return False
    if platform_tuple_a.name != platform_tuple_b.name and \
       platform_tuple_a.name != rocon_std_msgs.PlatformTuple.NAME_ANY and \
       platform_tuple_b.name != rocon_std_msgs.PlatformTuple.NAME_ANY:
        return False
    return True


def matches(platform_tuple_a, platform_tuple_b):
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

      @param platform_tuple_a
      @type rocon_std_msgs.PlatformTuple

      @param platform_tuple_b
      @type rocon_std_msgs.PlatformTuple

      @result true if the first argument matches the second's specification
      @rtype boolean
    '''
    if platform_tuple_a.os != platform_tuple_b.os and \
            platform_tuple_b.os != rocon_std_msgs.PlatformTuple.OS_ANY:
        return False
    if platform_tuple_a.version != platform_tuple_b.version and \
            platform_tuple_b.version != rocon_std_msgs.PlatformTuple.VERSION_ANY:
        return False
    if platform_tuple_a.system != platform_tuple_b.system and \
            platform_tuple_b.system != rocon_std_msgs.PlatformTuple.SYSTEM_ANY:
        return False
    if platform_tuple_a.platform != platform_tuple_b.platform and \
            platform_tuple_b.platform != rocon_std_msgs.PlatformTuple.PLATFORM_ANY:
        return False
    if platform_tuple_a.name != platform_tuple_b.name and \
            platform_tuple_b.name != rocon_std_msgs.PlatformTuple.NAME_ANY:
        return False
    return True


def string_matches(platform_tuple_string_a, platform_tuple_string_b):
    '''
      A tuple string equivalent of the matches command.

      @param platform_tuple_string_a
      @type string

      @param platform_tuple_string_b
      @type string

      @result true if the first argument matches the second's specification
      @rtype boolean
    '''
    return matches(to_msg(platform_tuple_string_a), to_msg(platform_tuple_string_b))
