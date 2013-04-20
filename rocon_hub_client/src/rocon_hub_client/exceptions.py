#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_multimaster/master/rocon_hub_client/LICENSE
#

'''
  Core exceptions raised when interacting with a rocon hub.
'''
##############################################################################
# Imports
##############################################################################

from gateway_msgs.msg import ErrorCodes

##############################################################################
# Exceptions
##############################################################################


class HubError(Exception):
    def __init__(self, msg):
        super(HubError, self).__init__(msg)
        self.id = ''


# Raised when the gateway can't connect to the hub's redis server
class HubNotFoundError(HubError):
    def __init__(self, msg):
        super(HubNotFoundError, self).__init__(msg)
        self.id = ErrorCodes.HUB_CONNECTION_UNRESOLVABLE


# Raised when the hub's redis server has no key setting for the hub name.
class HubNameNotFoundError(HubError):
    def __init__(self, msg):
        super(HubNameNotFoundError, self).__init__(msg)
        self.id = ErrorCodes.HUB_NAME_NOT_FOUND


# When the hub name or uri is in the blacklist
class HubConnectionBlacklistedError(HubError):
    def __init__(self, msg):
        super(HubConnectionBlacklistedError, self).__init__(msg)
        self.id = ErrorCodes.HUB_CONNECTION_BLACKLISTED


# When the hub name or uri is not in a non-empty whitelist
class HubConnectionNotWhitelistedError(HubError):
    def __init__(self, msg):
        super(HubConnectionNotWhitelistedError, self).__init__(msg)
        self.id = ErrorCodes.HUB_CONNECTION_NOT_IN_NONEMPTY_WHITELIST


# When the hub name or uri is not in a non-empty whitelist
class HubConnectionAlreadyExistsError(HubError):
    def __init__(self, msg):
        super(HubConnectionAlreadyExistsError, self).__init__(msg)
        self.id = ErrorCodes.HUB_CONNECTION_NOT_IN_NONEMPTY_WHITELIST


# Raised when a hub client tries an operation on a hub that has lost its
# connection
class HubConnectionLostError(HubError):
    def __init__(self, msg):
        super(HubConnectionLostError, self).__init__(msg)
        self.id = ErrorCodes.HUB_CONNECTION_NOT_IN_NONEMPTY_WHITELIST
