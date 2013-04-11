#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_multimaster/master/rocon_gateway/LICENSE
#

'''
  Core exceptions raised by the gateway.
'''
##############################################################################
# Exceptions
##############################################################################


class GatewayError(Exception):
    pass


# Raised when information for a specific gateway id is requested, but that
# gateway is not connected to the hub
class GatewayUnavailableError(Exception):
    pass


# Raised when a connection type of the kind gateway_msgs.ConnectionType is
# invalid
class ConnectionTypeError(GatewayError):
    pass


class HubConnectionLostError(GatewayError):
    pass


# Raised when the gateway can't connect to the hub's redis server
class HubUnavailableError(GatewayError):
    pass


# Raised when the hub's redis server has no key setting for the hub name.
class HubNameNotFoundError(GatewayError):
    pass
