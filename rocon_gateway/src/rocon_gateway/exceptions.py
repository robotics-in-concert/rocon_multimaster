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
#    '''
#      Throw an exception with the hub to be handled.
#    '''
#    def __init__(self, hub):
#        '''
#          @param message : usual exception message
#          @param hub : hub that lost connection
#          @type : hub_api.Hub
#        '''
#        # Call the base class constructor with the parameters it needs
#        GatewayError.__init__(self, '')
#        self.hub = hub


# Raised when the gateway can't connect to the hub's redis server
class HubNotFoundError(GatewayError):
    pass


# Raised when an operation tries to use an unoconnected hub
class HubNotConnectedError(GatewayError):
    pass


# Raised when the hub's redis server has no key setting for the hub name.
class HubNameNotFoundError(GatewayError):
    pass


# Raised whenever any of the samples falls over in runtime
class GatewaySampleRuntimeError(Exception):
    pass
