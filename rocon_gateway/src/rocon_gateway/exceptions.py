#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_multimaster/license/LICENSE
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


# Raised whenever any of the samples falls over in runtime
class GatewaySampleRuntimeError(Exception):
    pass
