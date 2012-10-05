#!/usr/bin/env python
#       
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_multimaster/master/multimaster_client/rocon_gateway/LICENSE 
#

'''
  Core exceptions raised by the gateway.
'''
##############################################################################
# Exceptions
##############################################################################

class GatewayError(Exception):
    pass


class ConnectionTypeError(GatewayError):
    pass

