#!/usr/bin/env python
#       
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_multimaster/master/multimaster_client/rocon_gateway_sync/LICENSE 
#

'''
  Core exceptions raised by the gateway.
'''
##############################################################################
# Exceptions
##############################################################################

class GatewayError(Exception):
    pass

# for exceptions that are not errors
class GatewayException(Exception):
    '''
      For gateway exceptions that are not errors.
    '''
    pass

#class NothingToSeeHereException(GatewayException):
#    '''
#      Used sometimes when we want to throw an exception
#      to inform a higher level code block that nothing was done.
#    '''
#    pass

class ConnectionTypeError(GatewayError):
    pass

