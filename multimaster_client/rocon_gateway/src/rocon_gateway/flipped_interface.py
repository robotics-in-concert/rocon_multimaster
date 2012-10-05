#!/usr/bin/env python
#       
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_multimaster/master/multimaster_client/rocon_gateway/LICENSE 
#

##############################################################################
# Imports
##############################################################################

from .utils import Connection, connectionTypeString, connectionType

##############################################################################
# Public Interface
##############################################################################

class FlippedInterface(object):
    '''
      The flipped interface is the set of connections (pubs/subs/services/actions)
      that are flipped to other gateways.
    '''
    def __init__(self):
        self.interface = {}

    ##########################################################################
    # Flipped Interfaces
    ##########################################################################


    ##########################################################################
    # Filters
    ##########################################################################

    # ToDo