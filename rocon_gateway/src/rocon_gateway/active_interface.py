#!/usr/bin/env python
#       
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_multimaster/master/rocon_gateway/LICENSE 
#

##############################################################################
# Imports
##############################################################################

import roslib; roslib.load_manifest('rocon_gateway')
import threading

# Local imports
import utils

##############################################################################
# Classes
##############################################################################

class ActiveInterface(object):
    '''
      Parent interface for flip and pull interfaces.
    '''
    def __init__(self, default_rule_blacklist):
        # Default rules used in the xxxAll modes
        self._default_blacklist = default_rule_blacklist # dictionary of gateway-gateway_comms.msg.Rule lists, not RemoteRules!
        
        # keys are connection_types, elements are lists of gateway_comms.msg.RemoteRule objects
        self.watchlist = utils.createEmptyConnectionTypeDictionary()    # Specific rules used to determine what local rules to flip  
        
        # keys are connection_types, elements are lists of utils.Registration objects
        self.registrations = utils.createEmptyConnectionTypeDictionary() # Flips from remote gateways that have been locally registered
        
        # Blacklists when doing flip all - different for each gateway, each value is one of our usual rule type dictionaries
        self._blacklist = {}

        self._lock = threading.Lock()
