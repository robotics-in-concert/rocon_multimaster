#!/usr/bin/env python
#       
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_multimaster/master/multimaster_client/rocon_gateway/LICENSE 
#

###############################################################################
# Functions
###############################################################################

def resolveZeroconfAddress(msg):
    '''
      Resolves a zeroconf address into ip/port portions.
      @var msg : zeroconf_comms.DiscoveredService 
      @return (string,int) : ip, port pair.
    '''
    ip = "localhost"
    if not msg.is_local:
        ip = msg.ipv4_addresses[0]
    return (ip,msg.port)
    
