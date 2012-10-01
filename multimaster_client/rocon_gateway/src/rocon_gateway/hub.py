#!/usr/bin/env python
#       
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_multimaster/master/multimaster_client/rocon_gateway/LICENSE 
# Copyright (c) 2012, Yujin Robot, Daniel Stonier
#

import redis

###############################################################################
# Functions
###############################################################################

def resolveHub(ip, port):
    '''
      Pings the hub for identification. We currently use this to check
      against the gateway whitelist/blacklists to determine if a connection
      should proceed or not.
      
      @return string - hub name
    '''
    r = redis.Redis()
    return r.get("rocon:hub:name") # perhaps should store all key names somewhere central
    
