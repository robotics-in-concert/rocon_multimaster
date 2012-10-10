#!/usr/bin/env python
#       
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_multimaster/master/multimaster_client/rocon_gateway/LICENSE 
#

import roslib; roslib.load_manifest('rocon_gateway')
import roslib.packages
import rospy
import re
from gateway_comms.msg import Connection
import utils

###############################################################################
# Functions
###############################################################################

def setupRosParameters():
    '''
    Returns the gateway parameters from the ros param server.
    Most of these should be fairly self explanatory.
    '''
    param = {}
    
    # Hub
    param['hub_uri'] = rospy.get_param('~hub_uri','')
    param['hub_whitelist'] = rospy.get_param('~hub_whitelist',[])
    param['hub_blacklist'] = rospy.get_param('~hub_blacklist',[])
    
    # Gateway
    param['name'] = rospy.get_param('~name','gateway')
    param['watch_loop_period'] = rospy.get_param('~watch_loop_period',5) # in seconds

    # Topics and services for pre-initialisation/configuration
    param['default_public_interface'] = rospy.get_param('~default_public_interface', '')
    param['default_blacklist'] = rospy.get_param('~default_blacklist', [])

    # self.param['remote_topic'] = rospy.get_param('~remote_topic','')
    # self.param['remote_service'] = rospy.get_param('~remote_service','')

    return param

def generateConnectionsFromParam(param):
    '''
      Converts a param of the suitable type (see default_blacklist.yaml)
      into a dictionary of Connection types.
      
      @return all connections as gateway_comms.msg.Connection objects in our usual keyed dictionary format 
      @rtype type keyed dictionary of Connection lists
    '''
    connections = utils.createEmptyConnectionTypeDictionary()
    for value in param:
        connection = Connection()
        connection.name = value['name']
        # maybe also check for '' here?
        pattern = re.compile("None",re.IGNORECASE)
        if pattern.match(value['node']):
            connection.node = None
        else:
            connection.node = value['node']
        connection.type = value['type']
        connections[connection.type].append(connection)
    return connections
