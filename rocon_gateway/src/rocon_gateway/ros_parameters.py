#!/usr/bin/env python
#       
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_multimaster/master/multimaster_client/rocon_gateway/LICENSE 
#

import roslib; roslib.load_manifest('rocon_gateway')
import roslib.packages
import rospy
import re
from gateway_comms.msg import Rule
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

def generateRules(param):
    '''
      Converts a param of the suitable type (see default_blacklist.yaml)
      into a dictionary of Rule types.
      
      @return all rules as gateway_comms.msg.Rule objects in our usual keyed dictionary format 
      @rtype type keyed dictionary of Rule lists
    '''
    rules = utils.createEmptyConnectionTypeDictionary()
    for value in param:
        rule = Rule()
        rule.name = value['name']
        # maybe also check for '' here?
        pattern = re.compile("None",re.IGNORECASE)
        if pattern.match(value['node']):
            rule.node = None
        else:
            rule.node = value['node']
        rule.type = value['type']
        rules[rule.type].append(rule)
    return rules
