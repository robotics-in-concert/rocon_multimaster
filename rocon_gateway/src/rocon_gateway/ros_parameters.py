#!/usr/bin/env python
#       
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_multimaster/master/rocon_gateway/LICENSE 
#

import roslib; roslib.load_manifest('rocon_gateway')
import roslib.packages
import rospy
import re
from gateway_msgs.msg import Rule, RemoteRule
import utils

###############################################################################
# Functions
###############################################################################

def setup_ros_parameters():
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
    param['watch_loop_period'] = rospy.get_param('~watch_loop_period',10) # in seconds
    
    # Blacklist used for advertise all, flip all and pull all commands
    param['default_blacklist'] = rospy.get_param('~default_blacklist', []) # list of Rule objects

    # Used to block/permit remote gateway's from flipping to this gateway.
    param['firewall'] = rospy.get_param('~firewall', True)

    # Make everything publicly available (excepting the default blacklist)
    param['advertise_all'] = rospy.get_param('~advertise_all', [])  # boolean

    # Topics and services for pre-initialisation/configuration
    param['default_advertisements'] = rospy.get_param('~default_advertisements', [])  # list of Rule objects
    param['default_flips'] = rospy.get_param('~default_flips', [])  # list of RemoteRule objects
    param['default_pulls'] = rospy.get_param('~default_pulls', [])  # list of RemoteRule objects

    return param

def generate_rules(param):
    '''
      Converts a param of the suitable type (see default_blacklist.yaml)
      into a dictionary of Rule types.
      
      @return all rules as gateway_msgs.msg.Rule objects in our usual keyed dictionary format 
      @rtype type keyed dictionary of Rule lists
    '''
    rules = utils.createEmptyConnectionTypeDictionary()
    for value in param:
        rule = Rule()
        rule.name = value['name']
        # maybe also check for '' here?
        pattern = re.compile("None",re.IGNORECASE)
        if pattern.match(value['node']):
            rule.node = '' #ROS Message fields should not be None, maybe not a problem here though, see below
        else:
            rule.node = value['node']
        rule.type = value['type']
        rules[rule.type].append(rule)
    return rules

def generate_remote_rules(param):
    ''' 
       Converts a param of the suitable type (default_flips, default_pulls) into
       a list of RemoteRule objects and a list of target gateways for flip_all/pull_all.
       
       @param yaml object
       @type complicated
       
       @return list of remote rules
       @return RemoteRule[]
    '''
    remote_rules = []
    all_targets = []
    pattern = re.compile("None",re.IGNORECASE)
    for remote_rule in param:
        if 'rule' in remote_rule:
            # maybe also check for '' here?
            node = None if pattern.match(remote_rule['rule']['node']) else remote_rule['rule']['node'] 
            remote_rules.append(RemoteRule(remote_rule['gateway'],
                                       Rule(remote_rule['rule']['type'],
                                            remote_rule['rule']['name'],
                                            node
                                            )
                                       )
                            )
        else:
            all_targets.append(remote_rule['gateway'])
    return remote_rules, all_targets
