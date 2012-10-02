#!/usr/bin/env python
#       
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_multimaster/master/multimaster_client/rocon_gateway/LICENSE 
#

import roslib; roslib.load_manifest('rocon_gateway')
import rospy

###############################################################################
# Functions
###############################################################################

def rosParameters():
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

    # Topics and services for pre-initialisation/configuration
    param['local_public_topic'] = rospy.get_param('~local_public_topic',[])
    param['local_public_service'] = rospy.get_param('~local_public_service',[])
    param['public_named_topics'] = rospy.get_param('~public_named_topics', '')
    param['public_named_topics_blacklist'] = rospy.get_param('~public_named_topics_blacklist', '.*zeroconf.*,.*gateway.*,.*rosout.*,.*parameter_descriptions,.*parameter_updates,/tf')
    param['public_named_services'] = rospy.get_param('~public_named_services', '')
    param['public_named_services_blacklist'] = rospy.get_param('~public_named_services_blacklist', '.*zeroconf.*,.*gateway.*,.*get_loggers,.*set_logger_level')
    
    # Topics and services that need from remote server
    # self.param['remote_topic'] = rospy.get_param('~remote_topic','')
    # self.param['remote_service'] = rospy.get_param('~remote_service','')

    return param
