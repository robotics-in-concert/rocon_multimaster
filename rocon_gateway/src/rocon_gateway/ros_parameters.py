#!/usr/bin/env python
#       
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_multimaster/master/multimaster_client/rocon_gateway/LICENSE 
#

import roslib; roslib.load_manifest('rocon_gateway')
import roslib.packages
import rospy

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
    param['blacklist'] = rospy.get_param('~blacklist', roslib.packages.get_pkg_dir('rocon_gateway') + '/param/default_blacklist.yaml')

    # param['default_topics_blacklist'] = rospy.get_param('~default_topics_blacklist', '.*zeroconf.*,.*gateway.*,.*rosout.*,.*parameter_descriptions,.*parameter_updates,/tf')
    # param['default_services_blacklist'] = rospy.get_param('~default_services_blacklist', '.*zeroconf.*,.*gateway.*,.*get_loggers,.*set_logger_level')

    # Topics and services that need from remote server
    # self.param['remote_topic'] = rospy.get_param('~remote_topic','')
    # self.param['remote_service'] = rospy.get_param('~remote_service','')

    return param

