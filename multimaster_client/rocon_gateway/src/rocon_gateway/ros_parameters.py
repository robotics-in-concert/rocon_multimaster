#!/usr/bin/env python
#       
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_multimaster/master/multimaster_client/rocon_gateway/LICENSE 
#

import roslib; roslib.load_manifest('rocon_gateway')
import roslib.packages
import rospy
import yaml
from gateway_comms.msg import Connection

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

    param['default_public_interface'] = rospy.get_param('~default_public_interface', '')
    param['blacklist'] = rospy.get_param('~blacklist', roslib.packages.get_pkg_dir('rocon_gateway') + '/param/default_blacklist.yaml')

    # param['default_topics_blacklist'] = rospy.get_param('~default_topics_blacklist', '.*zeroconf.*,.*gateway.*,.*rosout.*,.*parameter_descriptions,.*parameter_updates,/tf')
    # param['default_services_blacklist'] = rospy.get_param('~default_services_blacklist', '.*zeroconf.*,.*gateway.*,.*get_loggers,.*set_logger_level')
    
    # Topics and services that need from remote server
    # self.param['remote_topic'] = rospy.get_param('~remote_topic','')
    # self.param['remote_service'] = rospy.get_param('~remote_service','')

    return param

def parseConnectionsFromFile(file):
    '''
    Takes a YAML file supplied as a ROS parameter, and parses a list of
    connection messages from it

    @param file : absolute location of the file to parse
    @type str
    '''
    connections = dict()
    connections[Connection.PUBLISHER] = set()
    connections[Connection.SUBSCRIBER] = set()
    connections[Connection.SERVICE] = set()
    connections[Connection.ACTION_SERVER] = set()
    connections[Connection.ACTION_CLIENT] = set()
    try:
        stream = open(file, 'r')
        list = yaml.load(stream)
    except:
        rospy.logerr('Gateway : Unable to load yaml from file [%s]'%file)
        return connections

    if list:
        for l in list:
            try:
                connections[l['type']].add(tuple(l['list']))
            except:
                rospy.logerr('Gateway : Unable to parse item in file [%s]'%str(l))
    return connections

