#!/usr/bin/env python
#       
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_multimaster/master/multimaster_client/rocon_gateway_sync/LICENSE 
#

##############################################################################
# Imports
##############################################################################

import re
import yaml
import rospy
from gateway_comms.msg import Connection

##############################################################################
# Ros string utilities
##############################################################################

def reshapeUri(uri):
    '''
      Correct human vagarities, make sure the string always ends with a '/'.
      
      @param uri: a simple uri
      @type  uri: str
      @return: uri string with a trailing slash
      @rtype: str
    '''
    if uri[len(uri)-1] is not '/':
        uri = uri + '/'
    return uri

def reshapeTopic(t):
    '''
      Correct human vagarities, make sure the topic name always starts with a '/'.
      
      @param t: a simple topic name
      @type  t: str
      @return: properly namespaced topic string
      @rtype: str
    '''
    if t[0] is not '/':
        t = '/' + t
    return t

##############################################################################
# Connections - usually our wierd triple style string representations.
##############################################################################

def enum(*sequential, **named):
    enums = dict(zip(sequential, range(len(sequential))), **named)
    return type('Enum', (), enums)

ConnectionEnum = enum('topic','service','action', 'invalid')
ConnectionStrings = ['topic','service','action', 'invalid']

def connectionType(connection):
    '''
      Checks a connection string representation to determine if it is a 
      topic, service or action. Topic types are always represented by a triple:
      
       - topic    { name, type, xmlrpc node uri }
                  { /chatter,std_msgs/String,http://snorriheim:35403/ }
       - service  { name, rosrpc uri, xmlrpc node uri }
                  { /add_two_ints,rosrpc://snorriheim:59822,http://snorriheim:34035/ }
       - action   ???
        
      @param connection : the string representation for a connection 
    '''
    components = connection.split(',')
    if len( components ) < 2: 
        return ConnectionEnum.invalid
    if re.match('rosrpc',components[1]):
        return ConnectionEnum.service
    if re.match('http://',components[2]):
        return ConnectionEnum.topic
    # action not implemented yet
    return ConnectionEnum.invalid

def connectionTypeString(connection):
    '''
      Checks a connection string representation to determine if it is a 
      topic, service or action. Returns a string representation of the type.
      
                { "topic", "service", "action", "invalid" }
        
      @param connection : the string representation for a connection
      @return string : the string representation for the connection type. 
    '''
    return ConnectionStrings[connectionType(connection)] 

def getEmptyConnectionList():
    '''
    Supplies a empty watchlist/blacklist. The watchlist/blacklist is designed
    as a dictionary over connection types. Each value inside the dictionary is
    a unique set of tuples of connection information

    @return connections: empty watchlist/blacklist
    @type dict of sets of tuples
    '''
    connections = dict()
    connections[Connection.PUBLISHER] = set()
    connections[Connection.SUBSCRIBER] = set()
    connections[Connection.SERVICE] = set()
    connections[Connection.ACTION_SERVER] = set()
    connections[Connection.ACTION_CLIENT] = set()
    return connections

def getAllAllowedConnectionList():
    '''
    Returns a connection list where everything is allowed for every connection
    type. Useful for *_all requests. A '.*' regex filter is put in for every
    connection type

    @return connections: connectionlist with 
    @type dict of sets of tuples
    '''
    connections = dict()
    connections[Connection.PUBLISHER] = set(('.*',))
    connections[Connection.SUBSCRIBER] = set(('.*',))
    connections[Connection.SERVICE] = set(('.*',))
    connections[Connection.ACTION_SERVER] = set(('.*',))
    connections[Connection.ACTION_CLIENT] = set(('.*',))
    return connections
        
def parseConnectionsFromFile(file):
    '''
    Takes a YAML file , and parses a list of connection tuples and type from it

    @param file : absolute location of the file to parse
    @type str
    @return connections: parsed watchlist/blacklist
    @type dict of sets of tuples
    '''
    connections = getEmptyConnectionList()
    try:
        stream = open(file, 'r')
        list = yaml.load(stream)
    except:
        rospy.logerr('Gateway : Unable to load yaml from file [%s]'%file)
        return connections

    try:
        for connection_type in list:
            for l in list[connection_type]: 
                connections[connection_type].add(tuple(l))
    except Exception as e:
        rospy.logerr('Gateway : Unable to parse item in yaml file [%s]'%str(e))
    return connections

def connectionsFromConnectionMsgList(msg_list):
    '''
    Converts a list of gateway_comms/Connection messages to the internal easily
    indexable format

    @param msg_list: list of Connection msgs
    @return connections: parsed watchlist/blacklist
    @type dict of sets of tuples
    '''
    connections = getEmptyConnectionList()
    for connection in msg_list:
        connections[connection.type].add(tuple(connection.list))
    return connections

def connectionMsgListFromConnections(connections):
    '''
    Converts the internal easily indexable connectionlist format into a list of
    connection messages

    @param connections: parsed watchlist/blacklist
    @type dict of sets of tuples
    @return msg_list: list of Connection msgs
    '''
    msg_list = []
    for connection_type in connections:
        for l in connections[connection_type]:
            connection = Connection()
            connection.type = connection_type
            connection.list = list(l)
            msg_list.append(connection)
    return msg_list

def addToConnectionList(list, additions):
    '''
    Performs a set update for each individual connection type. Tuples from 
    additions are added to the list.
    '''
    for connection_type in list:
        list[connection_type] |= additions[connection_type]

def removeFromConnectionList(list, subtractions):
    '''
    Performs a set difference update for each individual connection type. Tuples 
    from subtractions are removed from the list.
    '''
    for connection_type in list:
        list[connection_type] |= subtractions[connection_type]

if __name__ == "__main__":
    '''
      For testing.
    '''
    if connectionType('/chatter,std_msgs/String,http://snorriheim:35403/') == ConnectionEnum.topic:
        print "topic"
    else:
        print "not topic"
    if connectionType('/add_two_ints,rosrpc://snorriheim:59822,http://snorriheim:34035/') == ConnectionEnum.service:
        print "service"
    else:
        print "not service"
    
    print connectionTypeString('/chatter,std_msgs/String,http://snorriheim:35403/')
    print connectionTypeString('/add_two_ints,rosrpc://snorriheim:59822,http://snorriheim:34035/')
    print connectionTypeString('/add_two_ints,snorriheim:59822,snorriheim:34035/')
    
    dudette="dudette"
    raise Exception("dude %s"%dudette)
