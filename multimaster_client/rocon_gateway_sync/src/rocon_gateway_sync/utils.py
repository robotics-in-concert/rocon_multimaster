#!/usr/bin/env python
#       
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_multimaster/master/multimaster_client/rocon_gateway_sync/LICENSE 
#

##############################################################################
# Imports
##############################################################################

import re

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

Connection = enum('topic','service','action', 'invalid')
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
        return Connection.invalid
    if re.match('rosrpc',components[1]):
        return Connection.service
    if re.match('http://',components[2]):
        return Connection.topic
    # action not implemented yet
    return Connection.invalid

def connectionTypeString(connection):
    '''
      Checks a connection string representation to determine if it is a 
      topic, service or action. Returns a string representation of the type.
      
                { "topic", "service", "action", "invalid" }
        
      @param connection : the string representation for a connection
      @return string : the string representation for the connection type. 
    '''
    return ConnectionStrings[connectionType(connection)] 
        

if __name__ == "__main__":
    '''
      For testing.
    '''
    if connectionType('/chatter,std_msgs/String,http://snorriheim:35403/') == Connection.topic:
        print "topic"
    else:
        print "not topic"
    if connectionType('/add_two_ints,rosrpc://snorriheim:59822,http://snorriheim:34035/') == Connection.service:
        print "service"
    else:
        print "not service"
    
    print connectionTypeString('/chatter,std_msgs/String,http://snorriheim:35403/')
    print connectionTypeString('/add_two_ints,rosrpc://snorriheim:59822,http://snorriheim:34035/')
    print connectionTypeString('/add_two_ints,snorriheim:59822,snorriheim:34035/')
    
    dudette="dudette"
    raise Exception("dude %s"%dudette)
