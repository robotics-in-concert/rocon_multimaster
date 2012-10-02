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
# Enums
##############################################################################

def enum(*sequential, **named):
    enums = dict(zip(sequential, range(len(sequential))), **named)
    return type('Enum', (), enums)

Connection = enum('topic','service','action', 'invalid')

##############################################################################
# Connection type handlers
##############################################################################

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
