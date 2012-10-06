#!/usr/bin/env python
#       
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_multimaster/master/multimaster_client/rocon_gateway/LICENSE 
#

##############################################################################
# Imports
##############################################################################

import json
import collections
from gateway_comms.msg import Connection as ConnectionMsg

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

class Connection(ConnectionMsg):
    '''
      Hashable wrapper around the connection message 
    '''
    def __init__(self, type, name, node, uri, service_api = None, topic_type = None):
        self.type = type
        self.name = name
        self.node = node
        self.uri = uri
        if not service_api:
            service_api = ''
        self.service_api = service_api
        if not topic_type:
            topic_type = ''
        self.topic_type = topic_type
    
    # Need these for hashable containers (like sets), ugh!
    def __eq__(self, other):
        if isinstance(other, self.__class__):
            return self.__dict__ == other.__dict__
        else:
            return False
    
    def __ne__(self, other):
        return not self.__eq__(other)
    
    def __hash__(self):
        return hash(self.type) ^ hash(self.name) ^ hash(self.node) ^ hash(self.uri) ^ hash(self.service_api) ^ hash(self.topic_type)
    
    def __str__(self):
        if self.type == ConnectionMsg.SERVICE:
            return '{%s, name: %s, node: %s, service_api: %s, node_uri: %s}'%(self.type,self.name,self.node,self.service_api,self.uri)
        else:
            return '{%s, name: %s, node: %s, topic_type: %s, node_uri: %s}'%(self.type,self.name,self.node,self.topic_type,self.uri)

    def __repr__(self):
        return self.__str__()

    # def serializeJson(self):
    #     data = [self.type,self.name,self.uri,self.service_api,self.topic_type]
    #     return serialize(data)

    # def deserializeJson(self, string):
    #     data = deserialize(string)
    #     self.type = data[0]
    #     self.name = data[1]
    #     self.uri = data[2]
    #     self.service_api = data[3]
    #     self.topic_type = data[4]

def getConnectionTypes():
    connection_types = []
    connection_types.append(ConnectionMsg.PUBLISHER);
    connection_types.append(ConnectionMsg.SUBSCRIBER);
    connection_types.append(ConnectionMsg.SERVICE);
    connection_types.append(ConnectionMsg.ACTION_SERVER);
    connection_types.append(ConnectionMsg.ACTION_CLIENT);
    return connection_types

##########################################################################
# Json serialization/deserialization Functions
##########################################################################

def convert(data):
    '''
      Convert unicode to standard string (Not sure how necessary this is)
      http://stackoverflow.com/questions/1254454/fastest-way-to-convert-a-dicts-keys-values-from-unicode-to-str
    '''
    if isinstance(data, unicode):
        return str(data)
    elif isinstance(data, collections.Mapping):
        return dict(map(convert, data.iteritems()))
    elif isinstance(data, collections.Iterable):
        return type(data)(map(convert, data))
    else:
        return data

def serialize(data):
    return json.dumps(data)

def deserialize(str_msg):
    return convert(json.loads(str_msg))
