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
import StringIO
from gateway_comms.msg import Connection
import yaml

##############################################################################
# Constants
##############################################################################

connection_types = frozenset([Connection.PUBLISHER, Connection.SUBSCRIBER, Connection.SERVICE, Connection.ACTION_CLIENT, Connection.ACTION_SERVER])

##############################################################################
# Registration
##############################################################################

class Registration():
    '''
      An object that represents a connection registered with the local 
      master (or about to be registered). This has all the gory detail
      for the connection. It includes the gateway name it originated 
      from as well as master registration information.
      
       - remote_gateway
       - remote_connection.name (the remote connection name)
       - remote_node            (the remote node)
       - type                   (one of Connection.PUBLISHER, etc)
       - type_info              (msg type for pubsub or service api for services)
       - xmlrpc_uri             (the xmlrpc node uri for the connection)
       - local_node             (the local anonymously generated node name)
    '''
    def __init__(self, remote_gateway, remote_name, remote_node, connection_type, type_info, xmlrpc_uri, local_node = None):
        '''
          @param type_info : either topic_type (pubsub), service api (service) or ??? (action)
          @type string  
        '''
        self.remote_gateway = remote_gateway
        self.remote_name = remote_name
        self.remote_node = remote_node
        self.connection_type = connection_type
        self.type_info = type_info
        self.xmlrpc_uri = xmlrpc_uri
        self.local_node = local_node
    
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

##########################################################################
# serialization/deserialization Functions
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

def serializeRosMsg(msg):
    buffer = StringIO.StringIO()
    msg.serialize(buffer)
    msg_str = buffer.getvalue()
    buffer.close()
    return msg_str

##########################################################################
# Other Utilities
##########################################################################

def formatRule(rule):
    return '{type: %s, name/regex: %s, node name: %s}'%(rule.connection.type,rule.connection.name,rule.connection.node)

def formatConnection(connection):
    if connection.type == connection.SERVICE: 
        return '{%s, name: %s, node: %s, uri: %s, service_api: %s}'%(connection.type,connection.name,connection.node,connection.uri,connection.service_api)
    else:
        return '{%s, name: %s, node: %s, uri: %s, topic_type: %s}'%(connection.type,connection.name,connection.node,connection.uri,connection.topic_type)

def createEmptyConnectionTypeDictionary():
    '''
      Used to initialise a dictionary with connection type keys 
      and empty lists. 
    '''
    dic = {}
    for type in connection_types:
        dic[type] = []
    return dic
