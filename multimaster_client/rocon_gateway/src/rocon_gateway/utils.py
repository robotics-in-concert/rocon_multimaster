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
#from gateway_comms.msg import Connection as Connection
from gateway_comms.msg import Connection

##############################################################################
# Constants
##############################################################################

connection_types = frozenset([Connection.PUBLISHER, Connection.SUBSCRIBER, Connection.SERVICE, Connection.ACTION_CLIENT, Connection.ACTION_SERVER])

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

def getConnectionTypes():
    connection_types = []
    connection_types.append(Connection.PUBLISHER);
    connection_types.append(Connection.SUBSCRIBER);
    connection_types.append(Connection.SERVICE);
    connection_types.append(Connection.ACTION_SERVER);
    connection_types.append(Connection.ACTION_CLIENT);
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

##########################################################################
# Other Utilities
##########################################################################

def createEmptyConnectionTypeDictionary():
    '''
      Used to initialise a dictionary with connection type keys 
      and empty lists. 
    '''
    dic = {}
    for type in connection_types:
        dic[type] = []
    return dic
