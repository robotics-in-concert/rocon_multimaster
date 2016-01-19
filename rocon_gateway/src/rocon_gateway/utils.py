#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_multimaster/license/LICENSE
#
##############################################################################
# Imports
##############################################################################

import copy
import cPickle as pickle
import os
import re

import rospy

from Crypto.PublicKey import RSA
import Crypto.Util.number as CUN

import gateway_msgs.msg as gateway_msgs

##############################################################################
# Constants
##############################################################################

# for help in iterating over the set of connection constants
connection_types = frozenset([gateway_msgs.ConnectionType.PUBLISHER, gateway_msgs.ConnectionType.SUBSCRIBER,
                             gateway_msgs.ConnectionType.SERVICE, gateway_msgs.ConnectionType.ACTION_CLIENT, gateway_msgs.ConnectionType.ACTION_SERVER])
connection_types_list = [gateway_msgs.ConnectionType.PUBLISHER, gateway_msgs.ConnectionType.SUBSCRIBER,
                         gateway_msgs.ConnectionType.SERVICE, gateway_msgs.ConnectionType.ACTION_CLIENT, gateway_msgs.ConnectionType.ACTION_SERVER]
connection_type_strings_list = ["publisher", "subscriber", "service", "action_client", "action_server"]
action_types = ['/goal', '/cancel', '/status', '/feedback', '/result']

##############################################################################
# Rule
##############################################################################


class Connection():

    '''
      An object that represents a connection containing all the gory details
      about a connection, allowing a connection to be passed through to a
      foreign gateway

       - rule (gateway_msgs.msg.Rule) (containing type,name,node)
       - type_info              (msg type for pubsub or service api for services)
       - xmlrpc_uri             (the xmlrpc node uri for the connection)
    '''

    def __init__(self, rule, type_msg, type_info, xmlrpc_uri):
        """
        @param type_msg : message type of the topic or service
        @param type_info : either topic_type (pubsub), service api (service) or ??? (action)
        @type string
        """
        self.rule = rule
        self.type_msg = type_msg
        self.type_info = type_info
        self.xmlrpc_uri = xmlrpc_uri

    def __eq__(self, other):
        if isinstance(other, self.__class__):
            return self.__dict__ == other.__dict__
        else:
            return False

    def __ne__(self, other):
        return not self.__eq__(other)

    def __str__(self):
        if self.rule.type == gateway_msgs.ConnectionType.SERVICE:
            return '{type: %s, name: %s, node: %s, uri: %s, service_api: %s, service_type: %s}' % (
                self.rule.type, self.rule.name, self.rule.node, self.xmlrpc_uri, self.type_info, self.type_msg)
        else:
            return '{type: %s, name: %s, node: %s, uri: %s, topic_type: %s}' % (
                self.rule.type, self.rule.name, self.rule.node, self.xmlrpc_uri, self.type_info)

    def __repr__(self):
        return self.__str__()

    def inConnectionList(self, connection_list):
        '''
          Checks to see if this connection has the same rule
          as an item in the given connection_list

          @param connection_list : connection list to trawl.
          @type utils.Connection[]
          @return true if this connection rule matches a connection rule in the list
          @rtype Bool
        '''
        for connection in connection_list:
            if self.hasSameRule(connection):
                return True
        return False

    def hasSameRule(self, connection):
        '''
          Checks for equivalency regardless of type_info and xmlrpc_uri.

          @param connection : connection to compare with
          @type utils.Connection
          @return true if equivalent, false otherwise
          @rtype Bool
        '''
        return (self.rule.name == connection.rule.name and
                self.rule.type == connection.rule.type and
                self.rule.node == connection.rule.node)

##############################################################################
# Registration
##############################################################################


class Registration():

    '''
      An object that represents a connection registered with the local
      master (or about to be registered). This has all the gory detail
      for the connection. It includes the gateway name it originated
      from as well as master registration information.

       - connection             (the remote connection information)
       - remote_gateway         (the remote gateway from where this connection originated)
       - local_node             (the local anonymously generated node name)
    '''

    def __init__(self, connection, remote_gateway, local_node=None):
        '''
          @param connection : string identifier storing the remote connection details (type, name, node)
          @type gateway_msgs.msg.RemoteRule

          @param remote_gateway : string identifier for where this registration game from
          @type string

          @param local_node : the local node that this registration is created under
          @type string
        '''
        self.connection = connection
        self.remote_gateway = remote_gateway
        self.local_node = local_node

    def __eq__(self, other):
        if isinstance(other, self.__class__):
            return self.__dict__ == other.__dict__
        else:
            return False

    def __ne__(self, other):
        return not self.__eq__(other)

    def __str__(self):
        return '[%s]%s' % (self.remote_gateway, format_rule(self.connection.rule))

    def __repr__(self):
        return self.__str__()

##########################################################################
# serialization/deserialization Functions
##########################################################################


# def convert(data):
#     '''
#       Convert unicode to standard string (Not sure how necessary this is)
#       http://stackoverflow.com/questions/1254454/fastest-way-to-convert-a-dicts-keys-values-from-unicode-to-str
#     '''
#     if isinstance(data, unicode):
#         return str(data)
#     elif isinstance(data, collections.Mapping):
#         return dict(map(convert, data.iteritems()))
#     elif isinstance(data, collections.Iterable):
#         return type(data)(map(convert, data))
#     else:
#         return data
#

def serialize(data):
    # return json.dumps(data)
    return pickle.dumps(data)


def deserialize(str_msg):
    # return convert(json.loads(str_msg))
    try:
        deserialized_data = pickle.loads(str_msg)
    except ValueError as e:
        rospy.logwarn("Gateway : error in deserialization [%s]" % e)
        import traceback
        print(traceback.format_exc())
        print("Data : %s" % str_msg)
    return deserialized_data


# TODO :pickle API support directly in Connection object instead of here to make it more visible.
def serialize_connection(connection):
    return serialize([connection.rule.type,
                      connection.rule.name,
                      connection.rule.node,
                      connection.type_msg,
                      connection.type_info,
                      connection.xmlrpc_uri]
                     )


def deserialize_connection(connection_str):
    deserialized_list = deserialize(connection_str)
    rule = gateway_msgs.Rule(deserialized_list[0],
                deserialized_list[1],
                deserialized_list[2]
                )
    return Connection(rule, deserialized_list[3], deserialized_list[4], deserialized_list[5])


def serialize_connection_request(command, source, connection):
    return serialize([command, source,
                      connection.rule.type,
                      connection.rule.name,
                      connection.rule.node,
                      connection.type_msg,
                      connection.type_info,
                      connection.xmlrpc_uri]
                     )


def serialize_rule_request(command, source, rule):
    return serialize([command, source, rule.type, rule.name, rule.node])


def deserialize_request(request_str):
    deserialized_list = deserialize(request_str)
    return deserialized_list[0], deserialized_list[1], deserialized_list[2:]


def get_connection_from_list(connection_argument_list):
    rule = gateway_msgs.Rule(connection_argument_list[0], connection_argument_list[1], connection_argument_list[2])
    return Connection(rule, connection_argument_list[3], connection_argument_list[4], connection_argument_list[5])


def get_rule_from_list(rule_argument_list):
    return gateway_msgs.Rule(rule_argument_list[0], rule_argument_list[1], rule_argument_list[2])

##########################################################################
# Encryption/Decryption
##########################################################################

MAX_PLAINTEXT_LENGTH = 256


def generate_private_public_key():
    key = RSA.generate(8 * MAX_PLAINTEXT_LENGTH)
    public_key = key.publickey()
    return key, public_key


def deserialize_key(serialized_key):
    return RSA.importKey(serialized_key)


def serialize_key(key):
    return key.exportKey()


def encrypt(plaintext, public_key):
    if len(plaintext) > MAX_PLAINTEXT_LENGTH:
        # TODO need to have arbitrary lengths
        raise ValueError('Trying to encrypt text longer than ' + MAX_PLAINTEXT_LENGTH + ' bytes!')
    K = CUN.getRandomNumber(128, os.urandom)  # Not used, legacy compatibility
    ciphertext = public_key.encrypt(plaintext, K)
    return ciphertext[0]
    # return plaintext


def decrypt(ciphertext, key):
    return key.decrypt(ciphertext)
    # return ciphertext


def decrypt_connection(connection, key):
    decrypted_connection = copy.deepcopy(connection)
    decrypted_connection.type_info = decrypt(connection.type_info, key)
    decrypted_connection.xmlrpc_uri = decrypt(connection.xmlrpc_uri, key)
    return decrypted_connection


def encrypt_connection(connection, key):
    encrypted_connection = copy.deepcopy(connection)
    encrypted_connection.type_info = encrypt(connection.type_info, key)
    encrypted_connection.xmlrpc_uri = encrypt(connection.xmlrpc_uri, key)
    return encrypted_connection

##########################################################################
# Regex
##########################################################################


def is_all_pattern(pattern):
    '''
      Convenience function for detecting the 'all' pattern.

      @param pattern : the name rule string for the flip all concept
      @type str
      @return true if matching, false otherwise
      @rtype Bool
    '''
    if pattern == ".*":
        return True
    else:
        return False


##########################################################################
# Formatters
##########################################################################

def format_rule(rule):
    return '[%s][%s][%s]' % (rule.type, rule.name, rule.node)

##########################################################################
# Factories
##########################################################################


def create_empty_connection_type_dictionary():
    '''
      Used to initialise a dictionary with rule type keys
      and empty lists.
    '''
    dic = {}
    for connection_type in connection_types:
        dic[connection_type] = []
    return dic

difflist = lambda l1, l2: [x for x in l1 if x not in l2]  # diff of lists

##########################################################################
# Conversion from Connection Cache Proxy channels (as passed in callback)
# to Gateway connections
##########################################################################


def _get_connections_from_service_chan_dict(ccproxy_channel_dict, connection_type):
    connections = []
    for name, service in ccproxy_channel_dict.iteritems():
        service_name = service.name
        service_type = service.type
        service_uri = service.xmlrpc_uri
        nodes = service.nodes
        for node in nodes:
            connection = Connection(gateway_msgs.Rule(connection_type, service_name, node[0]), service_type, service_uri, node[1])
            connections.append(connection)
    return connections


def _get_connections_from_pub_sub_chan_dict(ccproxy_channel_dict, connection_type):
    connections = []
    for name, topic in ccproxy_channel_dict.iteritems():
        topic_name = topic.name
        topic_type = topic.type
        nodes = topic.nodes
        for node in nodes:
            connection = Connection(gateway_msgs.Rule(connection_type, topic_name, node[0]), topic_type, topic_type, node[1])
            connections.append(connection)
    return connections


def _get_connections_from_action_chan_dict(ccproxy_channel_dict, connection_type):
    connections = []
    for name, action in ccproxy_channel_dict.iteritems():
        action_name = action.name + '/goal'
        goal_topic_type = action.type
        nodes = action.nodes
        for node in nodes:
            connection = Connection(gateway_msgs.Rule(connection_type, action_name, node[0]), goal_topic_type, goal_topic_type, node[1])  # node_uri
            connections.append(connection)
    return connections