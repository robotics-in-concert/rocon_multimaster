#!/usr/bin/env pythonupdate
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_multimaster/master/rocon_gateway/LICENSE
#
###############################################################################
# Imports
###############################################################################

import redis
import threading
import rospy
import re
import utils
import gateway_msgs.msg
import rosgraph
from .exceptions import UnavailableGatewayError, HubConnectionLostError

###############################################################################
# Utility Functions
###############################################################################


def create_key(key):
    '''
      Root the specified redis key name in our pseudo redis database.
    '''
    if re.match('rocon:', key):  # checks if leading rocon: is foundupdate
        return key
    else:
        return 'rocon:' + key


def create_hub_key(key):
    '''
      Root the specified redis key name in our pseudo redis database under
      the hub namespace
    '''
    if re.match('rocon:hub:', key):  # checks if leading rocon: is foundupdate
        return key
    else:
        return 'rocon:hub:' + key


def create_gateway_key(unique_gateway_name, key):
    '''
      Root the specified redis key name in our pseudo redis database under
      the gateway namespace.

      @note : currently does no checking of the incoming keys
    '''
    return 'rocon:' + unique_gateway_name + ":" + key


def extract_key(key):
    '''
      Extract the specified redis key name from our pseudo redis database.
    '''
    if re.match('rocon:', key):  # checks if leading rocon: is found
        return re.sub(r'rocon:', '', key)
    else:
        return key


def key_base_name(key):
    '''
      Extract the base name (i.e. last value) from the key.
      e.g. rocon:key:pirate24 -> pirate24
    '''
    return key.split(':')[-1]


def resolve_hub(ip, port):
    '''
      Pings the hub for identification. We currently use this to check
      against the gateway whitelist/blacklists to determine if a rule
      should proceed or not.

      @return string - hub name
    '''
    r = redis.Redis(host=ip, port=port)
    return r.get("rocon:hub:name")  # perhaps should store all key names somewhere central

###############################################################################
# Redis Callback Handler
##############################################################################


class RedisListenerThread(threading.Thread):
    '''
      Tunes into the redis channels that have been subscribed to and
      calls the apropriate callbacks.
    '''
    def __init__(self, redis_pubsub_server, remote_gateway_request_callbacks):
        threading.Thread.__init__(self)
        self._redis_pubsub_server = redis_pubsub_server
        self._remote_gateway_request_callbacks = remote_gateway_request_callbacks

    def run(self):
        '''
          Used as a callback for incoming requests on redis pubsub channels.

          The received argument is a list of strings for 'flip':

            - [0] - command : this one is 'flip'
            - [1] - remote_gateway : the name of the gateway that is flipping to us
            - [2] - remote_name
            - [3] - remote_node
            - [4] - type : one of ConnectionType.PUBLISHER etc
            - [5] - type_info : a ros format type (e.g. std_msgs/String or service api)
            - [6] - xmlrpc_uri : the xmlrpc node uri

          The command 'unflip' is the same, not including args 5 and 6.
        '''
        try:
            for r in self._redis_pubsub_server.listen():
                if r['type'] != 'unsubscribe' and r['type'] != 'subscribe':
                    command, source, contents = utils.deserialize_request(r['data'])
                    rospy.logdebug("Gateway : redis listener received a channel publication from %s : [%s]" % (source, command))
                    if command == 'flip':
                        registration = utils.Registration(utils.get_connection_from_list(contents), source)
                        self._remote_gateway_request_callbacks['flip'](registration)
                    elif command == 'unflip':
                        self._remote_gateway_request_callbacks['unflip'](utils.get_rule_from_list(contents), source)
                    else:
                        rospy.logerr("Gateway : received an unknown command from the hub.")
        except redis.exceptions.ConnectionError:
            rospy.logwarn("Gateway : lost connection to the hub (probably shut down)")

##############################################################################
# Hub Manager - Redis Implementation
##############################################################################


class Hub(object):

    pool = None
    server = None
    pubsub = None
    callback = None

    def __init__(self, remote_gateway_request_callbacks, gateway_name, firewall):
        '''
          @param remote_gateway_request_callbacks : to handle redis responses
          @type list of function pointers (back to GatewaySync class

          @param gateway_name : recommended name for this gateway
          @type string

          @param firewall : whether this gateway blocks flips or not (we publish this info)
          @type Bool
        '''
        self.name = ''  # the hub name
        self.uri = ''
        self._gateway_name = gateway_name  # used to generate the unique name key later
        self._unique_gateway_name = ''  # set when gateway is registered
        self._firewall = 1 if firewall else 0
        self._remote_gateway_request_callbacks = remote_gateway_request_callbacks
        self._redis_keys = {}
        #self._redis_keys['gateway'] = '' # it's a unique id set when gateway is registered
        #self._redis_keys['firewall'] = '' # it's also generated later when gateway is registered
        self._redis_keys['index'] = create_hub_key('index')  # used for uniquely id'ing the gateway (client)
        self._redis_keys['gatewaylist'] = create_hub_key('gatewaylist')
        self._redis_channels = {}
        self._redis_pubsub_server = None

    ##########################################################################
    # Hub Connections
    ##########################################################################

    def connect(self, ip, portarg):
        try:
            self.pool = redis.ConnectionPool(host=ip, port=portarg, db=0)
            self.server = redis.Redis(connection_pool=self.pool)
            rospy.logdebug("Gateway : connected to the hub's redis server.")
            self._redis_pubsub_server = self.server.pubsub()
            self.uri = str(ip) + ":" + str(portarg)
        except redis.exceptions.ConnectionError as unused_e:
            rospy.logerror("Gateway : failed rule to the hub's redis server.")
            raise

    def register_gateway(self, ip):
        '''
          Register a gateway with the hub. Note that you must have already
          connected before calling this function.

          On registration, the hub will provide a unique identifier number
          which will be appended to the suggested name of this gateway to
          ensure a unique id string and key for this gateway.

          @return: success or failure of the operation
          @rtype: bool

          @todo - maybe merge with 'connect', or at the least check if it
          is actually connected here first.
        '''
        if self.server.sadd(self._redis_keys['gatewaylist'], create_key(self._gateway_name)):
            self._unique_gateway_name = self._gateway_name
            self._redis_keys['gateway'] = create_key(self._unique_gateway_name)
        else:
            unique_num = self.server.incr(self._redis_keys['index'])
            self._unique_gateway_name = self._gateway_name + str(unique_num)
            self._redis_keys['gateway'] = create_key(self._unique_gateway_name)
            unused_ret = self.server.sadd(self._redis_keys['gatewaylist'], self._redis_keys['gateway'])
        unused_ret = self.server.sadd(self._redis_keys['gatewaylist'], self._redis_keys['gateway'])
        self._redis_keys['firewall'] = create_gateway_key(self._unique_gateway_name, 'firewall')
        self.server.set(self._redis_keys['firewall'], self._firewall)
        self._redis_keys['ip'] = create_gateway_key(self._unique_gateway_name, 'ip')
        self.server.set(self._redis_keys['ip'], ip)
        self._redis_channels['gateway'] = self._redis_keys['gateway']
        self._redis_pubsub_server.subscribe(self._redis_channels['gateway'])
        self.remote_gateway_listener_thread = RedisListenerThread(self._redis_pubsub_server, self._remote_gateway_request_callbacks)
        self.remote_gateway_listener_thread.start()
        self.name = key_base_name(self.server.get("rocon:hub:name"))
        return key_base_name(self._redis_keys['gateway'])

    def unregister_gateway(self):
        '''
          Remove all gateway info from the hub.

          @return: success or failure of the operation
          @rtype: bool
        '''
        try:
            self._redis_pubsub_server.unsubscribe()
            gateway_keys = self.server.keys(self._redis_keys['gateway'] + ":*")
            pipe = self.server.pipeline()
            pipe.delete(*gateway_keys)
            pipe.srem(self._redis_keys['gatewaylist'], self._redis_keys['gateway'])
            pipe.execute()
            self._redis_channels = {}
            self._unique_gateway_name = ''
            self.name = ''
        except redis.exceptions.ConnectionError:
            # usually just means the hub has gone down just before us, let it go quietly
            # rospy.logwarn("Gateway : problem unregistering from the hub (likely that hub shutdown before the gateway).")
            return False
        rospy.loginfo("Gateway : unregistering gateway from the hub.")
        return True

    ##########################################################################
    # Hub Data Retrieval
    ##########################################################################

    def remote_gateway_info(self, gateway):
        '''
          Return remote gateway information for the specified gateway string id.

          @param gateways : gateway id string to search for
          @type string
          @return remote gateway information
          @rtype gateway_msgs.msg.RemotGateway or None
        '''
        firewall = self.server.get(create_gateway_key(gateway, 'firewall'))
        ip = self.server.get(create_gateway_key(gateway, 'ip'))
        if firewall is None:
            return None  # equivalent to saying no gateway of this id found
        else:
            remote_gateway = gateway_msgs.msg.RemoteGateway()
            remote_gateway.name = gateway
            remote_gateway.ip = ip
            remote_gateway.firewall = True if int(firewall) else False
            remote_gateway.public_interface = []
            encoded_advertisements = self.server.smembers(create_gateway_key(gateway, 'advertisements'))
            for encoded_advertisement in encoded_advertisements:
                advertisement = utils.deserialize_connection(encoded_advertisement)
                remote_gateway.public_interface.append(advertisement.rule)
            remote_gateway.flipped_interface = []
            encoded_flips = self.server.smembers(create_gateway_key(gateway, 'flips'))
            for encoded_flip in encoded_flips:
                [target_gateway, name, type, node] = utils.deserialize(encoded_flip)
                remote_rule = gateway_msgs.msg.RemoteRule(target_gateway, gateway_msgs.msg.Rule(name, type, node))
                remote_gateway.flipped_interface.append(remote_rule)
            remote_gateway.pulled_interface = []
            encoded_pulls = self.server.smembers(create_gateway_key(gateway, 'pulls'))
            for encoded_pull in encoded_pulls:
                [target_gateway, name, type, node] = utils.deserialize(encoded_pull)
                remote_rule = gateway_msgs.msg.RemoteRule(target_gateway, gateway_msgs.msg.Rule(name, type, node))
                remote_gateway.pulled_interface.append(remote_rule)
            return remote_gateway

    def list_remote_gateway_names(self):
        '''
          Return a list of the gateways (name list, not redis keys).
          e.g. ['gateway32','pirate33']
        '''
        if not self.server:
            rospy.logerr("Gateway : cannot retrive remote gateway names [not connected to a hub]")
            return []
        try:
            gateway_keys = self.server.smembers(self._redis_keys['gatewaylist'])
        except redis.ConnectionError as unused_e:
            rospy.logwarn("Concert Client : lost connection to the hub (probably shut down)")
            raise HubConnectionLostError()
        gateways = []
        for gateway in gateway_keys:
            if key_base_name(gateway) != self._unique_gateway_name:
                gateways.append(key_base_name(gateway))
        return gateways

    def matches_remote_gateway_name(self, gateway):
        '''
          Use this when gateway can be a regular expression and
          we need to check it off against list_remote_gateway_names()
        '''
        try:
            for remote_gateway in self.list_remote_gateway_names():
                if re.match(gateway, remote_gateway):
                    return True
        except HubConnectionLostError:
            raise
        return False

    def get_remote_connection_state(self, gateway):
        '''
          Equivalent to getConnectionState, but generates it from the public
          interface of a foreign gateway
       '''
        connections = utils.createEmptyConnectionTypeDictionary()
        key = create_gateway_key(gateway, 'advertisements')
        public_interface = self.server.smembers(key)
        for connection_str in public_interface:
            connection = utils.deserialize_connection(connection_str)
            connections[connection.rule.type].append(connection)
        return connections

    def get_remote_gateway_firewall_flag(self, gateway):
        '''
          Returns the value of the remote gateway's firewall (flip)
          flag.

          @param gateway : gateway string id
          @param string

          @return state of the flag
          @rtype Bool

          @raise UnavailableGatewayError when specified gateway is not on the hub
        '''
        firewall = self.server.get(create_gateway_key(gateway, 'firewall'))
        if firewall is not None:
            return True if int(firewall) else False
        else:
            raise UnavailableGatewayError

    ##########################################################################
    # Posting Information to the Hub
    ##########################################################################

    def advertise(self, connection):
        '''
          Places a topic, service or action on the public interface. On the
          redis server, this representation will always be:

           - topic : a triple { name, type, xmlrpc node uri }
           - service : a triple { name, rosrpc uri, xmlrpc node uri }
           - action : ???

          @param connection: representation of a connection (topic, service, action)
          @type  connection: str
          @raise .exceptions.ConnectionTypeError: if connection arg is invalid.
        '''
        key = create_gateway_key(self._unique_gateway_name, 'advertisements')
        msg_str = utils.serialize_connection(connection)
        self.server.sadd(key, msg_str)

    def unadvertise(self, connection):
        '''
          Removes a topic, service or action from the public interface.

          @param connection: representation of a connection (topic, service, action)
          @type  connection: str
          @raise .exceptions.ConnectionTypeError: if connectionarg is invalid.
        '''
        key = create_gateway_key(self._unique_gateway_name,'advertisements')
        msg_str = utils.serialize_connection(connection)
        self.server.srem(key,msg_str)

    def post_flip_details(self, gateway, name, type, node):
        '''
          Post flip details to the redis server. This has no actual functionality,
          it is just useful for debugging with the remote_gateway_info service.

          @param gateway : the target of the flip
          @type string
          @param name : the name of the connection
          @type string
          @param type : the type of the connection (one of ConnectionType.xxx
          @type string
          @param node : the node name it was pulled from
          @type string
        '''
        key = create_gateway_key(self._unique_gateway_name, 'flips')
        serialized_data = utils.serialize([gateway, name, type, node])
        self.server.sadd(key, serialized_data)

    def remove_flip_details(self, gateway, name, type, node):
        '''
          Post flip details to the redis server. This has no actual functionality,
          it is just useful for debugging with the remote_gateway_info service.

          @param gateway : the target of the flip
          @type string
          @param name : the name of the connection
          @type string
          @param type : the type of the connection (one of ConnectionType.xxx
          @type string
          @param node : the node name it was pulled from
          @type string
        '''
        key = create_gateway_key(self._unique_gateway_name, 'flips')
        serialized_data = utils.serialize([gateway, name, type, node])
        self.server.srem(key, serialized_data)

    def post_pull_details(self, gateway, name, type, node):
        '''
          Post pull details to the hub. This has no actual functionality,
          it is just useful for debugging with the remote_gateway_info service.

          @param gateway : the gateway it is pulling from
          @type string
          @param name : the name of the connection
          @type string
          @param type : the type of the connection (one of ConnectionType.xxx
          @type string
          @param node : the node name it was pulled from
          @type string
        '''
        key = create_gateway_key(self._unique_gateway_name, 'pulls')
        serialized_data = utils.serialize([gateway, name, type, node])
        self.server.sadd(key, serialized_data)

    def remove_pull_details(self, gateway, name, type, node):
        '''
          Post pull details to the hub. This has no actual functionality,
          it is just useful for debugging with the remote_gateway_info service.

          @param gateway : the gateway it was pulling from
          @type string
          @param name : the name of the connection
          @type string
          @param type : the type of the connection (one of ConnectionType.xxx
          @type string
          @param node : the node name it was pulled from
          @type string
        '''
        key = create_gateway_key(self._unique_gateway_name, 'pulls')
        serialized_data = utils.serialize([gateway, name, type, node])
        self.server.srem(key, serialized_data)

    ##########################################################################
    # Gateway-Gateway Communications
    ##########################################################################

    def send_flip_request(self, gateway, connection):
        '''
          Sends a message to the remote gateway via redis pubsub channel. This is called from the
          watcher thread, when a flip rule gets activated.

           - redis channel name: rocon:<remote_gateway_name>
           - data : list of [ command, gateway, rule type, type, xmlrpc_uri ]
            - [0] - command       : in this case 'flip'
            - [1] - gateway       : the name of this gateway, i.e. the flipper
            - [2] - name          : local name
            - [3] - node          : local node name
            - [4] - connection_type : one of ConnectionType.PUBLISHER etc
            - [5] - type_info     : a ros format type (e.g. std_msgs/String or service api)
            - [6] - xmlrpc_uri    : the xmlrpc node uri

          @param command : string command name - either 'flip' or 'unflip'
          @type str

          @param flip_rule : the flip to send
          @type gateway_msgs.msg.RemoteRule

          @param type_info : topic type (e.g. std_msgs/String)
          @param str

          @param xmlrpc_uri : the node uri
          @param str
        '''
        source = key_base_name(self._redis_keys['gateway'])
        cmd = utils.serialize_connection_request('flip', source, connection)
        try:
            self.server.publish(create_key(gateway),cmd)
        except Exception as e:
            return False
        return True

    def send_unflip_request(self, gateway, rule):
        if rule.type == gateway_msgs.msg.ConnectionType.ACTION_CLIENT:
            action_name = rule.name
            rule.type = gateway_msgs.msg.ConnectionType.PUBLISHER
            rule.name = action_name + "/goal"
            self._send_unflip_request(gateway, rule)
            rule.name = action_name + "/cancel"
            self._send_unflip_request(gateway, rule)
            rule.type = gateway_msgs.msg.ConnectionType.SUBSCRIBER
            rule.name = action_name + "/feedback"
            self._send_unflip_request(gateway, rule)
            rule.name = action_name + "/status"
            self._send_unflip_request(gateway, rule)
            rule.name = action_name + "/result"
            self._send_unflip_request(gateway, rule)
        elif rule.type == gateway_msgs.msg.ConnectionType.ACTION_SERVER:
            action_name = rule.name
            rule.type = gateway_msgs.msg.ConnectionType.SUBSCRIBER
            rule.name = action_name + "/goal"
            self._send_unflip_request(gateway, rule)
            rule.name = action_name + "/cancel"
            self._send_unflip_request(gateway, rule)
            rule.type = gateway_msgs.msg.ConnectionType.PUBLISHER
            rule.name = action_name + "/feedback"
            self._send_unflip_request(gateway, rule)
            rule.name = action_name + "/status"
            self._send_unflip_request(gateway, rule)
            rule.name = action_name + "/result"
            self._send_unflip_request(gateway, rule)
        else:
            self._send_unflip_request(gateway, rule)

    def _send_unflip_request(self, gateway, rule):
        source = key_base_name(self._redis_keys['gateway'])
        cmd = utils.serialize_rule_request('unflip', source, rule)
        try:
            self.server.publish(create_key(gateway), cmd)
        except Exception as e:
            return False
        return True
