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
import gateway_msgs.msg as gateway_msgs


# local imports
import rocon_utilities
from .exceptions import GatewayUnavailableError, HubConnectionLostError, \
              HubNameNotFoundError, HubNotFoundError, HubNotConnectedError

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


def ping_hub(ip, port):
    '''
      Pings the hub for identification. This is currently used
      by the hub discovery module.

      @return Bool
    '''
    try:
        r = redis.Redis(host=ip, port=port)
        name = r.get("rocon:hub:name")
    except redis.exceptions.ConnectionError:
        return False
    if name is None:  # returns None if the server was there, but the key was not found.
        return False
    return True

###############################################################################
# Redis Callback Handler
##############################################################################


class RedisListenerThread(threading.Thread):
    '''
      Tunes into the redis channels that have been subscribed to and
      calls the apropriate callbacks.
    '''
    def __init__(self, redis_pubsub_server, remote_gateway_request_callbacks, hub_connection_lost_hook):
        threading.Thread.__init__(self)
        self._redis_pubsub_server = redis_pubsub_server
        self._remote_gateway_request_callbacks = remote_gateway_request_callbacks
        self._hub_connection_lost_hook = hub_connection_lost_hook

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
            # This is a generator so it will keep spitting out (alt. to having a while loop here)
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
            self._hub_connection_lost_hook()

##############################################################################
# Hub
##############################################################################


class Hub(object):

    def __init__(self, ip, port):
        '''
          @param remote_gateway_request_callbacks : to handle redis responses
          @type list of function pointers (back to GatewaySync class

          @param ip : redis server ip
          @param port : redis server port

          @raise HubNameNotFoundError, HubNotFoundError
        '''
        self.uri = str(ip) + ":" + str(port)
        try:
            self.pool = redis.ConnectionPool(host=ip, port=port, db=0)
            self._redis_server = redis.Redis(connection_pool=self.pool)
            self._redis_pubsub_server = self._redis_server.pubsub()
            hub_key_name = self._redis_server.get("rocon:hub:name")
            # Be careful, hub_name is None, it means the redis server is
            # found but hub_name not yet set or not set at all.
            if not hub_key_name:
                self._redis_server = None
                raise HubNameNotFoundError()
            else:
                self.name = key_base_name(hub_key_name)  # perhaps should store all key names somewhere central
                rospy.logdebug("Gateway : resolved hub name [%s].", self.name)
        except redis.exceptions.ConnectionError:
            self._redis_server = None
            raise HubNotFoundError()
        self._redis_keys = {}
        self._redis_channels = {}
        self._firewall = 0
        self._hub_connection_lost_gateway_hook = None

    ##########################################################################
    # Hub Connections
    ##########################################################################

    def register_gateway(self, firewall, unique_gateway_name, remote_gateway_request_callbacks, hub_connection_lost_gateway_hook, gateway_ip):
        '''
          Register a gateway with the hub.

          @param firewall
          @param unique_gateway_name
          @param remote_gateway_request_callbacks
          @param hub_connection_lost_hook : used to trigger Gateway.disengage_hub(hub) on lost hub connections in redis pubsub listener thread.
          @gateway_ip

          @raise HubNotConnectedError if for some reason, this class is not in
               a valid state (i.e. not connected to the redis server)
        '''
        if not self._redis_server:
            raise HubNotConnectedError()
        self._unique_gateway_name = unique_gateway_name
        self._redis_keys['gateway'] = create_key(unique_gateway_name)
        self._redis_keys['firewall'] = create_gateway_key(unique_gateway_name, 'firewall')
        self._firewall = 1 if firewall else 0
        self._redis_keys['gatewaylist'] = create_hub_key('gatewaylist')
        self._remote_gateway_request_callbacks = remote_gateway_request_callbacks
        self._hub_connection_lost_gateway_hook = hub_connection_lost_gateway_hook
        if not self._redis_server.sadd(self._redis_keys['gatewaylist'], self._redis_keys['gateway']):
            # should never get here - unique should be unique
            pass
        unused_ret = self._redis_server.sadd(self._redis_keys['gatewaylist'], self._redis_keys['gateway'])
        self._redis_server.set(self._redis_keys['firewall'], self._firewall)
        # I think we just used this for debugging, but we might want to hide it in future (it's the ros master hostname/ip)
        self._redis_keys['ip'] = create_gateway_key(unique_gateway_name, 'ip')
        self._redis_server.set(self._redis_keys['ip'], gateway_ip)
        self._redis_channels['gateway'] = self._redis_keys['gateway']
        self._redis_pubsub_server.subscribe(self._redis_channels['gateway'])
        self.remote_gateway_listener_thread = RedisListenerThread(self._redis_pubsub_server, self._remote_gateway_request_callbacks, self._hub_connection_lost_hook)
        self.remote_gateway_listener_thread.start()

    def _hub_connection_lost_hook(self):
        '''
          This gets triggered by the redis pubsub listener when the hub connection is lost.
          The trigger is passed to the gateway who needs to remove the hub.
        '''
        rospy.loginfo("Gateway : lost connection to the hub [%s][%s]" % (self.name, self.uri))
        if self._hub_connection_lost_gateway_hook is not None:
            self._hub_connection_lost_gateway_hook(self)

    def unregister_gateway(self):
        '''
          Remove all gateway info from the hub.

          @return: success or failure of the operation
          @rtype: bool
        '''
        try:
            self._redis_pubsub_server.unsubscribe()
            gateway_keys = self._redis_server.keys(self._redis_keys['gateway'] + ":*")
            pipe = self._redis_server.pipeline()
            pipe.delete(*gateway_keys)
            pipe.srem(self._redis_keys['gatewaylist'], self._redis_keys['gateway'])
            pipe.execute()
            self._redis_channels = {}
        except redis.exceptions.ConnectionError:
            # usually just means the hub has gone down just before us, let it go quietly
            # rospy.logwarn("Gateway : problem unregistering from the hub (likely that hub shutdown before the gateway).")
            pass
        # should we not also shut down self.remote_gatew
        rospy.loginfo("Gateway : unregistered from the hub [%s]" % self.name)

    ##########################################################################
    # Hub Data Retrieval
    ##########################################################################

    def remote_gateway_info(self, gateway):
        '''
          Return remote gateway information for the specified gateway string id.

          @param gateways : gateway id string to search for
          @type string
          @return remote gateway information
          @rtype gateway_msgs.RemotGateway or None
        '''
        firewall = self._redis_server.get(create_gateway_key(gateway, 'firewall'))
        ip = self._redis_server.get(create_gateway_key(gateway, 'ip'))
        if firewall is None:
            return None  # equivalent to saying no gateway of this id found
        else:
            remote_gateway = gateway_msgs.RemoteGateway()
            remote_gateway.name = gateway
            remote_gateway.ip = ip
            remote_gateway.firewall = True if int(firewall) else False
            remote_gateway.public_interface = []
            encoded_advertisements = self._redis_server.smembers(create_gateway_key(gateway, 'advertisements'))
            for encoded_advertisement in encoded_advertisements:
                advertisement = utils.deserialize_connection(encoded_advertisement)
                remote_gateway.public_interface.append(advertisement.rule)
            remote_gateway.flipped_interface = []
            encoded_flips = self._redis_server.smembers(create_gateway_key(gateway, 'flips'))
            for encoded_flip in encoded_flips:
                [target_gateway, name, connection_type, node] = utils.deserialize(encoded_flip)
                remote_rule = gateway_msgs.RemoteRule(target_gateway, gateway_msgs.Rule(connection_type, name, node))
                remote_gateway.flipped_interface.append(remote_rule)
            remote_gateway.pulled_interface = []
            encoded_pulls = self._redis_server.smembers(create_gateway_key(gateway, 'pulls'))
            for encoded_pull in encoded_pulls:
                [target_gateway, name, connection_type, node] = utils.deserialize(encoded_pull)
                remote_rule = gateway_msgs.RemoteRule(target_gateway, gateway_msgs.Rule(connection_type, name, node))
                remote_gateway.pulled_interface.append(remote_rule)
            return remote_gateway

    def list_remote_gateway_names(self):
        '''
          Return a list of the gateways (name list, not redis keys).
          e.g. ['gateway32adcda32','pirate21fasdf']. If not connected, just
          returns an empty list.
        '''
        if not self._redis_server:
            rospy.logerr("Gateway : cannot retrieve remote gateway names [%s][%s]." % (self.name, self.uri))
            return []
        gateways = []
        try:
            gateway_keys = self._redis_server.smembers(self._redis_keys['gatewaylist'])
            for gateway in gateway_keys:
                if key_base_name(gateway) != self._unique_gateway_name:
                    gateways.append(key_base_name(gateway))
        except redis.ConnectionError as unused_e:
            #rospy.logwarn("Gateway : lost connection to the hub [list_remote_gateway_names][%s][%s]" % (self.name, self.uri))
            pass
        return gateways

    def matches_remote_gateway_name(self, gateway):
        '''
          Use this when gateway can be a regular expression and
          we need to check it off against list_remote_gateway_names()

          @return a list of matches (higher level decides on action for duplicates).
          @rtype list[str] : list of remote gateway names.
        '''
        matches = []
        try:
            for remote_gateway in self.list_remote_gateway_names():
                if re.match(gateway, remote_gateway):
                    matches.append(remote_gateway)
        except HubConnectionLostError:
            raise
        return matches

    def matches_remote_gateway_basename(self, gateway):
        '''
          Use this when gateway can be a regular expression and
          we need to check it off against list_remote_gateway_names()
        '''
        weak_matches = []
        try:
            for remote_gateway in self.list_remote_gateway_names():
                if re.match(gateway, rocon_utilities.gateway_basename(remote_gateway)):
                    weak_matches.append(remote_gateway)
        except HubConnectionLostError:
            raise
        return weak_matches

    def get_remote_connection_state(self, remote_gateway):
        '''
          Equivalent to getConnectionState, but generates it from the public
          interface of a remote gateway

          @param remote_gateway : hash name for a remote gateway
          @type str
          @return dictionary of remote advertisements
          @rtype dictionary of connection type keyed connection values
       '''
        connections = utils.create_empty_connection_type_dictionary()
        key = create_gateway_key(remote_gateway, 'advertisements')
        public_interface = self._redis_server.smembers(key)
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

          @raise GatewayUnavailableError when specified gateway is not on the hub
        '''
        firewall = self._redis_server.get(create_gateway_key(gateway, 'firewall'))
        if firewall is not None:
            return True if int(firewall) else False
        else:
            raise GatewayUnavailableError

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
        self._redis_server.sadd(key, msg_str)

    def unadvertise(self, connection):
        '''
          Removes a topic, service or action from the public interface.

          @param connection: representation of a connection (topic, service, action)
          @type  connection: str
          @raise .exceptions.ConnectionTypeError: if connectionarg is invalid.
        '''
        key = create_gateway_key(self._unique_gateway_name, 'advertisements')
        msg_str = utils.serialize_connection(connection)
        self._redis_server.srem(key, msg_str)

    def post_flip_details(self, gateway, name, connection_type, node):
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
        serialized_data = utils.serialize([gateway, name, connection_type, node])
        self._redis_server.sadd(key, serialized_data)

    def remove_flip_details(self, gateway, name, connection_type, node):
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
        serialized_data = utils.serialize([gateway, name, connection_type, node])
        self._redis_server.srem(key, serialized_data)

    def post_pull_details(self, gateway, name, connection_type, node):
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
        serialized_data = utils.serialize([gateway, name, connection_type, node])
        self._redis_server.sadd(key, serialized_data)

    def remove_pull_details(self, gateway, name, connection_type, node):
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
        serialized_data = utils.serialize([gateway, name, connection_type, node])
        self._redis_server.srem(key, serialized_data)

    ##########################################################################
    # Gateway-Gateway Communications
    ##########################################################################

    def send_flip_request(self, remote_gateway, connection):
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
          @type gateway_msgs.RemoteRule

          @param type_info : topic type (e.g. std_msgs/String)
          @param str

          @param xmlrpc_uri : the node uri
          @param str
        '''
        source = key_base_name(self._redis_keys['gateway'])
        cmd = utils.serialize_connection_request('flip', source, connection)
        try:
            self._redis_server.publish(create_key(remote_gateway), cmd)
        except Exception as unused_e:
            return False
        return True

    def send_unflip_request(self, remote_gateway, rule):
        if rule.type == gateway_msgs.ConnectionType.ACTION_CLIENT:
            action_name = rule.name
            rule.type = gateway_msgs.ConnectionType.PUBLISHER
            rule.name = action_name + "/goal"
            self._send_unflip_request(remote_gateway, rule)
            rule.name = action_name + "/cancel"
            self._send_unflip_request(remote_gateway, rule)
            rule.type = gateway_msgs.ConnectionType.SUBSCRIBER
            rule.name = action_name + "/feedback"
            self._send_unflip_request(remote_gateway, rule)
            rule.name = action_name + "/status"
            self._send_unflip_request(remote_gateway, rule)
            rule.name = action_name + "/result"
            self._send_unflip_request(remote_gateway, rule)
        elif rule.type == gateway_msgs.ConnectionType.ACTION_SERVER:
            action_name = rule.name
            rule.type = gateway_msgs.ConnectionType.SUBSCRIBER
            rule.name = action_name + "/goal"
            self._send_unflip_request(remote_gateway, rule)
            rule.name = action_name + "/cancel"
            self._send_unflip_request(remote_gateway, rule)
            rule.type = gateway_msgs.ConnectionType.PUBLISHER
            rule.name = action_name + "/feedback"
            self._send_unflip_request(remote_gateway, rule)
            rule.name = action_name + "/status"
            self._send_unflip_request(remote_gateway, rule)
            rule.name = action_name + "/result"
            self._send_unflip_request(remote_gateway, rule)
        else:
            self._send_unflip_request(remote_gateway, rule)

    def _send_unflip_request(self, remote_gateway, rule):
        source = key_base_name(self._redis_keys['gateway'])
        cmd = utils.serialize_rule_request('unflip', source, rule)
        try:
            self._redis_server.publish(create_key(remote_gateway), cmd)
        except Exception as unused_e:
            return False
        return True
