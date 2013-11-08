#!/usr/bin/env python
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
import rocon_utilities
import rocon_hub_client
from rocon_hub_client import hub_api
from rocon_hub_client.exceptions import HubConnectionLostError, \
              HubNameNotFoundError, HubNotFoundError


# local imports
from .exceptions import GatewayUnavailableError


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
        except redis.exceptions.ConnectionError as unused_e:
            # If we ever start using socket timeouts, we need to distinguish between
            # timeouts here and actual conenction errors. Unfortunately we only have
            # strings to do that:
            #    "Error while reading from socket: ('timed out',)"
            #    "Socket closed on remote end":
            self._hub_connection_lost_hook()

##############################################################################
# Hub
##############################################################################


class GatewayHub(rocon_hub_client.Hub):

    def __init__(self, ip, port, whitelist, blacklist):
        '''
          @param remote_gateway_request_callbacks : to handle redis responses
          @type list of function pointers (back to GatewaySync class

          @param ip : redis server ip
          @param port : redis server port

          @raise HubNameNotFoundError, HubNotFoundError
        '''
        try:
            super(GatewayHub, self).__init__(ip, port, whitelist, blacklist)  # can just do super() in python3
        except HubNotFoundError:
            raise
        except HubNameNotFoundError:
            raise
        self._hub_connection_lost_gateway_hook = None
        self._firewall = 0

        # Setting up some basic parameters in-case we use this API without registering a gateway
        self._redis_keys['gatewaylist'] = hub_api.create_rocon_hub_key('gatewaylist')
        self._unique_gateway_name = ''

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

          @raise HubConnectionLostError if for some reason, the redis server has become unavailable.
        '''
        if not self._redis_server:
            raise HubConnectionLostError()
        self._unique_gateway_name = unique_gateway_name
        self._redis_keys['gateway'] = hub_api.create_rocon_key(unique_gateway_name)
        self._redis_keys['firewall'] = hub_api.create_rocon_gateway_key(unique_gateway_name, 'firewall')
        self._firewall = 1 if firewall else 0
        self._remote_gateway_request_callbacks = remote_gateway_request_callbacks
        self._hub_connection_lost_gateway_hook = hub_connection_lost_gateway_hook
        if not self._redis_server.sadd(self._redis_keys['gatewaylist'], self._redis_keys['gateway']):
            # should never get here - unique should be unique
            pass
        unused_ret = self._redis_server.sadd(self._redis_keys['gatewaylist'], self._redis_keys['gateway'])
        self._redis_server.set(self._redis_keys['firewall'], self._firewall)
        # I think we just used this for debugging, but we might want to hide it in future (it's the ros master hostname/ip)
        self._redis_keys['ip'] = hub_api.create_rocon_gateway_key(unique_gateway_name, 'ip')
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
            self._unregister_named_gateway(self._redis_keys['gateway'])
            self._redis_channels = {}
        except (redis.exceptions.ConnectionError, redis.exceptions.ResponseError):
            # usually just means the hub has gone down just before us or is in the
            # middle of doing so let it die nice and peacefully
            # rospy.logwarn("Gateway : problem unregistering from the hub (likely that hub shutdown before the gateway).")
            pass
        # should we not also shut down self.remote_gatew
        rospy.loginfo("Gateway : unregistered from the hub [%s]" % self.name)

    def _unregister_named_gateway(self, gateway_key):
        '''
          Remove all gateway info for given gateway key from the hub.
        '''
        try:
            gateway_keys = self._redis_server.keys(gateway_key + ":*")
            pipe = self._redis_server.pipeline()
            pipe.delete(*gateway_keys)
            pipe.srem(self._redis_keys['gatewaylist'], gateway_key)
            pipe.execute()
        except (redis.exceptions.ConnectionError, redis.exceptions.ResponseError):
            pass

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
        firewall = self._redis_server.get(hub_api.create_rocon_gateway_key(gateway, 'firewall'))
        ip = self._redis_server.get(hub_api.create_rocon_gateway_key(gateway, 'ip'))
        if firewall is None:
            return None  # equivalent to saying no gateway of this id found
        else:
            remote_gateway = gateway_msgs.RemoteGateway()
            remote_gateway.name = gateway
            remote_gateway.ip = ip
            remote_gateway.firewall = True if int(firewall) else False
            remote_gateway.public_interface = []
            encoded_advertisements = self._redis_server.smembers(hub_api.create_rocon_gateway_key(gateway, 'advertisements'))
            for encoded_advertisement in encoded_advertisements:
                advertisement = utils.deserialize_connection(encoded_advertisement)
                remote_gateway.public_interface.append(advertisement.rule)
            remote_gateway.flipped_interface = []
            encoded_flips = self._redis_server.smembers(hub_api.create_rocon_gateway_key(gateway, 'flips'))
            for encoded_flip in encoded_flips:
                [target_gateway, name, connection_type, node] = utils.deserialize(encoded_flip)
                remote_rule = gateway_msgs.RemoteRule(target_gateway, gateway_msgs.Rule(connection_type, name, node))
                remote_gateway.flipped_interface.append(remote_rule)
            remote_gateway.pulled_interface = []
            encoded_pulls = self._redis_server.smembers(hub_api.create_rocon_gateway_key(gateway, 'pulls'))
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
                if hub_api.key_base_name(gateway) != self._unique_gateway_name:
                    gateways.append(hub_api.key_base_name(gateway))
        except redis.ConnectionError as unused_e:
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
          Equivalent to get_connection_state, but generates it from the public
          interface of a remote gateway

          @param remote_gateway : hash name for a remote gateway
          @type str
          @return dictionary of remote advertisements
          @rtype dictionary of connection type keyed connection values
       '''
        connections = utils.create_empty_connection_type_dictionary()
        key = hub_api.create_rocon_gateway_key(remote_gateway, 'advertisements')
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
        firewall = self._redis_server.get(hub_api.create_rocon_gateway_key(gateway, 'firewall'))
        if firewall is not None:
            return True if int(firewall) else False
        else:
            raise GatewayUnavailableError

    def get_local_advertisements(self):
        '''
          Retrieves the local list of advertisements from the hub. This
          gets used to sync across multiple hubs.

          @return dictionary of remote advertisements
          @rtype dictionary of connection type keyed connection values
       '''
        connections = utils.create_empty_connection_type_dictionary()
        key = hub_api.create_rocon_gateway_key(self._unique_gateway_name, 'advertisements')
        public_interface = self._redis_server.smembers(key)
        for connection_str in public_interface:
            connection = utils.deserialize_connection(connection_str)
            connections[connection.rule.type].append(connection)
        return connections

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
        key = hub_api.create_rocon_gateway_key(self._unique_gateway_name, 'advertisements')
        msg_str = utils.serialize_connection(connection)
        self._redis_server.sadd(key, msg_str)

    def unadvertise(self, connection):
        '''
          Removes a topic, service or action from the public interface.

          @param connection: representation of a connection (topic, service, action)
          @type  connection: str
          @raise .exceptions.ConnectionTypeError: if connectionarg is invalid.
        '''
        key = hub_api.create_rocon_gateway_key(self._unique_gateway_name, 'advertisements')
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
        key = hub_api.create_rocon_gateway_key(self._unique_gateway_name, 'flips')
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
        key = hub_api.create_rocon_gateway_key(self._unique_gateway_name, 'flips')
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
        key = hub_api.create_rocon_gateway_key(self._unique_gateway_name, 'pulls')
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
        key = hub_api.create_rocon_gateway_key(self._unique_gateway_name, 'pulls')
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
        source = hub_api.key_base_name(self._redis_keys['gateway'])
        cmd = utils.serialize_connection_request('flip', source, connection)
        try:
            self._redis_server.publish(hub_api.create_rocon_key(remote_gateway), cmd)
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
        source = hub_api.key_base_name(self._redis_keys['gateway'])
        cmd = utils.serialize_rule_request('unflip', source, rule)
        try:
            self._redis_server.publish(hub_api.create_rocon_key(remote_gateway), cmd)
        except Exception as unused_e:
            return False
        return True
