#!/usr/bin/env pythonupdate
#       
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_multimaster/master/rocon_gateway/LICENSE 
#

import redis
import threading
import roslib; roslib.load_manifest('rocon_gateway')
import rospy
import re
import utils
import gateway_comms.msg
from .exceptions import UnavailableGatewayError

###############################################################################
# Utility Functions
###############################################################################

def createKey(key):
    '''
      Root the specified redis key name in our pseudo redis database.
    '''
    if re.match('rocon:',key): # checks if leading rocon: is foundupdate
        return key
    else:
        return 'rocon:'+key

def createHubKey(key):
    '''
      Root the specified redis key name in our pseudo redis database under
      the hub namespace
    '''
    if re.match('rocon:hub:',key): # checks if leading rocon: is foundupdate
        return key
    else:
        return 'rocon:hub:'+key

def createGatewayKey(unique_gateway_name, key):
    '''
      Root the specified redis key name in our pseudo redis database under
      the gateway namespace.
      
      @note : currently does no checking of the incoming keys
    '''
    return 'rocon:'+ unique_gateway_name + ":" + key

def extractKey(key):
    '''
      Extract the specified redis key name from our pseudo redis database.
    '''
    if re.match('rocon:',key): # checks if leading rocon: is found
        return re.sub(r'rocon:','',key)
    else:
        return key

def keyBaseName(key):
    '''
      Extract the base name (i.e. last value) from the key.
      e.g. rocon:key:pirate24 -> pirate24
    '''
    return key.split(':')[-1]

def resolveHub(ip, port):
    '''
      Pings the hub for identification. We currently use this to check
      against the gateway whitelist/blacklists to determine if a rule
      should proceed or not.
      
      @return string - hub name
    '''
    r = redis.Redis(host=ip,port=port)
    return r.get("rocon:hub:name") # perhaps should store all key names somewhere central

###############################################################################
# Redis Callback Handler
##############################################################################

class RedisListenerThread(threading.Thread):
    '''
      Tunes into the redis channels that have been subscribed to and
      calls the apropriate callbacks.
    '''
    def __init__(self,redis_pubsub_server,remote_gateway_request_callbacks):
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
        for r in self._redis_pubsub_server.listen():
            if r['type'] != 'unsubscribe' and r['type'] != 'subscribe':
                command, source, contents = utils.deserializeRequest(r['data'])
                rospy.logdebug("Gateway : redis listener received a channel publication from %s : [%s]"%(source,command))
                if command == 'flip':
                    registration = utils.Registration(utils.getConnectionFromList(contents), source)
                    self._remote_gateway_request_callbacks['flip'](registration)
                elif command == 'unflip':
                    self._remote_gateway_request_callbacks['unflip'](utils.getRuleFromList(contents), source)
                else:
                    rospy.logerr("Gateway : received an unknown command from the hub.")

##############################################################################
# Hub Manager - Redis Implementation
##############################################################################

class Hub(object):

    pool = None
    server = None
    pubsub = None
    callback = None

    def __init__(self,remote_gateway_request_callbacks,gateway_name,firewall):
        '''
          @param remote_gateway_request_callbacks : to handle redis responses
          @type list of function pointers (back to GatewaySync class
          
          @param gateway_name : recommended name for this gateway
          @type string
          
          @param firewall : whether this gateway blocks flips or not (we publish this info)
          @type Bool
        '''
        self.name = '' # the hub name
        self._gateway_name = gateway_name # used to generate the unique name key later
        self._unique_gateway_name = '' # set when gateway is registered
        self._firewall = 1 if firewall else 0
        self._remote_gateway_request_callbacks = remote_gateway_request_callbacks
        self._redis_keys = {}
        #self._redis_keys['gateway'] = '' # it's a unique id set when gateway is registered
        #self._redis_keys['firewall'] = '' # it's also generated later when gateway is registered
        self._redis_keys['index'] = createHubKey('index') # used for uniquely id'ing the gateway (client)
        self._redis_keys['gatewaylist'] = createHubKey('gatewaylist')
        self._redis_channels = {}
        self._redis_pubsub_server = None
        
    ##########################################################################
    # Hub Connections
    ##########################################################################

    def connect(self,ip,portarg):
        try:
            self.pool = redis.ConnectionPool(host=ip,port=portarg,db=0)
            self.server = redis.Redis(connection_pool=self.pool)
            rospy.logdebug("Gateway : connected to the hub's redis server.")
            self._redis_pubsub_server = self.server.pubsub()
        except redis.exceptions.ConnectionError as e:
            rospy.logerror("Gateway : failed rule to the hub's redis server.")
            raise


    def registerGateway(self):
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
        gateways = self.listRemoteGatewayNames()
        if self._gateway_name in gateways:
            unique_num = self.server.incr(self._redis_keys['index'])
            self._unique_gateway_name = self._gateway_name+str(unique_num)
        else:
            self._unique_gateway_name = self._gateway_name
        self._redis_keys['gateway'] = createKey(self._unique_gateway_name)
        self._redis_keys['firewall'] = createGatewayKey(self._unique_gateway_name,'firewall')
        self.server.sadd(self._redis_keys['gatewaylist'],self._redis_keys['gateway'])
        self.server.set(self._redis_keys['firewall'], self._firewall)
        self._redis_channels['gateway'] = self._redis_keys['gateway']
        self._redis_pubsub_server.subscribe(self._redis_channels['gateway'])
        self.remote_gateway_listener_thread = RedisListenerThread(self._redis_pubsub_server, self._remote_gateway_request_callbacks)
        self.remote_gateway_listener_thread.start()
        self.name = keyBaseName(self.server.get("rocon:hub:name"))
        return keyBaseName(self._redis_keys['gateway'])

    def unregisterGateway(self):
        '''
          Remove all gateway info from the hub.
          
          @return: success or failure of the operation
          @rtype: bool
        '''
        try:
            self._redis_pubsub_server.unsubscribe()
            gateway_keys = self.server.keys(self._redis_keys['gateway']+":*")
            pipe = self.server.pipeline()
            pipe.delete(*gateway_keys)
            pipe.srem(self._redis_keys['gatewaylist'],self._redis_keys['gateway'])
            pipe.execute()
            self._redis_channels = {}
            self._unique_gateway_name = ''
            self.name = ''
        except Exception as e:
            rospy.logerr("Gateway : error unregistering gateway from the hub (need better error handling here).")
            return False
        rospy.loginfo("Gateway : unregistering gateway from the hub.")
        return True

    ##########################################################################
    # Hub Data Retrieval
    ##########################################################################

    def remoteGatewayInfo(self, gateway):
        '''
          Return remote gateway information for the specified gateway string id.
          
          @param gateways : gateway id string to search for
          @type string
          @return remote gateway information
          @rtype gateway_comms.msg.RemotGateway or None
        '''
        firewall = self.server.get(createGatewayKey(gateway,'firewall'))
        if firewall is None:
            return None # equivalent to saying no gateway of this id found
        else:
            remote_gateway = gateway_comms.msg.RemoteGateway()
            remote_gateway.name = gateway
            remote_gateway.firewall = True if int(firewall) else False
            encoded_connections = self.server.smembers(createGatewayKey(gateway,'connection'))
            remote_gateway.public_interface = []
            for encoded_connection in encoded_connections:
                connection = utils.deserializeConnection(encoded_connection)
                remote_gateway.public_interface.append(connection.rule)
            return remote_gateway 

    def listRemoteGatewayNames(self):
        '''
          Return a list of the gateways (name list, not redis keys).
          e.g. ['gateway32','pirate33']
        '''
        gateway_keys = self.server.smembers(self._redis_keys['gatewaylist']) 
        gateways = []
        for gateway in gateway_keys:
            if keyBaseName(gateway) != self._unique_gateway_name:
                gateways.append(keyBaseName(gateway))
        return gateways

    def getRemoteConnectionState(self, gateway):
        '''
          Equivalent to getConnectionState, but generates it from the public
          interface of a foreign gateway
       '''
        connections = utils.createEmptyConnectionTypeDictionary()
        gateway_key = createKey(gateway)
        key = gateway_key +":connection"
        public_interface = self.server.smembers(key)
        for connection_str in public_interface:
            connection = utils.deserializeConnection(connection_str)
            connections[connection.rule.type].append(connection)
        return connections
    
    def getRemoteGatewayFirewallFlag(self, gateway):
        '''
          Returns the value of the remote gateway's firewall (flip)
          flag.
          
          @param gateway : gateway string id
          @param string
          
          @return state of the flag
          @rtype Bool
          
          @raise UnavailableGatewayError when specified gateway is not on the hub
        '''
        firewall = self.server.get(createGatewayKey(gateway,'firewall'))
        if firewall is not None:
            return True if int(firewall) else False
        else:
            raise UnavailableGatewayError
                
    ##########################################################################
    # Public Interface
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
          @raise .exceptions.ConnectionTypeError: if connectionarg is invalid.
        '''
        key = self._redis_keys['gateway']+":connection"
        msg_str = utils.serializeConnection(connection)
        self.server.sadd(key,msg_str)
    
    def unadvertise(self, connection):
        '''
          Removes a topic, service or action from the public interface.
          
          @param connection: representation of a connection (topic, service, action)
          @type  connection: str
          @raise .exceptions.ConnectionTypeError: if connectionarg is invalid.
        '''
        key = self._redis_keys['gateway']+":connection"
        msg_str = utils.serializeConnection(connection)
        self.server.srem(key,msg_str)

    ##########################################################################
    # Gateway-Gateway Communications
    ##########################################################################
    
    def sendFlipRequest(self, gateway, connection):
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
          @type RemoteRule
          
          @param type_info : topic type (e.g. std_msgs/String)
          @param str
          
          @param xmlrpc_uri : the node uri
          @param str
        '''
        source = keyBaseName(self._redis_keys['gateway'])
        cmd = utils.serializeConnectionRequest('flip', source, connection)
        try:
            self.server.publish(createKey(gateway),cmd)
        except Exception as e:
            return False
        return True

    def sendUnflipRequest(self, gateway, rule):
        source = keyBaseName(self._redis_keys['gateway'])
        cmd = utils.serializeRuleRequest('unflip', source, rule)
        try:
            self.server.publish(createKey(gateway),cmd)
        except Exception as e:
            return False
        return True

    ##########################################################################
    # Redis Converters - convert variosu types to redis readable strings
    ##########################################################################
    

