#!/usr/bin/env python
#       
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_multimaster/master/multimaster_client/rocon_gateway_sync/LICENSE 
#

import redis
import threading
import roslib; roslib.load_manifest('rocon_gateway_sync')
import rospy
import re

from .exceptions import GatewayError, ConnectionTypeError
from .utils import Connection, connectionTypeString

###############################################################################
# Classes
###############################################################################

class RedisSubscriberThread(threading.Thread):
    '''
      Tunes into the redis channels that have been subscribed to and
      calls the apropriate callbacks.
      
      Note, the callbacks are from GatewaySync's processUpdate method.
    '''
    def __init__(self,pubsub,callback):
        threading.Thread.__init__(self)
        self.pubsub = pubsub
        self.callback = callback
            
    def run(self):
        rospy.logdebug("Gateway : listening to the subscribed redis channels.")
        for r in self.pubsub.listen():
            if r['type'] != 'unsubscribe' and r['type'] != 'subscribe':
                self.callback(r['data'])
   
##############################################################################
# Hub Manager - Redis Implementation
##############################################################################

class Hub(object):

    pool = None
    server = None
    pubsub = None
    callback = None

    def __init__(self,pubsubcallback,name):
        self._name = name # used to generate the unique name key later
        self.callback = pubsubcallback
        self.redis_keys = {}
        #self.redis_keys['name'] = '' # it's a unique id generated later when connecting
        self.redis_keys['index'] = self.createKey('hub:index') # used for uniquely id'ing the gateway (client)
        self.redis_keys['gatewaylist'] = self.createKey('gatewaylist')
        self.redis_channels = {}
        self.redis_channels['update_topic'] = self.createKey('update')
        
    ##########################################################################
    # Hub
    ##########################################################################

    def connect(self,ip,portarg):
        try:
            self.pool = redis.ConnectionPool(host=ip,port=portarg,db=0)
            self.server = redis.Redis(connection_pool=self.pool)
            rospy.logdebug("Gateway : connected to the hub's redis server.")
            self.pubsub = self.server.pubsub()
        except ConnectionError as e:
            rospy.logerror("Gateway : failed connection to the hub's redis server.")
            raise

    def listGateways(self):
        '''
          Return a list of the gateways (name list, not redis keys).
          e.g. ['gateway32','pirate33']
        '''
        gateway_keys = self.getMembers(self.redis_keys['gatewaylist'])
        gateway_list = []
        for gateway in gateway_keys:
            gateway_list.append(self.baseName(key))
        return gateway_list

    def listPublicInterfaces(self):
        '''
          Return all the 'remote' public interfaces connnected to the hub.
        '''
        public_interfaces = {}
        gateway_keys = self.getMembers(self.redis_keys['gatewaylist'])
        for gateway_key in gateway_keys:
            gateway = self.baseName(gateway_key)
            public_interfaces[gateway] = {}
            # get public topic list of this master
            key = gateway_key +":topic"
            public_interfaces[gateway]['topic'] = self.getMembers(key)

            # get public service list of this master
            key = gateway_key +":service"
            public_interfaces[gateway]['service'] = self.getMembers(key)
        return public_interfaces
        
    ##########################################################################
    # Gateway Connection
    ##########################################################################

    def registerGateway(self):
        unique_num = self.server.incr(self.redis_keys['index'])
        self.redis_keys['name'] = self.createKey(self._name+str(unique_num))
        self.server.sadd(self.redis_keys['gatewaylist'],self.redis_keys['name'])
        self.pubsub.subscribe(self.redis_channels['update_topic'])
        self.pubsub.subscribe(self.redis_keys['name'])
        self.subThread = RedisSubscriberThread(self.pubsub,self.callback)
        self.subThread.start()

        #return self.baseName(self.redis_keys['name'])
        return self.redis_keys['name']

    def unregisterGateway(self,client_key):
        try:
            pipe = self.server.pipeline()
            topiclist = client_key +":topic"
            self.server.delete(topiclist)
            srvlist = client_key +":service"
            self.server.delete(srvlist)
            self.server.srem(self.redis_keys['gatewaylist'],client_key)
            pipe.execute()
            self.pubsub.unsubscribe()
        except Exception as e:
            rospy.logerr("Gateway : error unregistering gateway from the hub (need better error handling here).")
            return False
        rospy.loginfo("Gateway : unregistering gateway from the hub.")
        return True

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
        '''
        identifier = connectionTypeString(connection)
        if identifier not in ['topic', 'service']:  # action not yet implemented
            raise ConnectionTypeError("trying to advertise an invalid connection type on the hub [%s]"%connection)
        key = self.redis_keys['name']+":"+identifier
        self.addMembers(key,connection)
    
    def unadvertise(self, connection):
        '''
          Removes a topic, service or action from the public interface.
          Compare with 'advertise'.
        '''
        identifier = connectionTypeString(connection)
        if identifier not in ['topic', 'service']:  # action not yet implemented
            raise ConnectionTypeError("trying to unadvertise an invalid connection type on the hub.")
        key = self.redis_keys['name']+":"+identifier
        self.removeMembers(key,connection)
    

    ##########################################################################
    # Gateway-Gateway Communications
    ##########################################################################

    def broadcastTopicUpdate(self,msg):
        self.server.publish(self.redis_channels['update_topic'],msg)

    ##########################################################################
    # Redis Api
    ##########################################################################

    def getMembers(self,key):
        return self.server.smembers(key)

    def addMembers(self,key,topic):
        try:
            pipe = self.server.pipeline()
            pipe.sadd(key,topic)
            pipe.execute()
        except:
            print "Error : addMembers"
            return False
        return True

    def removeMembers(self,key,string):
        try:
            pipe = self.server.pipeline()
            pipe.srem(key,string)
            pipe.execute()
        except:
            print "Error : removeMembers"
            return False
        return True

    def sendMessage(self,channel,msg):
        self.server.publish(channel,msg)


    ##########################################################################
    # Redis Key Manipulation Functions - not class related, should push out
    ##########################################################################
    
    def createKey(self,key):
        '''
          Root the specified redis key name in our pseudo redis database.
        '''
        if re.match('rocon:',key): # checks if leading rocon: is found
            return key
        else:
            return 'rocon:'+key
    
    def extractKey(self,key):
        '''
          Extract the specified redis key name from our pseudo redis database.
        '''
        if re.match('rocon:',key): # checks if leading rocon: is found
            re.sub(r'rocon:','',key)
        else:
            return key
    
    def baseName(self,key):
        '''
          Extract the base name (i.e. last value) from the key.
          e.g. rocon:key:pirate24 -> pirate24
        '''
        return key.split(':')[-1]
