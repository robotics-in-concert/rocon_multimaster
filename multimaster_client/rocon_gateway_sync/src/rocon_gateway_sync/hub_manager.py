#!/usr/bin/env python
#       
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_multimaster/master/multimaster_client/rocon_gateway_sync/LICENSE 
# Copyright (c) 2012, Yujin Robot, Daniel Stonier
#

import redis
import threading
import roslib; roslib.load_manifest('rocon_gateway_sync')
import rospy

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

class HubManager(object):

    pool = None
    server = None
    pubsub = None
    callback = None

    def __init__(self,pubsubcallback,name):
        self.name = name
        self.callback = pubsubcallback
        self.redis_keys = {}
        self.redis_keys['index'] = 'rocon:hub:index' # used for uniquely id'ing the gateway (client)
        self.redis_keys['gatewaylist'] = 'rocon:gatewaylist'
        self.redis_channels = {}
        self.redis_channels['update_topic'] = 'rocon:update'
        
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

    def gatewayList(self):
        return self.redis_manager.getMembers(self.redis_keys['gatewaylist'])

    ##########################################################################
    # Gateway Connection
    ##########################################################################

    def registerGateway(self):
        unique_num = self.server.incr(self.redis_keys['index'])
        client_key = 'rocon:'+self.name+str(unique_num)
        self.server.sadd(self.redis_keys['gatewaylist'],client_key)
        self.pubsub.subscribe(self.redis_channels['update_topic'])
        self.pubsub.subscribe(client_key)
        self.subThread = RedisSubscriberThread(self.pubsub,self.callback)
        self.subThread.start()

        return client_key

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
            print "Error : unregistering client"
            return False
        print "Client Unregistered"
        return True

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

