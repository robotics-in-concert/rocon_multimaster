#!/usr/bin/env python       
# Software License Agreement (BSD License)
#
# Copyright (c) 2012, Yujin Robot, Daniel Stonier, Jihoon Lee
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Yujin Robot nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import redis
import threading

class SubThread(threading.Thread):
  def __init__(self,pubsub,callback):
    threading.Thread.__init__(self)
    self.pubsub = pubsub
    self.callback = callback
            
  def run(self):
    print "Start Listening"
   
    for r in self.pubsub.listen():
      if r['type'] != 'unsubscribe':
        self.callback(r['data'])

    print "Done listening"
    return


class RedisManager(object):

  pool = None
  server = None
  pubsub = None
  callback = None

  def __init__(self,pubsubcallback):
    print "Init Redis Manager"
    self.callback = pubsubcallback

  def connect(self,ip,portarg):
    try:
      self.pool = redis.ConnectionPool(host=ip,port=portarg,db=0)
      self.server = redis.Redis(connection_pool=self.pool)
      self.pubsub = self.server.pubsub()
    except ConnectionError as e:
      raise

  def registerClient(self,masterlist,index,update_topic):
    unique_num = self.server.incr(index)
    client_key = 'client'+str(unique_num)
    self.server.sadd(masterlist,client_key)
    self.pubsub.subscribe(update_topic)
    self.subThread = SubThread(self.pubsub,self.callback)
    self.subThread.start()

    return client_key

  def getMembers(self,key):
    return self.server.smembers(key)

  def addMembers(self,key,list):
    try:
      pipe = self.server.pipeline()
    
      for topic in list:
        pipe.sadd(key,topic)
      pipe.execute()
    except:
      print "Error : addMembers"
      return False
  
    return True

  def removeMembers(self,key,list):
    try:
      pipe = self.server.pipeline()
    
      for topic in list:
        pipe.srem(key,topic)
      pipe.execute()

    except:
      print "Error : removeMembers"
      return False

    return True

  def unregisterClient(self,masterlist,client_key):

    try:
      pipe = self.server.pipeline()
      topiclist = client_key +":topic"
      self.server.delete(topiclist)
      srvlist = client_key +":service"
      self.server.delete(srvlist)
      self.server.srem(masterlist,client_key)
      pipe.execute()
      self.pubsub.unsubscribe()
    except Exception as e:
      print "Error : unregistering client"
      return False

    print "Client Unregistered"
  
    return True

  def sendUpdateMessage(self,update_topic,msg):
      self.server.publish(update_topic,msg)

