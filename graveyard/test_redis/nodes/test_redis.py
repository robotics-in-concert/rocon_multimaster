#!/usr/bin/env python

import roslib; roslib.load_manifest('test_redis')
import rospy
import redis
import rosgraph.masterapi
import threading

class SubThread(threading.Thread):
  def __init__(self,pubsub):
    threading.Thread.__init__(self)
    self.pubsub = pubsub
                  
  def run(self):
    rospy.loginfo("Start Listening")

    for r in self.pubsub.listen():
      rospy.loginfo(str(r))
      rospy.sleep(0.1)
    rospy.loginfo("Done listening")

class Gateway():
  # Redis db configuration
  pool = None 
  server = None
  local_uri = None
  
  master_list = "masterlist"


  def connect(self,hostname,portarg,db,public_topic_list):

    # Create a connection to database
    self._connectToDB()

    pipe = self.server.pipeline()
    # Adding ros master uri to masterlist key
    rospy.loginfo("Adding to ROS Master List")
    self._updateMasterList(pipe)
 
    # store public topics in this uri"
    rospy.loginfo("Advertise public topics")
    self._updatePublicTopic(pipe,public_topic_list)
    pipe.execute()

    # Create Sub Thread
    rospy.loginfo("Start Subscriber Thread")
    self.subThread = SubThread(self.pubsub)
    self.subThread.start()
    
    # Publish a new client message
    self.server.publish("info","New Client")  

    # Subscribe to Channels
    rospy.loginfo("Subscribing to info channel")
    self.pubsub.subscribe("info")

    rospy.loginfo("Done Connecting")

  def _connectToDB(self): 
    self.pool = redis.ConnectionPool(host='localhost', port=6379, db=0)
    rospy.loginfo("Connecting to server")
    self.server = redis.Redis(connection_pool=self.pool)
      
    # instanciate Pubsub Channel
    self.pubsub = self.server.pubsub()

  def _updateMasterList(self,pipe):
    self.local_uri = rosgraph.get_master_uri()
    pipe.sadd("masterlist",self.local_uri)


  def _updatePublicTopic(self,pipe,list):
    topic_storage = self.local_uri + ":topics"
    for topic in list:
      pipe.sadd(topic_storage,topic)


  def spin(self):
    self.connect('localhost',6379,0,['/chatter','/whoola!!'])
    resp = self.server.smembers('masterlist')
    rospy.loginfo(str(resp))
    
    for r in self.pubsub.listen():
      rospy.loginfo(str(r))
    rospy.spin()
    rospy.loginfo("Stopping Thread")
    self.pubsub.unsubscribe()
    rospy.loginfo("done")
    


if __name__ == '__main__':
  try:
    rospy.init_node('test_redis')

    test = Gateway()
    rospy.loginfo('Initialized')
    test.spin()
  except KeyboardInterrupt:
    pass
