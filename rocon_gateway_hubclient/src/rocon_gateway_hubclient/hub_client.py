import rospy
import threading
import roslib; roslib.load_manifest('rocon_gateway_hubclient')
import redis
from .hub_connector import HubConnector

class HubClient(HubConnector):

    # HubConnector provide 
    #    connect(hub_uri)

    pool = None
    server = None
    pubsub= None
    callback = None
    namespace= ''
    param = {}

    def __init__(self,whitelist,blacklist,is_zeroconf,namespace,name,firewall,callbacks):
        super(HubClient,self).__init__(whitelist,blacklist,is_zeroconf,self._connectToHub)

        self._namespace = namespace
        self._name = name
        self._unique_name = ''
        self._firewall = 1 if firewall else 0
        self._callbacks = callbacks

        self._redis_keys = {}
#        self._redis_keys['index'] = createHubKey('index')
        self._redis_channels = {}
        self._redis_pubsub_server= None

    def _connectToHub(self,ip,portarg):
        try:
            self.pool = redis.ConnectionPool(host=ip,port=portarg,db=0)
            self.server = redis.Redis(connection_pool=self.pool)
            rospy.logdebug("HubClient : connected to the hub's redist server.")
            self._redis_pubsub_server = self.server.pubsub()
        except redis.exceptions.ConnectionError as e:
            rospy.logerror("HubClient : failed rule to the hub's redis server.")
            return False
            raise
        return True

    def registerKey(self,key,element):
        key = self.namespace + ":"+self.key
        element = self.namespace + ":" +self.element
        
        if self.server.sadd(key,element):
            return True
        else:
            raise Exception("Same concertmaster name exist")
            return False

    def unregisterKey(self,key,element):
        key = self.namespace + ":"+self.key
        element = self.namespace + ":" +self.element
        
        self.server.srem(key,element)
