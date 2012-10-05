#!/usr/bin/env python
#       
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_multimaster/master/multimaster_client/rocon_gateway/LICENSE 
#

##############################################################################
# Imports
##############################################################################

import socket
import time
import re
import itertools
import copy

import roslib; roslib.load_manifest('rocon_gateway')
import rospy
import rosgraph
from std_msgs.msg import Empty

# Ros Comms
from gateway_comms.msg import Connection
from gateway_comms.srv import AdvertiseResponse
from gateway_comms.srv import AdvertiseAllResponse

# Local imports
from gateway_comms.msg import Connection
from gateway_comms.srv import AdvertiseResponse
from gateway_comms.srv import AdvertiseAllResponse
import utils
from .hub_api import Hub
from .master_api import LocalMaster
from .watcher_thread import WatcherThread
from .exceptions import GatewayError, ConnectionTypeError
from .public_interface import PublicInterface

##############################################################################
# Gateway
##############################################################################

'''
    The roles of GatewaySync is below
    1. communicate with ros master using xml rpc node
    2. communicate with redis server
'''

class GatewaySync(object):
    '''
    The gateway between ros system and redis server
    '''

    def __init__(self, name):
        self.unresolved_name = name # This gets used to build unique names after connection to the hub
        self.unique_name = None # single string value set after hub connection (note: it is not a redis rocon:: rooted key!)
        self.master_uri = None
        self.is_connected = False

        self.public_interface = PublicInterface()
        self.hub = Hub(self.processUpdate, self.unresolved_name)
        self.master = LocalMaster()
        self.master_uri = self.master.getMasterUri()

        # create a thread to clean-up unavailable topics
        self.watcher_thread = WatcherThread(self)

        self._initializeWatchlists()
        self._initializeBlacklists()

        # create a whitelist/blacklist of named topics and services for flipped
        self.flipped_topic_whitelist = dict()
        self.flipped_service_whitelist = dict()
        self.flip_public_topics = set()

        #create a list of flipped triples
        self.flipped_interface_list = dict()

    def connectToHub(self,ip,port):
        try:
            self.hub.connect(ip,port)
            self.unique_name = self.hub.registerGateway()
            self.is_connected = True
        except Exception as e:
            print str(e)
            return False
        return True

    ##########################################################################
    # Public Interface Methods
    ##########################################################################

    def advertise(self,request):
        success = False
        try:
            watchlist = utils.connectionsFromConnectionMsgList(request.watchlist)
            if not request.cancel:
                utils.addToConnectionList(self._public_watchlist, watchlist)
            else:
                utils.removeFromConnectionList(self._public_watchlist, watchlist)
            success = True
        except Exception as e:
            rospy.logerr("Gateway : advertise call error [%s]."%str(e))
        return success, utils.connectionMsgListFromConnections(self._public_watchlist)

    def advertiseAll(self,request):
        success = False
        try:
            blacklist = utils.connectionsFromConnectionMsgList(request.blacklist)
            if not request.cancel:
                self._public_blacklist = copy.deepcopy(self._default_blacklist)
                utils.addToConnectionList(self._public_blacklist, blacklist)
                self._public_whitelist = utils.getAllAllowedConnectionList()
            else:
                self._public_whitelist = utils.getEmptyConnectionList()
            success = True
        except Exception as e:
            rospy.logerr("Gateway : advertise all call error [%s]."%str(e))
        return success, utils.connectionMsgListFromConnections(self._public_blacklist)

    ##########################################################################
    # Legacy advertisement functions (yes, they are 2 days older)
    # These will be used later on to setup individual tuple information on the
    # server
    ##########################################################################

    def advertiseConnection(self,connection_type,connection):
        '''
        Adds a connection (topic/service/action) to the public interface.
        
        - adds to the public interface list
        - adds to the hub so it can be pulled by remote gateways
        
        @param connection_type : type of connection (pub,sub,srv,ac,as)
        @type str
        @param connection : tuple containing connection information
        @type tuple
        '''
        if not self.is_connected:
            rospy.logerr("Gateway : advertise connection call failed [no hub connection].")
            return False
        try:
            if self.public_interface.add(connection_type,connection):
                self.hub.advertise(connection_type,connection)
                rospy.loginfo("Gateway : added connection to the public interface [%s: %s]"%(connection_type, connection))
        except Exception as e: 
            rospy.logerr("Gateway : advertise connection call failed [%s]"%str(e))
            return False
        return True

    def unadvertiseConnection(self,connection_type,connection):
        '''
        Removes a connection (topic/service/action) to the public interface.
        
        - remove the public interface list
        - remove the connection from the hub, the hub announces the removal
        
        @param connection_type : type of connection (pub,sub,srv,ac,as)
        @type str
        @param connection : tuple containing connection information
        @type tuple
        '''
        if not self.is_connected:
            rospy.logerr("Gateway : advertise call failed [no hub connection].")
            return False
        try:
            if self.public_interface.remove(connection_type,connection):
                self.hub.unadvertise(connection_type,connection)
                rospy.loginfo("Gateway : added connection to the public interface [%s: %s]"%(connection_type, connection))
        except Exception as e: 
            rospy.logerr("Gateway : advertiseList call failed [%s]"%str(e))
            return False
        return True

    ##########################################################################
    # Flip Interface Methods
    ##########################################################################

    def flip(self,gateways,list):
        '''
        Flips a connection (topic/service/action) to a foreign gateway.

        @param gateways : list of gateways to flip to (all if empty)
        @param list : list of connection representations (usually stringified triples)
        '''
        if not self.is_connected:
            rospy.logerr("Gateway : flip call failed [no hub connection].")
            return False, []
        if len(gateways) == 0:
            gateways = self.hub.listGateways()
            gateways = [x for x in gateways if x != self.unique_name]
        for gateway in gateways:
            if gateway == self.unique_name:
                rospy.logerr("Gateway : cannot flip to self [%s]"%gateway)
                continue
            rospy.loginfo("Gateway : flipping connections %s to gateway [%s]"%(str(list),gateway))
            self.hub.flip(gateway,list)
        return True, [] 

    def unflip(self,gateways,list):
        '''
        Removes flipped connection (topic/service/action) to a foreign gateway

        @param gateways : list of gateways to flip to (all if empty)
        @param list : list of connection representations (usually stringified triples)
        '''
        if not self.is_connected:
            rospy.logerr("Gateway : unflip call failed [no hub connection].")
            return False, []
        if len(gateways) == 0:
            gateways = self.hub.listGateways()
        for gateway in gateways:
            rospy.loginfo("Gateway : removing flipped connections [%s] to gateway [%s]"%(str(list),gateway))
            self.hub.unflip(gateway,list)
        return True, []

    ##########################################################################
    # Pulling Methods
    ##########################################################################

    def pull(self,list):
        '''
        Registers connections (topic/service/action) on a foreign gateway's
        public interface with the local master.

        @todo - this can probably be almost passed directly back and forth form
        the master api itself.

        @param list : list of connection representations (usually stringified triples)
        @type list of str
        '''
        try:
            for l in list:
                if self.master.register(l):
                    rospy.loginfo("Gateway : adding foreign connection [%s]"%l)
        except Exception as e: 
            rospy.logerr("Gateway : %s"%str(e))
            return False, []
        return True, []

    def unpull(self,list):
        '''
        Unregisters connections (topic/service/action) on a foreign gateway's
        public interface with the local master.
        
        @todo - this can probably be almost passed directly back and forth form
        the master api itself.

        @param list : connection representations (usually stringified triples)
        @type list of str
        '''
        try:
            for l in list:
                if self.master.unregister(l):
                    rospy.loginfo("Gateway : removed foreign connection [%s]"%l)
        except Exception as e: 
            rospy.logerr("Gateway : %s"%str(e))
            return False, []
        return True, []

    ##########################################################################
    # Watchlist/Blacklist modification methods
    ##########################################################################

    def _initializeWatchlists(self):
        '''
        Initializes all watchlists (public/flip/pull) with null sets. This
        function is only used to initialize the vairable names, incase the 
        gateway does not setup the default lists explicityly
        '''
        self._public_watchlist = utils.getEmptyConnectionList()

    def _initializeBlacklists(self):
        '''
        Initializes all blacklists (topics/services/actions) that can never be
        advertised/flipped/pulled. A blacklist supplied in /pull_all, /flip_all,
        /advertise_all will be in addition to this blacklist.
        '''
        self._default_blacklist = utils.getEmptyConnectionList()
        self._public_blacklist = utils.getEmptyConnectionList()
        
    def setDefaultBlacklist(self, blacklist):
        '''
        Sets the default blacklists. This function should be called
        during gateway initialization with blacklists provided through a
        parameter file

        @param blacklists : a pre-formatted blacklists dict, most likely from
        @type dict of sets of tuples
        '''
        self._default_blacklist = blacklist

    def setPublicWatchlist(self, watchlist):
        '''
        Sets the default blacklists. This function should be called
        during gateway initialization with blacklists provided through a
        parameter file

        @param blacklists : a pre-formatted blacklists dict, most likely from
        @type dict of sets of tuples
        '''
        self._public_watchlist = watchlist

    def oldFlipWrapper(self,list):
        num = int(list[0])
        gateways = list[1:num+1]
        flip_list = list[num+1:len(list)]
        return self.flip(gateways,flip_list)

    def oldUnflipWrapper(self,list):
        num = int(list[0])
        gateways = list[1:num+1]
        unflip_list = list[num+1:len(list)]
        return self.unflip(gateways,unflip_list)
       
    def addPublicTopicByName(self,topic):
        list = self.getTopicString([topic])
        return self.advertise(list)

    def addNamedTopics(self, list):
        print "Adding named topics: " + str(list)
        self.public_topic_whitelist.extend(list)
        return True, []

    def getTopicString(self,list):
        l = []
        for topic in list:
            try:
                topicinfo = self.master.getTopicInfo(topic)
            
                # there may exist multiple publisher
                for info in topicinfo:
                    l.append(topic+","+info)
            except:
                print "Error while looking up topic. Perhaps topic does not exist"
        return l

    def removePublicTopicByName(self,topic):
        # remove topics that exist, but are no longer part of the public interface
        list = self.getTopicString([topic])
        return self.unadvertise(list)

    def removeNamedTopics(self, list):
        print "Removing named topics: " + str(list)
        self.public_topic_whitelist[:] = [x for x in self.public_topic_whitelist if x not in list]
        return True, []

    def addPublicServiceByName(self,service):
        list = self.getServiceString([service])
        return self.advertise(list)

    def addNamedServices(self, list):
        print "Adding named services: " + str(list)
        self.public_service_whitelist.extend(list)
        return True, []

    def getServiceString(self,list):
        list_with_node_ip = []
        for service in list:
            #print service
            try:
                srvinfo = self.master.getServiceInfo(service)
                list_with_node_ip.append(service+","+srvinfo)
            except:
                print "Error obtaining service info. Perhaps service does not exist?"
        return list_with_node_ip


    def removePublicServiceByName(self,service):
        # remove available services that should no longer be on the public interface
        list = self.getServiceString([service])
        return self.unadvertise(list)

    def removeNamedServices(self, list):
        print "Removing named services: " + str(list)
        self.public_service_whitelist[:] = [x for x in self.public_service_whitelist if x not in list]
        return True, []

    def addPublicInterfaceByName(self, identifier, name):
        if identifier == "topic":
            self.addPublicTopicByName(name)
        elif identifier == "service":
            self.addPublicServiceByName(name)

    def removePublicInterfaceByName(self,identifier,name):
        if identifier == "topic":
            self.removePublicTopicByName(name)
        elif identifier == "service":
            self.removePublicServiceByName(name)

    def addNamedFlippedTopics(self, list):
        # list[0] # of channel
        # list[1:list[0]] is channels
        # rest of them are fliping topics
        num = int(list[0])
        channels = list[1:num+1]
        topics = list[num+1:len(list)]
        print "Adding named topics to flip: " + str(list)
        for chn in channels:
            if chn not in self.flipped_topic_whitelist:
                self.flipped_topic_whitelist[chn] = set()
            self.flipped_topic_whitelist[chn].update(set(topics))
        return True, []

    def addFlippedTopicByName(self,clients,name):
        topic_triples = self.getTopicString([name])
        for client in clients:
            if client not in self.flipped_interface_list:
                self.flipped_interface_list[client] = set()
            add_topic_triples = [x for x in topic_triples if x not in self.flipped_interface_list[client]]
            self.flipped_interface_list[client].update(set(add_topic_triples))
            topic_list = list(itertools.chain.from_iterable([[1, client], add_topic_triples]))
            self.flip(topic_list)

    def removeFlippedTopicByName(self,clients,name):
        topic_triples = self.getTopicString([name])
        for client in clients:
            if client not in self.flipped_interface_list:
                continue
            delete_topic_triples = [x for x in topic_triples if x in self.flipped_interface_list[client]]
            self.flipped_interface_list[client].difference_update(set(delete_topic_triples))
            topic_list = list(itertools.chain.from_iterable([[1, client], delete_topic_triples]))
            self.unflip(topic_list)

    def removeNamedFlippedTopics(self,list):
        # list[0] # of channel
        # list[1:list[0]] is channels
        # rest of them are fliping topics
        num = int(list[0])
        channels = list[1:num+1]
        topics = list[num+1:len(list)]
        print "removing named topics from flip: " + str(list)
        for chn in channels:
            if chn in self.flipped_topic_whitelist:
                self.flipped_topic_whitelist[chn].difference_update(set(topics))
        return True, []

    def addFlippedServiceByName(self,clients,name):
        service_triples = self.getServiceString([name])
        for client in clients:
            if client not in self.flipped_interface_list:
                self.flipped_interface_list[client] = set()
            add_service_triples = [x for x in service_triples if x not in self.flipped_interface_list[client]]
            self.flipped_interface_list[client].update(set(add_service_triples))
            service_list = list(itertools.chain.from_iterable([[1, client], add_service_triples]))
            self.flip(service_list)

    def addNamedFlippedServices(self, list):
        # list[0] # of channel
        # list[1:list[0]] is channels
        # rest of them are fliping services
        num = int(list[0])
        channels = list[1:num+1]
        services = list[num+1:len(list)]
        print "Adding named services to flip: " + str(list)
        for chn in channels:
            if chn not in self.flipped_service_whitelist:
                self.flipped_service_whitelist[chn] = set()
            self.flipped_service_whitelist[chn].update(set(services))
        return True, []


    def removeFlippedServiceByName(self,clients,name):
        service_triples = self.getServiceString([name])
        for client in clients:
            if client not in self.flipped_interface_list:
                continue
            delete_service_triples = [x for x in service_triples if x in self.flipped_interface_list[client]]
            self.flipped_interface_list[client].difference_update(set(delete_service_triples))
            service_list = list(itertools.chain.from_iterable([[1, client], delete_service_triples]))
            self.unflip(service_list)

    def removeNamedFlippedServices(self,list):
        # list[0] # of channel
        # list[1:list[0]] is channels
        # rest of them are fliping services
        num = int(list[0])
        channels = list[1:num+1]
        services = list[num+1:len(list)]
        print "removing named services from flip: " + str(list)
        for chn in channels:
            if chn in self.flipped_service_whitelist:
                self.flipped_service_whitelist[chn].difference_update(set(services))
        return True, []

    def addFlippedInterfaceByName(self,identifier,clients,name):
        if identifier == 'topic':
            self.addFlippedTopicByName(clients,name)
        elif identifier == 'service':
            self.addFlippedServiceByName(clients,name)

    def removeFlippedInterfaceByName(self,identifier,clients,name):
        if identifier == 'topic':
            self.removeFlippedTopicByName(clients,name)
        elif identifier == 'service':
            self.removeFlippedServiceByName(clients,name)

    def flipAll(self,list):
        #list is channels
        for chn in list:
            if chn not in self.flipped_topic_whitelist:
              self.flipped_topic_whitelist[chn] = set()
            if chn not in self.flipped_service_whitelist:
              self.flipped_service_whitelist[chn] = set()
            self.flipped_topic_whitelist[chn].add('.*')
            self.flipped_service_whitelist[chn].add('.*')
            if chn in self.flip_public_topics:
                self.flip_public_topics.remove(chn)
        return True, []

    def flipAllPublic(self,list):
        #list is channels
        for chn in list:
            if chn in self.flipped_topic_whitelist:
              self.flipped_topic_whitelist[chn].difference_update(set(['.*']))
            if chn in self.flipped_service_whitelist:
              self.flipped_service_whitelist[chn].difference_update(set(['.*']))
            self.flip_public_topics.add(chn)
        return True, []

    def flipListOnly(self,list):
        #list is channels
        for chn in list:
            if chn in self.flipped_topic_whitelist:
              self.flipped_topic_whitelist[chn].difference_update(set(['.*']))
            if chn in self.flipped_service_whitelist:
              self.flipped_service_whitelist[chn].difference_update(set(['.*']))
            if chn in self.flip_public_topics:
                self.flip_public_topics.remove(chn)
        return True, []

    def makeAllPublic(self,list):
        print "Dumping all non-blacklisted interfaces"
        self.public_topic_whitelist.append('.*')
        self.public_service_whitelist.append('.*')
        return True, []

    def removeAllPublic(self,list):
        print "Resuming dump of explicitly whitelisted interfaces"
        self.public_topic_whitelist[:] = [x for x in self.public_topic_whitelist if x != '.*']
        self.public_service_whitelist[:] = [x for x in self.public_service_whitelist if x != '.*']
        return True, []

    def allowInterface(self,name,whitelist,blacklist):
        in_whitelist = False
        in_blacklist = False
        for x in whitelist:
            if re.match(x, name):
                in_whitelist = True
                break
        for x in blacklist:
            if re.match(x, name):
                in_blacklist = True
                break

        return in_whitelist and (not in_blacklist)

    def allowInterfaceInPublic(self,identifier,name):
        if identifier == 'topic':
            whitelist = self.public_topic_whitelist
            blacklist = self.public_topic_blacklist
        else:
            whitelist = self.public_service_whitelist
            blacklist = self.public_service_blacklist
        return self.allowInterface(name,whitelist,blacklist)

    def allowInterfaceInFlipped(self,identifier,client,name):
        #print '  testing ' + identifier + ': ' + name + ' for ' + client
        if client in self.flip_public_topics:
          #print '    client in public list'
          return self.allowInterfaceInPublic(identifier,name)

        if identifier == 'topic':
            if client not in self.flipped_topic_whitelist:
                return False
            whitelist = self.flipped_topic_whitelist[client]
            blacklist = self.public_topic_blacklist
        else:
            if client not in self.flipped_service_whitelist:
                return False
            whitelist = self.flipped_service_whitelist[client]
            blacklist = self.public_service_blacklist
        return self.allowInterface(name,whitelist,blacklist)

    def getFlippedClientList(self,identifier,name):
        list = self.hub.listPublicInterfaces()
        allowed_clients = []
        not_allowed_clients = []
        for chn in list:
            if self.allowInterfaceInFlipped(identifier,chn,name):
                allowed_clients.append(chn)
            else:
                not_allowed_clients.append(chn)
        return [allowed_clients, not_allowed_clients]

    def clearServer(self):
        self.hub.unregisterGateway()
        self.master.clear()

    def processUpdate(self,cmd,provider,info):
        '''
          Used as a callback for incoming requests on redis pubsub channels.
          It gets assigned to RedisManager.callback.
        '''
        if cmd == "flip":
            self.pull(info)
        elif cmd == "unflip":
            self.unpull(info)
        else:
            rospy.logerr("Gateway : Received unknown command [%s] from [%s]"%(cmd,provider))

    def getInfo(self):
        return self.unique_name
