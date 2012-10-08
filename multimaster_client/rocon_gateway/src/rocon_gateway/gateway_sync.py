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
import threading

import roslib; roslib.load_manifest('rocon_gateway')
import rospy
import rosgraph
from std_msgs.msg import Empty

# Ros Comms
import gateway_comms.msg
import gateway_comms.srv
from gateway_comms.msg import Connection
from gateway_comms.srv import AdvertiseResponse
from gateway_comms.srv import AdvertiseAllResponse

# Local imports
import utils
from .hub_api import Hub
from .master_api import LocalMaster
from .watcher_thread import WatcherThread
from .exceptions import GatewayError, ConnectionTypeError
from .flipped_interface import FlippedInterface
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
        self.is_connected = False
        self.flipped_interface = FlippedInterface() # Initalise the unique namespace hint for this upon connection later
        self.public_interface = PublicInterface()
        self.public_interface_lock = threading.Condition()
        self.hub = Hub(self.processRemoteGatewayRequest, self.unresolved_name)
        self.master = LocalMaster()

        # create a thread to watch local connection states
        self.watcher_thread = WatcherThread(self)

    ##########################################################################
    # Connection Logic
    ##########################################################################

    def connectToHub(self,ip,port):
        try:
            self.hub.connect(ip,port)
            self.unique_name = self.hub.registerGateway()
            self.flipped_interface.setDefaultRootNamespace(self.unique_name)
            self.is_connected = True
        except Exception as e:
            print str(e)
            return False
        return True

    def shutdown(self):
        self.hub.unregisterGateway()
        self.master.clear()

    ##########################################################################
    # Incoming commands from local system (ros service callbacks)
    ##########################################################################

    def rosServiceAdvertise(self,request):
        '''
          Puts/Removes a number of rules on the public interface watchlist.
          As local connections matching these rules become available/go away,
          the public interface is modified accordingly. A manual update is done
          at the end of the advertise call to quickly capture existing
          connections

          @param request
          @type gateway_comms.srv.AdvertiseRequest
          @return service response
          @rtype gateway_comms.srv.AdvertiseReponse
        '''
        result = gateway_comms.msg.Result.SUCCESS
        try:
            if not request.cancel:
                for rule in request.rules:
                    if not self.public_interface.addRule(rule):
                        result = gateway_comms.msg.Result.ADVERTISEMENT_EXISTS
            else:
                for rule in request.rules:
                    if not self.public_interface.removeRule(rule):
                        result = gateway_comms.msg.Result.ADVERTISEMENT_NOT_FOUND
        except Exception as e:
            rospy.logerr("Gateway : advertise call error [%s]."%str(e))
            result = gateway_comms.msg.Result.UNKNOWN_ADVERTISEMENT_ERROR

        #Do a manual update
        public_interface = self.updatePublicInterface()
        return result, self.public_interface.getWatchlist(), public_interface

    def rosServiceAdvertiseAll(self,request):
        '''
          Toggles the advertise all mode. If advertising all, an additional 
          blacklist parameter can be supplied which includes all the topics that
          will not be advertised/watched for. This blacklist is added to the
          default blacklist of the public interface

          @param request
          @type gateway_comms.srv.AdvertiseAllRequest
          @return service response
          @rtype gateway_comms.srv.AdvertiseAllReponse
        '''
        result = gateway_comms.msg.Result.SUCCESS
        try:
            if not request.cancel:
                self.public_interface.allowAll(request.blacklist)
            else:
                self.public_interface.allowAll(request.blacklist)
        except Exception as e:
            rospy.logerr("Gateway : advertise all call error [%s]."%str(e))
            result = gateway_comms.msg.Result.UNKNOWN_ADVERTISEMENT_ERROR

        #Do a manual update
        public_interface = self.updatePublicInterface()
        return result, self.public_interface.getBlacklist(), public_interface

    def rosServiceFlip(self,request):
        '''
          Puts a single connection on a watchlist and (un)flips it to a particular 
          gateway when it becomes (un)available. Note that this can also
          completely reconfigure the fully qualified name for the connection when 
          flipping (remapping). If not specified, it will simply reroot connection
          under <unique_gateway_name>.
          
          @param request
          @type gateway_comms.srv.FlipRequest
          @return service response
          @rtype gateway_comms.srv.FlipResponse
        '''
        response = gateway_comms.srv.FlipResponse()
        if not self.is_connected:
            rospy.logerr("Gateway : no hub connection, aborting flip.")
            response.result = gateway_comms.msg.Result.NO_HUB_CONNECTION
            response.error_message = "no hub connection" 
        elif request.flip_rule.gateway == self.unique_name:
            rospy.logerr("Gateway : gateway cannot flip to itself.")
            response.result = gateway_comms.msg.Result.FLIP_NO_TO_SELF
            response.error_message = "gateway cannot flip to itself" 
        elif not request.cancel:
            flip_rule = self.flipped_interface.addRule(request.flip_rule)
            if flip_rule:
                rospy.loginfo("Gateway : flipping to gateway %s [%s->%s]"%(flip_rule.gateway,flip_rule.connection.name,flip_rule.remapped_name))
                response.result = gateway_comms.msg.Result.SUCCESS
                # watcher thread will look after this from here
            else:
                rospy.logerr("Gateway : flip rule already exists [%s:%s->%s]"%(request.flip_rule.gateway,request.flip_rule.connection.name,request.flip_rule.remapped_name))
                response.result = gateway_comms.msg.Result.FLIP_RULE_ALREADY_EXISTS
                response.error_message = "flip rule already exists ["+request.flip_rule.gateway+":"+request.flip_rule.connection.name+"->"+request.flip_rule.remapped_name+"]"
        else: # request.cancel
            # unflip handling
            pass  
        return response

    def rosServiceFlipPattern(self,request):
        '''
          Puts regex patterns on a watchlist and (un)flips them on a particular
          gateway when they become (un)available. Note that this cannot remap, 
          but can optionally reroot connections under a configurable namespace (default is 
          <unique_gateway_name>). 
          
          @param request
          @type gateway_comms.srv.FlipPatternRequest
          @return service response
          @rtype gateway_comms.srv.FlipPatternResponse
        '''
        response = gateway_comms.srv.FlipPatternResponse()
        return response

    def rosServiceFlipAll(self,request):
        '''
          Flips everything except a specified blacklist to a particular gateway,
          or if the cancel flag is set, clears all flips to that gateway.
          
          @param request
          @type gateway_comms.srv.FlipAllRequest
          @return service response
          @rtype gateway_comms.srv.FlipAllResponse
        '''
        response = FlipAllResponse()
        return response

    ##########################################################################
    # Incoming commands from remote gateways
    ##########################################################################

    def processRemoteGatewayRequest(self,command, gateway, remapped_name, connection_type, type, xmlrpc_uri):
        '''
          Used as a callback for incoming requests on redis pubsub channels.
          It gets assigned to RedisManager.callback.
        '''
        if command == "flip":
            rospy.loginfo("Gateway : received a flip request [%s,%s,%s,%s,%s]"%(gateway,remapped_name,connection_type,type,xmlrpc_uri))
            #self.pull(info)
        elif command == "unflip":
            #self.unpull(info)
            pass
        else:
            rospy.logerr("Gateway : received unknown command [%s:%s]"%(command,gateway))

    ##########################################################################
    # Others - what are we using and what not?
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

    def updatePublicInterface(self):
        ''' 
          Process the list of local connections and check against 
          the current rules and patterns for flips. If a connection 
          has become (un)available take appropriate action.
        '''
        if not self.is_connected:
            rospy.logerr("Gateway : advertise call failed [no hub connection].")
            return None
        connections = self.master.getConnectionState()
        self.public_interface_lock.acquire()
        new_conns, lost_conns = self.public_interface.update(connections)
        public_interface = self.public_interface.getInterface()
        self.public_interface_lock.release()
        for connection_type in new_conns:
            for connection in new_conns[connection_type]:
                rospy.loginfo("Gateway : adding connection to public interface %s"%utils.formatConnection(connection.connection))
                self.hub.advertise(connection)
            for connection in lost_conns[connection_type]:
                rospy.loginfo("Gateway : removing connection to public interface %s"%utils.formatConnection(connection.connection))
                self.hub.unadvertise(connection)
        return public_interface

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


    ##########################################################################
    # Depracating
    ##########################################################################
    # (PK) MOVED TO PUBLIC INTERFACE
    # def _initializeWatchlists(self):
    #     '''
    #     Initializes all watchlists (public/flip/pull) with null sets. This
    #     function is only used to initialize the vairable names, incase the 
    #     gateway does not setup the default lists explicityly
    #     '''
    #     self._public_watchlist = utils.getEmptyConnectionList()

    # def _initializeBlacklists(self):
    #     '''
    #     Initializes all blacklists (topics/services/actions) that can never be
    #     advertised/flipped/pulled. A blacklist supplied in /pull_all, /flip_all,
    #     /advertise_all will be in addition to this blacklist.
    #     '''
    #     self._default_blacklist = utils.getEmptyConnectionList()
    #     self._public_blacklist = utils.getEmptyConnectionList()
    #     
    # def setDefaultBlacklist(self, blacklist):
    #     '''
    #     Sets the default blacklists. This function should be called
    #     during gateway initialization with blacklists provided through a
    #     parameter file

    #     @param blacklists : a pre-formatted blacklists dict, most likely from
    #     @type dict of sets of tuples
    #     '''
    #     self._default_blacklist = blacklist

    # def setPublicWatchlist(self, watchlist):
    #     '''
    #     Sets the default blacklists. This function should be called
    #     during gateway initialization with blacklists provided through a
    #     parameter file

    #     @param blacklists : a pre-formatted blacklists dict, most likely from
    #     @type dict of sets of tuples
    #     '''
    #     self._public_watchlist = watchlist

    
#    def addNamedFlippedTopics(self, list):
#        # list[0] # of channel
#        # list[1:list[0]] is channels
#        # rest of them are fliping topics
#        num = int(list[0])
#        channels = list[1:num+1]
#        topics = list[num+1:len(list)]
#        print "Adding named topics to flip: " + str(list)
#        for chn in channels:
#            if chn not in self.flipped_topic_whitelist:
#                self.flipped_topic_whitelist[chn] = set()
#            self.flipped_topic_whitelist[chn].update(set(topics))
#        return True, []
#
#    def addFlippedTopicByName(self,clients,name):
#        topic_triples = self.getTopicString([name])
#        for client in clients:
#            if client not in self.flipped_interface_list:
#                self.flipped_interface_list[client] = set()
#            add_topic_triples = [x for x in topic_triples if x not in self.flipped_interface_list[client]]
#            self.flipped_interface_list[client].update(set(add_topic_triples))
#            topic_list = list(itertools.chain.from_iterable([[1, client], add_topic_triples]))
#            self.flip(topic_list)
#
#    def removeFlippedTopicByName(self,clients,name):
#        topic_triples = self.getTopicString([name])
#        for client in clients:
#            if client not in self.flipped_interface_list:
#                continue
#            delete_topic_triples = [x for x in topic_triples if x in self.flipped_interface_list[client]]
#            self.flipped_interface_list[client].difference_update(set(delete_topic_triples))
#            topic_list = list(itertools.chain.from_iterable([[1, client], delete_topic_triples]))
#            self.unflip(topic_list)
#
#    def removeNamedFlippedTopics(self,list):
#        # list[0] # of channel
#        # list[1:list[0]] is channels
#        # rest of them are fliping topics
#        num = int(list[0])
#        channels = list[1:num+1]
#        topics = list[num+1:len(list)]
#        print "removing named topics from flip: " + str(list)
#        for chn in channels:
#            if chn in self.flipped_topic_whitelist:
#                self.flipped_topic_whitelist[chn].difference_update(set(topics))
#        return True, []
#
#    def addFlippedServiceByName(self,clients,name):
#        service_triples = self.getServiceString([name])
#        for client in clients:
#            if client not in self.flipped_interface_list:
#                self.flipped_interface_list[client] = set()
#            add_service_triples = [x for x in service_triples if x not in self.flipped_interface_list[client]]
#            self.flipped_interface_list[client].update(set(add_service_triples))
#            service_list = list(itertools.chain.from_iterable([[1, client], add_service_triples]))
#            self.flip(service_list)
#
#    def addNamedFlippedServices(self, list):
#        # list[0] # of channel
#        # list[1:list[0]] is channels
#        # rest of them are fliping services
#        num = int(list[0])
#        channels = list[1:num+1]
#        services = list[num+1:len(list)]
#        print "Adding named services to flip: " + str(list)
#        for chn in channels:
#            if chn not in self.flipped_service_whitelist:
#                self.flipped_service_whitelist[chn] = set()
#            self.flipped_service_whitelist[chn].update(set(services))
#        return True, []
#
#
#    def removeFlippedServiceByName(self,clients,name):
#        service_triples = self.getServiceString([name])
#        for client in clients:
#            if client not in self.flipped_interface_list:
#                continue
#            delete_service_triples = [x for x in service_triples if x in self.flipped_interface_list[client]]
#            self.flipped_interface_list[client].difference_update(set(delete_service_triples))
#            service_list = list(itertools.chain.from_iterable([[1, client], delete_service_triples]))
#            self.unflip(service_list)
#
#    def removeNamedFlippedServices(self,list):
#        # list[0] # of channel
#        # list[1:list[0]] is channels
#        # rest of them are fliping services
#        num = int(list[0])
#        channels = list[1:num+1]
#        services = list[num+1:len(list)]
#        print "removing named services from flip: " + str(list)
#        for chn in channels:
#            if chn in self.flipped_service_whitelist:
#                self.flipped_service_whitelist[chn].difference_update(set(services))
#        return True, []
#
#    def addFlippedInterfaceByName(self,identifier,clients,name):
#        if identifier == 'topic':
#            self.addFlippedTopicByName(clients,name)
#        elif identifier == 'service':
#            self.addFlippedServiceByName(clients,name)
#
#    def removeFlippedInterfaceByName(self,identifier,clients,name):
#        if identifier == 'topic':
#            self.removeFlippedTopicByName(clients,name)
#        elif identifier == 'service':
#            self.removeFlippedServiceByName(clients,name)
#
#    def flipAll(self,list):
#        #list is channels
#        for chn in list:
#            if chn not in self.flipped_topic_whitelist:
#              self.flipped_topic_whitelist[chn] = set()
#            if chn not in self.flipped_service_whitelist:
#              self.flipped_service_whitelist[chn] = set()
#            self.flipped_topic_whitelist[chn].add('.*')
#            self.flipped_service_whitelist[chn].add('.*')
#            if chn in self.flip_public_topics:
#                self.flip_public_topics.remove(chn)
#        return True, []
#
#    def flipAllPublic(self,list):
#        #list is channels
#        for chn in list:
#            if chn in self.flipped_topic_whitelist:
#              self.flipped_topic_whitelist[chn].difference_update(set(['.*']))
#            if chn in self.flipped_service_whitelist:
#              self.flipped_service_whitelist[chn].difference_update(set(['.*']))
#            self.flip_public_topics.add(chn)
#        return True, []
#
#    def flipListOnly(self,list):
#        #list is channels
#        for chn in list:
#            if chn in self.flipped_topic_whitelist:
#              self.flipped_topic_whitelist[chn].difference_update(set(['.*']))
#            if chn in self.flipped_service_whitelist:
#              self.flipped_service_whitelist[chn].difference_update(set(['.*']))
#            if chn in self.flip_public_topics:
#                self.flip_public_topics.remove(chn)
#        return True, []
#
#    def allowInterfaceInFlipped(self,identifier,client,name):
#        #print '  testing ' + identifier + ': ' + name + ' for ' + client
#        if client in self.flip_public_topics:
#          #print '    client in public list'
#          return self.allowInterfaceInPublic(identifier,name)
#
#        if identifier == 'topic':
#            if client not in self.flipped_topic_whitelist:
#                return False
#            whitelist = self.flipped_topic_whitelist[client]
#            blacklist = self.public_topic_blacklist
#        else:
#            if client not in self.flipped_service_whitelist:
#                return False
#            whitelist = self.flipped_service_whitelist[client]
#            blacklist = self.public_service_blacklist
#        return self.allowInterface(name,whitelist,blacklist)
#    def getFlippedClientList(self,identifier,name):
#        list = self.hub.listPublicInterfaces()
#        allowed_clients = []
#        not_allowed_clients = []
#        for chn in list:
#            if self.allowInterfaceInFlipped(identifier,chn,name):
#                allowed_clients.append(chn)
#            else:
#                not_allowed_clients.append(chn)
#        return [allowed_clients, not_allowed_clients]

    # def advertiseConnection(self,connection):
    #     '''
    #     Adds a connection (topic/service/action) to the public interface.
    #     
    #     - adds to the public interface list
    #     - adds to the hub so it can be pulled by remote gateways
    #     
    #     @param connection : tuple containing connection information
    #     @type tuple
    #     '''
    #     if not self.is_connected:
    #         rospy.logerr("Gateway : advertise connection call failed [no hub connection].")
    #         return False
    #     try:
    #         if self.public_interface.add(connection):
    #             #self.hub.advertise(connection)
    #             pass
    #     except Exception as e: 
    #         rospy.logerr("Gateway : advertise connection call failed [%s]"%str(e))
    #         return False
    #     return True

    # def unadvertiseConnection(self,connection):
    #     '''
    #     Removes a connection (topic/service/action) to the public interface.
    #     
    #     - remove the public interface list
    #     - remove the connection from the hub, the hub announces the removal
    #     
    #     @param connection : tuple containing connection information
    #     @type tuple
    #     '''
    #     if not self.is_connected:
    #         rospy.logerr("Gateway : advertise call failed [no hub connection].")
    #         return False
    #     try:
    #         if self.public_interface.remove(connection):
    #             #self.hub.unadvertise(connection)
    #             pass
    #     except Exception as e: 
    #         rospy.logerr("Gateway : advertiseList call failed [%s]"%str(e))
    #         return False
    #     return True
