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

    def __init__(self, name, watch_loop_period):
        self.unresolved_name = name # This gets used to build unique names after connection to the hub
        self.unique_name = None # single string value set after hub connection (note: it is not a redis rocon:: rooted key!)
        self.is_connected = False
        self.flipped_interface = FlippedInterface() # Initalise the unique namespace hint for this upon connection later
        self.public_interface = PublicInterface()
        self.public_interface_lock = threading.Condition()
        self.master = LocalMaster()
        self.remote_gateway_request_callbacks = {}
        self.remote_gateway_request_callbacks['flip'] = self.processRemoteGatewayFlipRequest
        self.remote_gateway_request_callbacks['unflip'] = self.processRemoteGatewayUnFlipRequest
        self.hub = Hub(self.remote_gateway_request_callbacks, self.unresolved_name)

        # create a thread to watch local connection states
        self.watcher_thread = WatcherThread(self, watch_loop_period)

    ##########################################################################
    # Connection Logic
    ##########################################################################

    def connectToHub(self,ip,port):
        try:
            self.hub.connect(ip,port)
            self.unique_name = self.hub.registerGateway()
            self.is_connected = True
        except Exception as e:
            print str(e)
            return False
        return True

    def shutdown(self):
        self.hub.unregisterGateway()

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
        if public_interface == None:
            return gateway_comms.msg.Result.NO_HUB_CONNECTION, self.public_interface.getWatchlist(), []
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
                self.public_interface.disallowAll()
        except Exception as e:
            rospy.logerr("Gateway : advertise all call error [%s]."%str(e))
            result = gateway_comms.msg.Result.UNKNOWN_ADVERTISEMENT_ERROR

        #Do a manual update
        public_interface = self.updatePublicInterface()
        if public_interface == None:
            return gateway_comms.msg.Result.NO_HUB_CONNECTION, self.public_interface.getBlacklist(), []
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
                rospy.loginfo("Gateway : added flip rule [%s:(%s,%s)]"%(flip_rule.gateway,flip_rule.connection.name,flip_rule.connection.type))
                response.result = gateway_comms.msg.Result.SUCCESS
                # watcher thread will look after this from here
            else:
                rospy.logerr("Gateway : flip rule already exists [%s:(%s,%s)]"%(request.flip_rule.gateway,request.flip_rule.connection.name,request.flip_rule.connection.type))
                response.result = gateway_comms.msg.Result.FLIP_RULE_ALREADY_EXISTS
                response.error_message = "flip rule already exists ["+request.flip_rule.gateway+":"+request.flip_rule.connection.name+"]"
        else: # request.cancel
            flip_rules = self.flipped_interface.removeRule(request.flip_rule)
            if flip_rules:
                rospy.loginfo("Gateway : removed flip rule [%s:%s]"%(request.flip_rule.gateway,request.flip_rule.connection.name))
                response.result = gateway_comms.msg.Result.SUCCESS
                # watcher thread will look after this from here
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
    # Public Interface utility functions
    ##########################################################################

    def updatePublicInterface(self,connections = None):
        ''' 
          Process the list of local connections and check against 
          the current rules and patterns for changes. If a connection 
          has become (un)available take appropriate action.
          
          @param connections : pregenerated list of connections, if None, this
                               function will generate them
          @type gateway_comms.msg.Connection
        '''
        if not self.is_connected:
            rospy.logerr("Gateway : advertise call failed [no hub connection].")
            return None
        if not connections:
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

    ##########################################################################
    # Incoming commands from remote gateways
    ##########################################################################

    def processRemoteGatewayFlipRequest(self, registration):
        '''
          Used as a callback for incoming requests on redis pubsub channels.
          It gets assigned to RedisManager.callback.
        '''
        rospy.loginfo("Gateway : received a flip request [%s,%s,%s,%s]"%(registration.remote_gateway,registration.type,registration.type_info,registration.xmlrpc_uri))
        # probably not necessary as the flipping gateway will already check this
        if not registration in self.flipped_interface.registrations[registration.type]:
            new_registration = self.master.register(registration)
            if new_registration:
                self.flipped_interface.registrations[registration.type].append(new_registration)
    
    def processRemoteGatewayUnFlipRequest(self,remote_gateway, remote_name, remote_node, connection_type):
        print "unflipping %s:%s:%s:%s"%(remote_gateway,remote_name,remote_node,connection_type)
        rospy.loginfo("Gateway : received an unflip request [%s,%s,%s,%s]"%(remote_gateway,remote_name,remote_node,connection_type))
        existing_registration = self.flipped_interface.findRegistrationMatch(remote_gateway,remote_name,remote_node,connection_type)
        if existing_registration:
            self.master.unregister(existing_registration)
            self.flipped_interface.registrations[existing_registration.type].remove(existing_registration)
