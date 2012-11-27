#!/usr/bin/env python
#       
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_multimaster/master/rocon_gateway/LICENSE 
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
import httplib

import roslib; roslib.load_manifest('rocon_gateway')
import rospy
import rosgraph
from std_msgs.msg import Empty

# Ros msgs
import gateway_msgs.msg
import gateway_msgs.srv
from gateway_msgs.msg import Rule
from gateway_msgs.srv import AdvertiseResponse
from gateway_msgs.srv import AdvertiseAllResponse

# Local imports
import utils
import ros_parameters
from .hub_api import Hub
from .master_api import LocalMaster
from .watcher_thread import WatcherThread
from .exceptions import GatewayError, UnavailableGatewayError
from .flipped_interface import FlippedInterface
from .public_interface import PublicInterface
from .pulled_interface import PulledInterface

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

    def __init__(self, param):
        self.param = param
        self.unresolved_name = self.param['name'] # This gets used to build unique names after rule to the hub
        self.unique_name = None # single string value set after hub rule (note: it is not a redis rocon:: rooted key!)
        self.is_connected = False
        default_rule_blacklist = ros_parameters.generateRules(self.param["default_blacklist"])
        
        default_rules, all_targets = ros_parameters.generateRemoteRules(self.param["default_flips"])
        self.flipped_interface = FlippedInterface(firewall = self.param['firewall'],
                                                  default_rule_blacklist = default_rule_blacklist, 
                                                  default_rules = default_rules,
                                                  all_targets = all_targets)
        default_rules, all_targets = ros_parameters.generateRemoteRules(self.param["default_pulls"]) 
        self.pulled_interface = PulledInterface(  default_rule_blacklist = default_rule_blacklist,
                                                  default_rules = default_rules,
                                                  all_targets = all_targets)
        self.public_interface = PublicInterface(  default_rule_blacklist=default_rule_blacklist,
                                                  default_rules = ros_parameters.generateRules(self.param['default_advertisements']))
        if self.param['advertise_all']:
            self.public_interface.advertiseAll([]) # no extra blacklist beyond the default (keeping it simple in yaml for now)
        self.master = LocalMaster()
        self.remote_gateway_request_callbacks = {}
        self.remote_gateway_request_callbacks['flip'] = self.processRemoteGatewayFlipRequest
        self.remote_gateway_request_callbacks['unflip'] = self.processRemoteGatewayUnflipRequest
        self.hub = Hub(self.remote_gateway_request_callbacks, self.unresolved_name, firewall=self.param['firewall'])

        # create a thread to watch local rule states
        self.watcher_thread = WatcherThread(self, self.param['watch_loop_period'])

    ##########################################################################
    # Rule Logic
    ##########################################################################

    def connectToHub(self,ip,port):
        try:
            self.hub.connect(ip,port)
            self.unique_name = self.hub.registerGateway()
            self.is_connected = True
        except Exception as e:
            rospy.logerr("Gateway : %s"%str(e))
            return False
        return True

    def shutdown(self):
        self.watcher_thread.shutdown()
        for connection_type in utils.connection_types:
            for flip in self.flipped_interface.flipped[connection_type]:
                self.hub.sendUnflipRequest(flip.gateway, flip.rule)
            for registration in self.flipped_interface.registrations[connection_type]:
                self.master.unregister(registration)
        self.hub.unregisterGateway()

    ##########################################################################
    # Incoming commands from local system (ros service callbacks)
    ##########################################################################

    def rosServiceAdvertise(self,request):
        '''
          Puts/Removes a number of rules on the public interface watchlist.
          As local rules matching these rules become available/go away,
          the public interface is modified accordingly. A manual update is done
          at the end of the advertise call to quickly capture existing
          rules

          @param request
          @type gateway_msgs.srv.AdvertiseRequest
          @return service response
          @rtype gateway_msgs.srv.AdvertiseReponse
        '''
        response = gateway_msgs.srv.AdvertiseResponse()
        response.result, response.error_message = self._rosServiceAdvertiseChecks()
        if response.result == gateway_msgs.msg.Result.SUCCESS:
            try:
                if not request.cancel:
                    for rule in request.rules:
                        if not self.public_interface.addRule(rule):
                            response.result = gateway_msgs.msg.Result.ADVERTISEMENT_EXISTS
                            response.error_message = "advertisment rule already exists [%s:(%s,%s)]"%(rule.name, rule.type, rule.node)
                else:
                    for rule in request.rules:
                        if not self.public_interface.removeRule(rule):
                            response.result = gateway_msgs.msg.Result.ADVERTISEMENT_NOT_FOUND
                            response.error_message = "advertisment not found [%s:(%s,%s)]"%(rule.name, rule.type, rule.node)
            except Exception as e:
                rospy.logerr("Gateway : unknown advertise error [%s]."%str(e))
                response.result = gateway_msgs.msg.Result.UNKNOWN_ADVERTISEMENT_ERROR

        # Let the watcher get on with the update asap
        if response.result == gateway_msgs.msg.Result.SUCCESS:
            self.watcher_thread.trigger_update = True
        else:
            rospy.logerr("Gateway : %s."%response.error_message)
        response.watchlist = self.public_interface.getWatchlist()
        return response

    def rosServiceAdvertiseAll(self,request):
        '''
          Toggles the advertise all mode. If advertising all, an additional 
          blacklist parameter can be supplied which includes all the topics that
          will not be advertised/watched for. This blacklist is added to the
          default blacklist of the public interface

          @param request
          @type gateway_msgs.srv.AdvertiseAllRequest
          @return service response
          @rtype gateway_msgs.srv.AdvertiseAllReponse
        '''
        response = gateway_msgs.srv.AdvertiseAllResponse()
        response.result, response.error_message = self._rosServiceAdvertiseChecks()
        if response.result == gateway_msgs.msg.Result.SUCCESS:
            try:
                if not request.cancel:
                    if not self.public_interface.advertiseAll(request.blacklist):
                        response.result = gateway_msgs.msg.Result.ADVERTISEMENT_EXISTS
                        response.error_message = "already advertising all."
                else:
                    self.public_interface.unadvertiseAll()
            except Exception as e:
                response.result = gateway_msgs.msg.Result.UNKNOWN_ADVERTISEMENT_ERROR
                response.error_message = "unknown advertise all error [%s]"%(str(e))

        # Let the watcher get on with the update asap
        if response.result == gateway_msgs.msg.Result.SUCCESS:
            self.watcher_thread.trigger_update = True
        else:
            rospy.logerr("Gateway : %s."%response.error_message)
        response.blacklist = self.public_interface.getBlacklist()
        return response

    def rosServiceFlip(self,request):
        '''
          Puts flip rules on a watchlist which (un)flips them when they 
          become (un)available. 
          
          @param request
          @type gateway_msgs.srv.RemoteRequest
          @return service response
          @rtype gateway_msgs.srv.RemoteResponse
        '''
        response = gateway_msgs.srv.RemoteResponse()
        for remote in request.remotes:
            response.result, response.error_message = self._rosServiceFlipChecks(remote.gateway)
            if response.result != gateway_msgs.msg.Result.SUCCESS:
                rospy.logerr("Gateway : %s."%response.error_message)
                return response
              
        # result is currently SUCCESS
        added_rules = []  
        for remote in request.remotes:
            if not request.cancel:
                flip_rule = self.flipped_interface.addRule(remote)
                if flip_rule:
                    added_rules.append(flip_rule)
                    rospy.loginfo("Gateway : added flip rule [%s:(%s,%s)]"%(flip_rule.gateway,flip_rule.rule.name,flip_rule.rule.type))
                else:
                    response.result = gateway_msgs.msg.Result.FLIP_RULE_ALREADY_EXISTS
                    response.error_message = "flip rule already exists [%s:(%s,%s)]"%(remote.gateway,remote.rule.name,remote.rule.type)
                    break
            else: # request.cancel
                removed_flip_rules = self.flipped_interface.removeRule(remote)
                if removed_flip_rules:
                    rospy.loginfo("Gateway : removed flip rule [%s:%s]"%(remote.gateway, remote.rule.name))
        
        if response.result == gateway_msgs.msg.Result.SUCCESS:
            self.watcher_thread.trigger_update = True
        else:
            if added_rules: # completely abort any added rules
                for added_rule in added_rules:
                    self.flipped_interface.removeRule(added_rule)
            rospy.logerr("Gateway : %s."%response.error_message)
        return response

    def rosServiceFlipAll(self,request):
        '''
          Flips everything except a specified blacklist to a particular gateway,
          or if the cancel flag is set, clears all flips to that gateway.
          
          @param request
          @type gateway_msgs.srv.RemoteAllRequest
          @return service response
          @rtype gateway_msgs.srv.RemoteAllResponse
        '''
        response = gateway_msgs.srv.RemoteAllResponse()
        response.result, response.error_message = self._rosServiceFlipChecks(request.gateway)
        if response.result == gateway_msgs.msg.Result.SUCCESS:
            if not request.cancel:
                if self.flipped_interface.flipAll(request.gateway, request.blacklist):
                    rospy.loginfo("Gateway : flipping all to gateway '%s'"%(request.gateway))
                else:
                    response.result = gateway_msgs.msg.Result.FLIP_RULE_ALREADY_EXISTS
                    response.error_message = "already flipping all to gateway '%s' "+request.gateway
            else: # request.cancel
                self.flipped_interface.unFlipAll(request.gateway)
                rospy.loginfo("Gateway : cancelling a previous flip all request [%s]"%(request.gateway))
        if response.result == gateway_msgs.msg.Result.SUCCESS:
            self.watcher_thread.trigger_update = True
        else:
            rospy.logerr("Gateway : %s."%response.error_message)
        return response

    def rosServicePull(self,request):
        '''
          Puts a single rule on a watchlist and (un)flips it to a particular 
          gateway when it becomes (un)available. Note that this can also
          completely reconfigure the fully qualified name for the rule when 
          flipping (remapping). If not specified, it will simply reroot rule
          under <unique_gateway_name>.
          
          @param request
          @type gateway_msgs.srv.RemoteRequest
          @return service response
          @rtype gateway_msgs.srv.RemoteResponse
        '''
        response = gateway_msgs.srv.RemoteResponse()
        for remote in request.remotes:
            response.result, response.error_message = self._rosServiceFlipChecks(remote.gateway)
            if response.result != gateway_msgs.msg.Result.SUCCESS:
                rospy.logerr("Gateway : %s."%response.error_message)
                return response

        # result is currently SUCCESS
        added_rules = []  
        for remote in request.remotes:
            if not request.cancel:
                pull_rule = self.pulled_interface.addRule(remote)
                if pull_rule:
                    added_rules.append(pull_rule)
                    rospy.loginfo("Gateway : added pull rule [%s:(%s,%s)]"%(pull_rule.gateway,pull_rule.rule.name,pull_rule.rule.type))
                else:
                    response.result = gateway_msgs.msg.Result.PULL_RULE_ALREADY_EXISTS
                    response.error_message = "pull rule already exists [%s:(%s,%s)]"%(remote.gateway,remote.rule.name,remote.rule.type)
                    break
            else: # request.cancel
                for remote in request.remotes:
                    removed_pull_rules = self.pulled_interface.removeRule(remote)
                    if removed_pull_rules:
                        rospy.loginfo("Gateway : removed pull rule [%s:%s]"%(remote.gateway, remote.rule.name))
        if response.result == gateway_msgs.msg.Result.SUCCESS:
            self.watcher_thread.trigger_update = True
        else:
            if added_rules: # completely abort any added rules
                for added_rule in added_rules:
                    self.pulled_interface.removeRule(added_rule)
            rospy.logerr("Gateway : %s."%response.error_message)
        return response

    def rosServicePullAll(self,request):
        '''
          Pull everything except a specified blacklist from a particular gateway,
          or if the cancel flag is set, clears all pulls from that gateway.
          
          @param request
          @type gateway_msgs.srv.RemoteAllRequest
          @return service response
          @rtype gateway_msgs.srv.RemoteAllResponse
        '''
        response = gateway_msgs.srv.RemoteAllResponse()
        response.result, response.error_message = self._rosServiceRemoteChecks(request.gateway)
        if response.result == gateway_msgs.msg.Result.SUCCESS:
            if not request.cancel:
                if self.pulled_interface.pullAll(request.gateway, request.blacklist):
                    rospy.loginfo("Gateway : pulling all from gateway '%s'"%(request.gateway))
                else:
                    response.result = gateway_msgs.msg.Result.FLIP_RULE_ALREADY_EXISTS
                    response.error_message = "already pulling all from gateway '%s' "+request.gateway
            else: # request.cancel
                self.pulled_interface.unPullAll(request.gateway)
                rospy.loginfo("Gateway : cancelling a previous pull all request [%s]"%(request.gateway))
        if response.result == gateway_msgs.msg.Result.SUCCESS:
            self.watcher_thread.trigger_update = True
        else:
            rospy.logerr("Gateway : %s."%response.error_message)
        return response

    def _rosServiceAdvertiseChecks(self):
        if not self.is_connected:
            return gateway_msgs.msg.Result.NO_HUB_CONNECTION, "not connected to hub, aborting"
        else: 
            return gateway_msgs.msg.Result.SUCCESS, ""

    def _rosServiceFlipChecks(self, gateway):
        '''
          Some simple checks for ros service flips.
          
          @param gateway : target gateway string of the flip
          @type string
          @return pair of result type and message
          @rtype gateway_msgs.msg.Result.xxx, string
        '''
        result, error_message = self._rosServiceRemoteChecks(gateway)
        if result == gateway_msgs.msg.Result.SUCCESS:
            firewall_flag = False
            try:
                firewall_flag = self.hub.getRemoteGatewayFirewallFlag(gateway)
                if firewall_flag:
                    return gateway_msgs.msg.Result.FLIP_REMOTE_GATEWAY_FIREWALLING, "remote gateway is firewalling flip requests, aborting [%s]"%gateway
            except UnavailableGatewayError:
                pass # handled earlier in rosServiceRemoteChecks
        return result, error_message

    def _rosServiceRemoteChecks(self, gateway):
        '''
          Some simple checks for ros service pulls
          
          @param gateway : target gateway string of the pull
          @type string
          @return pair of result type and message
          @rtype gateway_msgs.msg.Result.xxx, string
        '''
        if not self.is_connected:
            return gateway_msgs.msg.Result.NO_HUB_CONNECTION, "not connected to hub, aborting" 
        elif gateway == self.unique_name:
            return gateway_msgs.msg.Result.FLIP_NO_TO_SELF, "gateway cannot flip to itself"
        elif not self.hub.matchesRemoteGatewayName(gateway):
            return gateway_msgs.msg.Result.FLIP_REMOTE_GATEWAY_NOT_CONNECTED, "remote gateway is currently not connected [%s]"%gateway
        else:
            return gateway_msgs.msg.Result.SUCCESS, ""

    ##########################################################################
    # Update interface states (usually from watcher thread)
    ##########################################################################

    def updateFlipInterface(self,connections, gateways):
        '''
          Process the list of local connections and check against
          the current flip rules and patterns for changes. If a rule
          has become (un)available take appropriate action.
          
          @param connections : list of current local connections parsed from the master
          @type : dictionary of ConnectionType.xxx keyed lists of utils.Connections
          
          @param gateways : list of remote gateway string id's
          @type string
        '''
        new_flips, lost_flips = self.flipped_interface.update(connections, gateways)
        for connection_type in connections:
            for flip in new_flips[connection_type]:
                # for actions, need to post flip details here
                connections = self.master.generateConnectionDetails(flip.rule.type, flip.rule.name, flip.rule.node)
                if connection_type == utils.ConnectionType.ACTION_CLIENT or connection_type == utils.ConnectionType.ACTION_SERVER:
                    rospy.loginfo("Flipping to %s : %s"%(flip.gateway,utils.formatRule(flip.rule)))
                    self.hub.postFlipDetails(flip.gateway, flip.rule.name, flip.rule.type, flip.rule.node)
                    for connection in connections:
                        self.hub.sendFlipRequest(flip.gateway, connection) # flip the individual pubs/subs
                else:
                    for connection in connections:
                        rospy.loginfo("Flipping to %s : %s"%(flip.gateway,utils.formatRule(connection.rule)))
                        self.hub.sendFlipRequest(flip.gateway, connection)
                        self.hub.postFlipDetails(flip.gateway, connection.rule.name, connection.rule.type, connection.rule.node)
            for flip in lost_flips[connection_type]:
                rospy.loginfo("Unflipping to %s : %s"%(flip.gateway,utils.formatRule(flip.rule)))
                self.hub.sendUnflipRequest(flip.gateway, flip.rule)
                self.hub.removeFlipDetails(flip.gateway, flip.rule.name, flip.rule.type, flip.rule.node)

    def updatePulledInterface(self, connections, gateways ):
        '''
          Process the list of local connections and check against
          the current pull rules and patterns for changes. If a rule
          has become (un)available take appropriate action.
          
          @param connections : list of current local connections parsed from the master
          @type : dictionary of ConnectionType.xxx keyed lists of utils.Connections
          
          @param gateways : list of remote gateway string id's
          @type string
        '''
        for gateway in gateways + self.pulled_interface.listRemoteGatewayNames():
            connections = self.hub.getRemoteConnectionState(gateway)
            new_pulls, lost_pulls = self.pulled_interface.update(connections, gateway)
            for connection_type in connections:
                for pull in new_pulls[connection_type]:
                    for connection in connections[pull.rule.type]:
                        if connection.rule.name == pull.rule.name and \
                           connection.rule.node == pull.rule.node:
                            corresponding_connection = connection
                            break
                    # Register this pull
                    existing_registration = self.pulled_interface.findRegistrationMatch(gateway, pull.rule.name, pull.rule.node, pull.rule.type)
                    if not existing_registration:
                        registration = utils.Registration(connection, gateway) 
                        new_registration = self.master.register(registration)
                        self.pulled_interface.registrations[registration.connection.rule.type].append(new_registration)
                        self.hub.postPullDetails(gateway, pull.rule.name, pull.rule.type, pull.rule.node)
                for pull in lost_pulls[connection_type]:
                    # Unregister this pull
                    existing_registration = self.pulled_interface.findRegistrationMatch(gateway, pull.rule.name, pull.rule.node, pull.rule.type)
                    if existing_registration:
                        self.master.unregister(existing_registration)
                        self.hub.removePullDetails(gateway, pull.rule.name, pull.rule.type, pull.rule.node)
                        self.pulled_interface.registrations[existing_registration.connection.rule.type].remove(existing_registration)
                
    def updatePublicInterface(self, connections = None):
        ''' 
          Process the list of local connections and check against 
          the current rules and patterns for changes. If a rule 
          has become (un)available take appropriate action.
          
          @param connections : list of current local connections parsed from the master
          @type : dictionary of ConnectionType.xxx keyed lists of utils.Connections
        '''
        if not self.is_connected:
            rospy.logerr("Gateway : advertise call failed [no hub rule].")
            return None
        if not connections:
            try:
                connections = self.master.getConnectionState()
            except httplib.ResponseNotReady as e:
                rospy.logwarn("Received ResponseNotReady from master api")
                return None
        new_conns, lost_conns = self.public_interface.update(connections)
        public_interface = self.public_interface.getInterface()
        for connection_type in utils.connection_types:
            for connection in new_conns[connection_type]:
                rospy.loginfo("Gateway : adding rule to public interface %s"%utils.formatRule(connection.rule))
                self.hub.advertise(connection)
            for connection in lost_conns[connection_type]:
                rospy.loginfo("Gateway : removing rule to public interface %s"%utils.formatRule(connection.rule))
                self.hub.unadvertise(connection)
        return public_interface

    ##########################################################################
    # Incoming commands from remote gateways
    ##########################################################################

    def processRemoteGatewayFlipRequest(self, registration):
        '''
          Used as a callback for incoming requests on redis pubsub channels.
          It gets assigned to RedisManager.callback.
          
          @param registration : fully detailed registration to be processed
          @type utils.Registration
        '''
        if self.flipped_interface.firewall:
            rospy.logwarn("Gateway : firewalling a flip request %s"%registration)
        else:
            rospy.loginfo("Gateway : received a flip request %s"%registration)
            # probably not necessary as the flipping gateway will already check this
            existing_registration = self.flipped_interface.findRegistrationMatch(registration.remote_gateway,registration.connection.rule.name, registration.connection.rule.node, registration.connection.rule.type)
            if not existing_registration:
                new_registration = self.master.register(registration)
                if new_registration:
                    self.flipped_interface.registrations[registration.connection.rule.type].append(new_registration)
    
    def processRemoteGatewayUnflipRequest(self,rule,remote_gateway):
        rospy.loginfo("Gateway : received an unflip request from gateway %s: %s"%(remote_gateway,utils.formatRule(rule)))
        existing_registration = self.flipped_interface.findRegistrationMatch(remote_gateway,rule.name,rule.node,rule.type)
        if existing_registration:
            self.master.unregister(existing_registration)
            self.flipped_interface.registrations[existing_registration.connection.rule.type].remove(existing_registration)
