#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_multimaster/master/rocon_gateway/LICENSE
#

##############################################################################
# Imports
##############################################################################

import roslib
roslib.load_manifest('rocon_gateway')
import threading
from gateway_msgs.msg import RemoteRule
import re
import copy

# Local imports
import utils

##############################################################################
# Classes
##############################################################################


class InteractiveInterface(object):
    '''
      Parent interface for flip and pull interfaces.
    '''
    def __init__(self, default_rule_blacklist, default_rules, all_targets):
        '''
          @param default_rule_blacklist : used when in flip/pull all mode
          @type dictionary of gateway
          @param default_rules : static rules to flip/pull on startup
          @type gateway_msgs.msg.RemoteRule[]
          @param all_targets : static flip/pull all targets to flip/pull to on startup
          @type string[]
        '''
        # Rules that are active, ie have been flipped or pulled from remote gateways 
        # keys are connection_types, elements are lists of RemoteRule objects
        # This gets aliased to self.flipped or self.pulled as necessary in
        # the subclasses
        self.active = utils.createEmptyConnectionTypeDictionary()

        # Default rules used in the xxxAll modes
        self._default_blacklist = default_rule_blacklist # dictionary of gateway-gateway_msgs.msg.Rule lists, not RemoteRules!
        
        # keys are connection_types, elements are lists of gateway_msgs.msg.RemoteRule objects
        self.watchlist = utils.createEmptyConnectionTypeDictionary()    # Specific rules used to determine what local rules to flip  
        
        # keys are connection_types, elements are lists of utils.Registration objects
        self.registrations = utils.createEmptyConnectionTypeDictionary() # Flips from remote gateways that have been locally registered
        
        # Blacklists when doing flip all - different for each gateway, each value is one of our usual rule type dictionaries
        self._blacklist = {}

        self._lock = threading.Lock()

        # Load up static rules.
        for rule in default_rules:
            self.addRule(rule)
        for gateway in all_targets:
            self.addAll(gateway,[]) # don't add the complexity of extra blacklists yet, maybe later
        
        
    ##########################################################################
    # Rules
    ##########################################################################
        
    def addRule(self, remote_rule):
        '''
          Add a remote rule to the watchlist for monitoring.
          
          @param remote_rule : the remote rule to add to the watchlist
          @type gateway_msgs.msg.RemoteRule
          
          @return the remote rule, or None if the rule already exists.
          @rtype gateway_msgs.msg.RemoteRule || None
        '''
        result = None
        self._lock.acquire()
        rule_already_exists = False
        # Could be a bit smarter here - given regex expressions an added
        # rule may be redundant. It doesn't break the eventual behaviour though
        for watched_rule in self.watchlist[remote_rule.rule.type]:
            if watched_rule.gateway   == remote_rule.gateway and \
               watched_rule.rule.name == remote_rule.rule.name and \
               watched_rule.rule.node == remote_rule.rule.node:
                rule_already_exists = True
                break
        if not rule_already_exists:
            self.watchlist[remote_rule.rule.type].append(remote_rule)
            result = remote_rule
        self._lock.release()
        return result

    def removeRule(self, remote_rule):
        '''
          Remove a rule. Be a bit careful looking for a rule to remove, depending
          on the node name, which can be set (exact rule/node name match) or 
          None in which case all nodes of that kind of flip will match.
          
          Handle the remapping appropriately.
          
          @param remote_rule : the remote rule to remove from the watchlist.
          @type gateway_msgs.msg.RemoteRule
         
          @return Rules remaining in the watchlist
          @rtype RemoteRule[]
        '''
        if remote_rule.rule.node:
            # This looks for *exact* matches.
            try:
                self._lock.acquire()
                self.watchlist[remote_rule.rule.type].remove(remote_rule)
                self._lock.release()
                return [remote_rule]
            except ValueError:
                self._lock.release()
                return []
        else:
            # This looks for any flip rules which match except for the node name
            existing_rules = []
            self._lock.acquire()
            for existing_rule in self.watchlist[remote_rule.rule.type]:
                if (existing_rule.gateway == remote_rule.gateway) and \
                   (existing_rule.rule.name == remote_rule.rule.name):
                    existing_rules.append(existing_rule)
            for rule in existing_rules:
                self.watchlist[remote_rule.rule.type].remove(rule) # not terribly optimal
            self._lock.release()
            return existing_rules

    def addAll(self, gateway, blacklist):
        '''
          Instead of watching/acting on specific rules, take action
          on everything except for rules in a blacklist.  
          
          @param gateway : target remote gateway string id
          @type str
          
          @param blacklist : do not act on rules matching these patterns
          @type gateway_msgs.msg.Rule[]
          
          @return success or failure depending on if it ahs already been set or not
          @rtype Bool
        '''
        self._lock.acquire()
        # Blacklist
        if gateway in self._blacklist:
            self._lock.release()
            return False
        self._blacklist[gateway] = self._default_blacklist
        for rule in blacklist:
            self._blacklist[gateway][rule.type].append(rule)
        # Flips
        for connection_type in utils.connection_types:
            remote_rule = RemoteRule()
            remote_rule.gateway = gateway
            remote_rule.rule.name = '.*'
            remote_rule.rule.node = None
            remote_rule.rule.type = connection_type
            # Remove all other rules for that gateway
            self.watchlist[connection_type][:] = [rule for rule in self.watchlist[connection_type] if rule.gateway != gateway]
            # basically self.addRule() - do it manually here so we don't deadlock locks
            self.watchlist[connection_type].append(remote_rule)
        self._lock.release()
        return True

    def removeAll(self, gateway):
        '''
          Remove the add all rule for the specified gateway.
          
          @param gateway : target remote gateway string id
          @type str
        '''
        self._lock.acquire()
        if gateway in self._blacklist:
            del self._blacklist[gateway]
        for connection_type in utils.connection_types:
            for rule in self.watchlist[connection_type]:
                if rule.gateway == gateway:
                    # basically self.removeRule() - do it manually here so we don't deadlock locks
                    try:
                        self.watchlist[connection_type].remove(rule)
                    except ValueError:
                        pass # should never get here
        self._lock.release()

    ##########################################################################
    # Accessors for Gateway Info
    ##########################################################################

    def is_matched(self,rule,name,node):
        matched = False
        name_match_result = re.match(rule.rule.name, name)
        if name_match_result and name_match_result.group() == name:
            if utils.isAllPattern(rule.rule.name):
                if self._isInBlacklist(rule.gateway, type, name,node):
                    return False
            if rule.rule.node:
                node_match_result = re.match(rule.rule.node,node)
                if node_match_result and node_match_result.group() == node:
                    matched = True
            else:
                matched = True
        return matched

    def getLocalRegistrations(self):
        '''
          Gets the local registrations for GatewayInfo consumption (flipped ins/pulls).
          
          We don't need to show the service and node uri's here.
          
          Basic operation : convert Registration -> RemoteRule for each registration
          
          @return the list of registrations corresponding to remote interactions
          @rtype RemoteRule[]
        '''
        local_registrations = []
        for connection_type in utils.connection_types:
            for registration in self.registrations[connection_type]:
                remote_rule = RemoteRule()
                remote_rule.gateway = registration.remote_gateway
                remote_rule.rule.name = registration.connection.rule.name
                remote_rule.rule.node = registration.connection.rule.node
                remote_rule.rule.type = connection_type
                local_registrations.append(remote_rule)
        return local_registrations

    def getWatchlist(self):
        '''
          Gets the watchlist for GatewayInfo consumption.
          
          @return the list of flip rules that are being watched
          @rtype gateway_msgs.msg.RemoteRule[]
        '''
        watchlist = []
        for connection_type in utils.connection_types:
            watchlist.extend(copy.deepcopy(self.watchlist[connection_type]))
        # ros messages must have string output
        for remote in watchlist:
            if not remote.rule.node:
                remote.rule.node = 'None'
        return watchlist

    ##########################################################################
    # Utilities
    ##########################################################################
    
    def findRegistrationMatch(self,remote_gateway,remote_name,remote_node,connection_type):
        '''
          Check to see if a registration exists. Note that it doesn't use the
          local node name in the check. We will get things like unflip requests that 
          don't have this variable set (that gets autogenerated when registering
          the flip), but we need to find the matching registration.
          
          We then return the registration that matches.
          
          @param remote_gateway : string remote gateway id
          @type string
          @param remote_name, remote_node, connection_type : remote connection details
          @type string
          
          @return matching registration or none
          @rtype utils.Registration
        '''
        
        matched_registration = None
        self._lock.acquire()
        for registration in self.registrations[connection_type]:
            if (registration.remote_gateway  == remote_gateway) and \
               (registration.connection.rule.name == remote_name) and \
               (registration.connection.rule.node == remote_node) and \
               (registration.connection.rule.type == connection_type):
                matched_registration = registration
                break
            else:
                continue
        self._lock.release()
        return matched_registration

    def _isInBlacklist(self, gateway, type, name, node):
        '''
          Check if a particular connection is in the blacklist. Use this to
          filter connections from the flipAll command.
          
          @todo move to utils - should be shared with the public interface.
        '''
        for blacklist_rule in self._blacklist[gateway][type]:
            name_match_result = re.match(blacklist_rule.name, name)
            if name_match_result and name_match_result.group() == name:
                if blacklist_rule.node:
                    node_match_result = re.match(blacklist_rule.node,node)
                    if node_match_result and node_match_result.group() == node:
                        return True
                else: # rule.connection.node is None so we don't care about matching the node
                    return True
        return False
