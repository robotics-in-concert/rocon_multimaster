#!/usr/bin/env python
#       
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_multimaster/master/rocon_gateway/LICENSE 
#

##############################################################################
# Imports
##############################################################################

import roslib; roslib.load_manifest('rocon_gateway')
import threading
from gateway_comms.msg import RemoteRule

# Local imports
import utils

##############################################################################
# Classes
##############################################################################

class ActionInterface(object):
    '''
      Parent interface for flip and pull interfaces.
    '''
    def __init__(self, default_rule_blacklist, default_rules):
        '''
          @param default_rule_blacklist : used when in flip all mode
          @type dictionary of gateway
          @param default_rules : static rules to launch the interface with
          @type gateway_comms.msg.RemoteRule[]
        '''
        # Default rules used in the xxxAll modes
        self._default_blacklist = default_rule_blacklist # dictionary of gateway-gateway_comms.msg.Rule lists, not RemoteRules!
        
        # keys are connection_types, elements are lists of gateway_comms.msg.RemoteRule objects
        self.watchlist = utils.createEmptyConnectionTypeDictionary()    # Specific rules used to determine what local rules to flip  
        
        # keys are connection_types, elements are lists of utils.Registration objects
        self.registrations = utils.createEmptyConnectionTypeDictionary() # Flips from remote gateways that have been locally registered
        
        # Blacklists when doing flip all - different for each gateway, each value is one of our usual rule type dictionaries
        self._blacklist = {}

        self._lock = threading.Lock()

        # Load up static rules.
        for rule in default_rules:
            self.addRule(rule)
        
        
    ##########################################################################
    # Rules
    ##########################################################################
        
    def addRule(self, remote_rule):
        '''
          Add a remote rule to the watchlist for monitoring.
          
          @param remote_rule : the remote rule to add to the watchlist
          @type gateway_comms.msg.RemoteRule
          
          @return the remote rule, or None if the rule already exists.
          @rtype gateway_comms.msg.RemoteRule || None
        '''
        result = None
        self._lock.acquire()
        rule_already_exists = False
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
          @type gateway_comms.msg.RemoteRule
         
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
                self.watchlist[remote_rule.rule.type].remove(existing_rule) # not terribly optimal
            self._lock.release()
            return existing_rules

    def addAll(self, gateway, blacklist):
        '''
          Instead of watching/acting on specific rules, take action
          on everything except for rules in a blacklist.  
          
          @param gateway : target remote gateway string id
          @type str
          
          @param blacklist : do not act on rules matching these patterns
          @type gateway_comms.msg.Rule[]
          
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
          @rtype gateway_comms.msg.RemoteRule[]
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
