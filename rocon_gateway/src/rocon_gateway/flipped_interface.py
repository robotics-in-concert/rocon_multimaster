#!/usr/bin/env python
#       
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_multimaster/master/rocon_gateway/LICENSE 
#

##############################################################################
# Imports
##############################################################################

import roslib; roslib.load_manifest('rocon_gateway')
from gateway_comms.msg import RemoteRule
import copy
import threading
import re

# Local imports
import utils

##############################################################################
# Functions
##############################################################################

def flipRuleExists(flip_rule, flip_rules):
    '''
      Checks that the flip rule doesn't already exist in the list of flip
      rules (which can represent the flipped interface or the rules themselves).
      
      @param flip_rule : the rule to search for
      @type RemoteRule
      
      @param flip_rules : list of RemoteRule objects (flipped_interface[xxx] or rules[xxx]
      @type list : list of RemoteRule objects
      
      @return true if the flip rule exists, false otherwise
      @rtype bool
    '''
    for remote_rule in flip_rules:
        if remote_rule.gateway   == flip_rule.gateway and \
           remote_rule.rule.name == flip_rule.rule.name and \
           remote_rule.rule.node == flip_rule.rule.node:
            return True
    return False
          

##############################################################################
# Flipped Interface
##############################################################################

class FlippedInterface(object):
    '''
      The flipped interface is the set of rules 
      (pubs/subs/services/actions) and rules controlling flips
      to other gateways. 
    '''
    def __init__(self, default_rule_blacklist):
        '''
          Initialises the flipped interface.
        '''
        
        # keys are connection_types, elements are lists of RemoteRule objects
        self.flipped = utils.createEmptyConnectionTypeDictionary() # Rules that have been sent to remote gateways   
        self.watchlist = utils.createEmptyConnectionTypeDictionary()    # Specific rules used to determine what local rules to flip  
        
        # keys are connection_types, elements are lists of utils.Registration objects
        self.registrations = utils.createEmptyConnectionTypeDictionary() # Flips from remote gateways that have been locally registered
        
        # Default rules that cannot be flipped to any gateway - used in FlipAll mode
        self._default_blacklist = default_rule_blacklist # Note: dictionary of gateway-gateway_comms.msg.Rule lists, not RemoteRules!

        # Blacklists when doing flip all - different for each gateway, each value is one of our usual rule type dictionaries
        self._blacklist = {} 

        self.lock = threading.Lock()

    ##########################################################################
    # Rules
    ##########################################################################
        
    def addRule(self, flip_rule):
        '''
          Generate the flip rule, taking care to provide a sensible
          default for the remapping (root it in this gateway's namespace
          on the remote system).
          
          @param gateway, type, name, node
          @type str
          
          @param type : rule type
          @type str : string constant from gateway_comms.msg.Rule
          
          @return the flip rule, or None if the rule already exists.
          @rtype Flip || None
        '''
        result = None
        self.lock.acquire()
        if not flipRuleExists(flip_rule, self.watchlist[flip_rule.rule.type]):
            self.watchlist[flip_rule.rule.type].append(flip_rule)
            result = flip_rule
        self.lock.release()
        return result
    
    def removeRule(self, flip_rule):
        '''
          Remove a rule. Be a bit careful looking for a rule to remove, depending
          on the node name, which can be set (exact rule/node name match) or 
          None in which case all nodes of that kind of flip will match.
          
          Handle the remapping appropriately.
          
          @param flip_rule : the rule to unflip.
          @type RemoteRule
          
          @return Matching flip rule list
          @rtype RemoteRule[]
        '''
        if flip_rule.rule.node:
            # This looks for *exact* matches.
            try:
                self.lock.acquire()
                self.watchlist[flip_rule.rule.type].remove(flip_rule)
                self.lock.release()
                return [flip_rule]
            except ValueError:
                self.lock.release()
                return []
        else:
            # This looks for any flip rules which match except for the node name
            # also no need to check for type with the dic keys like they are
            existing_rules = []
            self.lock.acquire()
            for existing_rule in self.watchlist[flip_rule.rule.type]:
                if (existing_rule.gateway == flip_rule.gateway) and \
                   (existing_rule.rule.name == flip_rule.rule.name):
                    existing_rules.append(existing_rule)
            for rule in existing_rules:
                self.watchlist[flip_rule.rule.type].remove(existing_rule) # not terribly optimal
            self.lock.release()
            return existing_rules

    def flipAll(self, gateway, blacklist):
        '''
          Generate the flip all rule.
          
          @param gateway
          @type str
          
          @param blacklist : do not flip rules matching these patterns
          @type Rule[]
          
          @return failure if flip all rule exists, success otherwise
          @rtype Bool
        '''
        self.lock.acquire()
        # Blacklist
        if gateway in self._blacklist:
            self.lock.release()
            return False
        self._blacklist[gateway] = self._default_blacklist
        for rule in blacklist:
            self._blacklist[gateway][rule.type].append(rule)
        # Flips
        for connection_type in utils.connection_types:
            flip_rule = RemoteRule()
            flip_rule.gateway = gateway
            flip_rule.rule.name = '.*'
            flip_rule.rule.node = None
            flip_rule.rule.type = connection_type
            # Remove all other rules for that gateway
            self.watchlist[connection_type][:] = [rule for rule in self.watchlist[connection_type] if rule.gateway != gateway]
            # basically self.addRule() - do it manually here so we don't deadlock locks
            self.watchlist[connection_type].append(flip_rule)
        self.lock.release()
        return True

    def unFlipAll(self, gateway):
        '''
          Remove the flip all rule for the specified gateway.
          
          @param gateway
          @type str
          
          @param blacklist : do not flip rules matching these patterns
          @type Rule[]
          
          @return failure if flip all rule exists, success otherwise
          @rtype Bool
        '''
        self.lock.acquire()
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
        self.lock.release()

    ##########################################################################
    # Monitoring
    ##########################################################################

    def update(self,connections):
        '''
          Computes a new flipped interface and returns two dictionaries - 
          removed and newly added flips so the watcher thread can take
          appropriate action (inform the remote gateways).
          
          This is run in the watcher thread (warning: take care - other
          additions come from ros service calls in different threads!)
          
          @param connections : list of all the system state connections from the local master
          @type : connection type keyed dictionary of utils.Connection lists.
          
          @return new_flips, old_flips 
          @rtype pair of connection type keyed dictionary of gateway_comms.msg.Rule lists.
        '''
        # SLOW, EASY METHOD
        #   Totally regenerate a new flipped interface, compare with old
        flipped = utils.createEmptyConnectionTypeDictionary()
        new_flips = utils.createEmptyConnectionTypeDictionary()
        removed_flips = utils.createEmptyConnectionTypeDictionary()
        diff = lambda l1,l2: [x for x in l1 if x not in l2] # diff of lists
        self.lock.acquire()
        for connection_type in connections:
            for connection in connections[connection_type]:
                flipped[connection_type].extend(self._generateFlips(connection.rule.type, connection.rule.name, connection.rule.node))
            new_flips[connection_type] = diff(flipped[connection_type],self.flipped[connection_type])
            removed_flips[connection_type] = diff(self.flipped[connection_type],flipped[connection_type])
        self.flipped = copy.deepcopy(flipped)
        self.lock.release()
        return new_flips, removed_flips
        
        # OPTIMISED METHOD
        #   Keep old rule state and old flip rules/patterns around
        #
        #   1 - If flip rules/patterns disappeared [diff(old_rules,new_rules)]
        #         Check if the current flips are still valid
        #         If not all are, remove and unflip them
        #
        #   2 - If rules disappeared [diff(old_conns,new_conns)]
        #         If matching any in flipped, remove and unflip
        #
        #   3 - If flip rules/patterns appeared [diff(new_rules,old_rules)]
        #         parse all conns, if match found, flip
        #
        #   4 - If rules appeared [diff(new_conns,old_conns)]
        #         check for matches, if found, flou
        #
        # diff = lambda l1,l2: [x for x in l1 if x not in l2] # diff of lists

    ##########################################################################
    # Utility Methods
    ##########################################################################
    
    def findRegistrationMatch(self,remote_gateway,remote_name,remote_node,connection_type):
        '''
          Check to see if a registration exists. Note that it doesn't use the
          local node name in the check. We will get unflip requests that 
          don't have this variable set (that gets autogenerated when registering
          the flip), but we need to find the matching registration.
          
          We then return the registration that the unflip registration matches.
          
          @param remote_gateway : registration corresponding to unflip request
          @type utils.Registration
          
          @return matching registration or none
          @rtype utils.Registration
        '''
        
        matched_registration = None
        self.lock.acquire()
        for registration in self.registrations[connection_type]:
            if (registration.remote_gateway  == remote_gateway) and \
               (registration.connection.rule.name == remote_name) and \
               (registration.connection.rule.node == remote_node) and \
               (registration.connection.rule.type == connection_type):
                matched_registration = registration
                break
            else:
                continue
        self.lock.release()
        return matched_registration
        
    def _generateFlips(self, type, name, node):
        '''
          Checks if a local rule (obtained from master.getSystemState) 
          is a suitable association with any of the rules or patterns. This can
          return multiple matches, since the same local rule 
          properties can be multiply flipped to different remote gateways.
            
          Used in the update() call above that is run in the watcher thread.
          
          Note, don't need to lock here as the update() function takes care of it.
          
          @param type : rule type
          @type str : string constant from gateway_comms.msg.Rule
          
          @param name : fully qualified topic, service or action name
          @type str
          
          @param node : ros node name (coming from master.getSystemState)
          @type str
          
          @return all the flip rules that match this local rule
          @return list of RemoteRule objects updated with node names from self.watchlist
        '''
        matched_flip_rules = []
        for flip_rule in self.watchlist[type]:
            matched = False
            name_match_result = re.match(flip_rule.rule.name, name)
            if name_match_result and name_match_result.group() == name:
                if self._isFlipAllPattern(flip_rule.rule.name):
                    if self._isInBlacklist(flip_rule.gateway, type, name, node):
                        continue
                if flip_rule.rule.node:
                    node_match_result = re.match(flip_rule.rule.node,node)
                    if node_match_result and node_match_result.group() == node:
                        matched = True
                else: # flip_rule.rule.node is None so we don't care about matching the node
                    matched = True
            if matched:
                matched_flip = copy.deepcopy(flip_rule)
                matched_flip.rule.name = name # just in case we used a regex
                matched_flip.rule.node = node # just in case we used a regex
                matched_flip_rules.append(matched_flip)

        return matched_flip_rules
    
    ##########################################################################
    # Accessors
    ##########################################################################

    def flippedInConnections(self,connection_type):
        '''
          Parses the registrations list and hands out a set of flip rules for
          consumption by ros service getters (e.g. GatewayInfo). We don't need
          to show the service and node uri's here.
          
          Basic operation : convert Registration -> RemoteRule for each registration
          
          @param connection_type : one of Rule.XXX string constants.
          @type str
          
          @return the list of flip rules corresponding to local flip registrations
          @rtype RemoteRule[]
        '''
        flipped_in_rules = []
        for registration in self.registrations[connection_type]:
            flip_rule = RemoteRule()
            flip_rule.gateway = registration.remote_gateway
            flip_rule.rule.name = registration.remote_name
            flip_rule.rule.node = registration.remote_node
            flip_rule.rule.type = connection_type
            flipped_in_rules.append(flip_rule)
        return flipped_in_rules

    ##########################################################################
    # Utilities
    ##########################################################################

    def _isFlipAllPattern(self, pattern):
        ''' 
          Convenience function for detecting the flip all pattern.

          @todo move to utils - should be shared with the public interface.
          
          @param pattern : the name rule string for the flip all concept
          @type str
          @return true if matching, false otherwise
          @rtype Bool
        '''
        if pattern == ".*":
            return True
        else:
            return False
    
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
    
if __name__ == "__main__":
    
    r = re.compile("/chatte")
    result = r.match('/chatter')
    print result.group()
    print result.span()
    print len('/chatter')
