#!/usr/bin/env python
#       
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_multimaster/master/multimaster_client/rocon_gateway/LICENSE 
#

##############################################################################
# Imports
##############################################################################

import roslib; roslib.load_manifest('rocon_gateway')
from gateway_comms.msg import Connection, FlipRule
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
      @type FlipRule
      
      @param flip_rules : list of FlipRule objects (flipped_interface[xxx] or rules[xxx]
      @type list : list of FlipRule objects
      
      @return true if the flip rule exists, false otherwise
      @rtype bool
    '''
    for rule in flip_rules:
        if rule.gateway         == flip_rule.gateway and \
           rule.connection.name == flip_rule.connection.name and \
           rule.connection.node == flip_rule.connection.node:
            return True
    return False
          

##############################################################################
# Flipped Interface
##############################################################################

class FlippedInterface(object):
    '''
      The flipped interface is the set of connections 
      (pubs/subs/services/actions) and rules controlling flips
      to other gateways. 
    '''
    def __init__(self, default_connection_blacklist):
        '''
          Initialises the flipped interface.
        '''
        
        # keys are connection_types, elements are lists of FlipRule objects
        self.flipped = utils.createEmptyConnectionTypeDictionary() # Connections that have been sent to remote gateways   
        self.watchlist = utils.createEmptyConnectionTypeDictionary()    # Specific rules used to determine what local connections to flip  
        
        # keys are connection_types, elements are lists of utils.Registration objects
        self.registrations = utils.createEmptyConnectionTypeDictionary() # Flips from remote gateways that have been locally registered
        
        # Default rules that cannot be flipped to any gateway - used in FlipAll mode
        self._default_blacklist = default_connection_blacklist # Note: dictionary of gateway-gateway_comms.msg.Connection lists, not FlipRules!

        # Blacklists when doing flip all - different for each gateway, each value is one of our usual connection type dictionaries
        self._blacklist = {} 

        self.lock = threading.Lock()
        
    def addRule(self, flip_rule):
        '''
          Generate the flip rule, taking care to provide a sensible
          default for the remapping (root it in this gateway's namespace
          on the remote system).
          
          @param gateway, type, name, node
          @type str
          
          @param type : connection type
          @type str : string constant from gateway_comms.msg.Connection
          
          @return the flip rule, or None if the rule already exists.
          @rtype Flip || None
        '''
        result = None
        self.lock.acquire()
        if not flipRuleExists(flip_rule, self.watchlist[flip_rule.connection.type]):
            self.watchlist[flip_rule.connection.type].append(flip_rule)
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
          @type FlipRule
          
          @return Matching flip rule list
          @rtype FlipRule[]
        '''
        if flip_rule.connection.node:
            # This looks for *exact* matches.
            try:
                self.lock.acquire()
                self.watchlist[flip_rule.connection.type].remove(flip_rule)
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
            for existing_rule in self.watchlist[flip_rule.connection.type]:
                if (existing_rule.gateway == flip_rule.gateway) and \
                   (existing_rule.connection.name == flip_rule.connection.name):
                    existing_rules.append(existing_rule)
            for rule in existing_rules:
                self.watchlist[flip_rule.connection.type].remove(existing_rule) # not terribly optimal
            self.lock.release()
            return existing_rules

    def flipAll(self, gateway, blacklist):
        '''
          Generate the flip all rule.
          
          @param gateway
          @type str
          
          @param blacklist : do not flip connections matching these patterns
          @type Connection[]
          
          @return failure if flip all rule exists, success otherwise
          @rtype Bool
        '''
        self.lock.acquire()
        # Blacklist
        if gateway in self._blacklist:
            self.lock.release()
            return False
        self._blacklist[gateway] = self._default_blacklist
        for connection in blacklist:
            self._blacklist[gateway][connection.type].append(connection)
        # Flips
        for connection_type in utils.connection_types:
            flip_rule = FlipRule()
            flip_rule.gateway = gateway
            flip_rule.connection.name = '.*'
            flip_rule.connection.node = None
            flip_rule.connection.type = connection_type
            # Remove all other rules for that gateway
            self.watchlist[connection_type][:] = [rule for rule in self.watchlist[connection_type] if rule.gateway != gateway]
            # basically self.addRule() - do it manually here so we don't deadlock locks
            self.watchlist[connection_type].append(flip_rule)
        self.lock.release()
        return True

    def removeFlipAll(self, gateway):
        '''
          Remove the flip all rule for the specified gateway.
          
          @param gateway
          @type str
          
          @param blacklist : do not flip connections matching these patterns
          @type Connection[]
          
          @return failure if flip all rule exists, success otherwise
          @rtype Bool
        '''
        self.lock.acquire()
        if gateway in self._blacklist:
            return False
        del self._blacklist[gateway]
        flip_rule = FlipRule()
        flip_rule.gateway = gateway
        flip_rule.connection.name = '.*'
        flip_rule.connection.node = None
        for connection_type in utils.connection_types:
            flip_rule.connection.type = connection_type
            # basically self.removeRule() - do it manually here so we don't deadlock locks
            try:
                self.watchlist[flip_rule.connection.type].remove(flip_rule)
            except ValueError:
                pass # should never get here
        self.lock.release()
        return True

    def update(self,connections):
        '''
          Computes a new flipped interface and returns two dictionaries - 
          removed and newly added flips so the watcher thread can take
          appropriate action (inform the remote gateways).
          
          This is run in the watcher thread (warning: take care - other
          additions come from ros service calls in different threads!)
          
          @todo this will need a threading condition here to avoid muckups
                when adding flip rules etc.
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
                flipped[connection_type].extend(self._generateFlips(connection.type, connection.name, connection.node))
            new_flips[connection_type] = diff(flipped[connection_type],self.flipped[connection_type])
            removed_flips[connection_type] = diff(self.flipped[connection_type],flipped[connection_type])
        self.flipped = copy.deepcopy(flipped)
        self.lock.release()
        return new_flips, removed_flips
        
        # OPTIMISED METHOD
        #   Keep old connection state and old flip rules/patterns around
        #
        #   1 - If flip rules/patterns disappeared [diff(old_rules,new_rules)]
        #         Check if the current flips are still valid
        #         If not all are, remove and unflip them
        #
        #   2 - If connections disappeared [diff(old_conns,new_conns)]
        #         If matching any in flipped, remove and unflip
        #
        #   3 - If flip rules/patterns appeared [diff(new_rules,old_rules)]
        #         parse all conns, if match found, flip
        #
        #   4 - If connections appeared [diff(new_conns,old_conns)]
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
               (registration.remote_name     == remote_name) and \
               (registration.remote_node     == remote_node) and \
               (registration.connection_type == connection_type):
                matched_registration = registration
                break
            else:
                continue
        self.lock.release()
        return matched_registration
        
    def _generateFlips(self, type, name, node):
        '''
          Checks if a local connection (obtained from master.getSystemState) 
          is a suitable association with any of the rules or patterns. This can
          return multiple matches, since the same local connection 
          properties can be multiply flipped to different remote gateways.
            
          Used in the update() call above that is run in the watcher thread.
          
          Note, don't need to lock here as the update() function takes care of it.
          
          @param type : connection type
          @type str : string constant from gateway_comms.msg.Connection
          
          @param name : fully qualified topic, service or action name
          @type str
          
          @param node : ros node name (coming from master.getSystemState)
          @type str
          
          @return all the flip rules that match this local connection
          @return list of FlipRule objects updated with node names from self.watchlist
        '''
        matched_flip_rules = []
        for rule in self.watchlist[type]:
            matched = False
            name_match_result = re.match(rule.connection.name, name)
            if name_match_result and name_match_result.group() == name:
                if rule.connection.node:
                    node_match_result = re.match(rule.connection.node,node)
                    if node_match_result and node_match_result.group() == node:
                        matched = True
                else: # rule.connection.node is None so we don't care about matching the node
                    matched = True
            if matched:
                matched_flip = copy.deepcopy(rule)
                matched_flip.connection.name = name # just in case we used a regex
                matched_flip.connection.node = node # just in case we used a regex
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
          
          Basic operation : convert Registration -> FlipRule for each registration
          
          @param connection_type : one of Connection.XXX string constants.
          @type str
          
          @return the list of flip rules corresponding to local flip registrations
          @rtype FlipRule[]
        '''
        flipped_in_rules = []
        for registration in self.registrations[connection_type]:
            flip_rule = FlipRule()
            flip_rule.gateway = registration.remote_gateway
            flip_rule.connection.name = registration.remote_name
            flip_rule.connection.node = registration.remote_node
            flip_rule.connection.type = connection_type
            flipped_in_rules.append(flip_rule)
        return flipped_in_rules
    
if __name__ == "__main__":
    
    r = re.compile("/chatte")
    result = r.match('/chatter')
    print result.group()
    print result.span()
    print len('/chatter')
