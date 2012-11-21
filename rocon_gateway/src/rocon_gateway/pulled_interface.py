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
import re

# Local imports
import utils
import interactive_interface

##############################################################################
# Flipped Interface
##############################################################################

class PulledInterface(interactive_interface.InteractiveInterface):
    '''
      The flipped interface is the set of rules 
      (pubs/subs/services/actions) and rules controlling flips
      to other gateways. 
    '''
    def __init__(self, default_rule_blacklist, default_rules):
        '''
          Initialises the flipped interface.
          
          @param default_rule_blacklist : used when in flip all mode
          @type dictionary of gateway
          @param default_rules : static rules to launch the interface with
          @type gateway_comms.msg.RemoteRule[]
        '''
        interactive_interface.InteractiveInterface.__init__(self,default_rule_blacklist, default_rules)
        
        # Function aliases
        self.pulled = self.active
        self.pullAll = self.addAll
        self.unPullAll = self.removeAll

    def update(self,connections, gateway = None):
        '''
          Computes a new pulled interface from the incoming connections list
           and returns two dictionaries - 
          removed and newly added pulls so the watcher thread can take
          appropriate action ((un)registrations).
          
          This is run in the watcher thread (warning: take care - other
          additions come from ros service calls in different threads!)
        '''
        # SLOW, EASY METHOD
        #   Totally regenerate a new flipped interface, compare with old
        flipped = utils.createEmptyConnectionTypeDictionary()
        new_flips = utils.createEmptyConnectionTypeDictionary()
        removed_flips = utils.createEmptyConnectionTypeDictionary()
        diff = lambda l1,l2: [x for x in l1 if x not in l2] # diff of lists
        self._lock.acquire()
        for connection_type in connections:
            for connection in connections[connection_type]:
                flipped[connection_type].extend(self._generatePulls(connection.rule.type, connection.rule.name, connection.rule.node, gateway))
            new_flips[connection_type] = diff(flipped[connection_type],self.pulled[connection_type])
            removed_flips[connection_type] = diff(self.pulled[connection_type],flipped[connection_type])
        self.pulled = copy.deepcopy(flipped)
        self._lock.release()
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
        
    def _generatePulls(self, type, name, node, gateway = None):
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
          
          @return all the pull rules that match this local rule
          @return list of RemoteRule objects updated with node names from self.watchlist
        '''
        matched_flip_rules = []
        for rule in self.watchlist[type]:
            # This is a bit different to _generateFlips - does it need to be? DJS
            if gateway and not re.match(rule.gateway,gateway):
                continue
            # Check names
            matched = False
            name_match_result = re.match(rule.rule.name, name)
            if name_match_result and name_match_result.group() == name:
                if utils.isAllPattern(rule.rule.name):
                    if self._isInBlacklist(rule.gateway, type, name, node):
                        continue
                if rule.rule.node:
                    node_match_result = re.match(rule.rule.node,node)
                    if node_match_result and node_match_result.group() == node:
                        matched = True
                else: # rule.rule.node is None so we don't care about matching the node
                    matched = True
            if matched:
                matched_flip = copy.deepcopy(rule)
                matched_flip.rule.name = name # just in case we used a regex
                matched_flip.rule.node = node # just in case we used a regex
                matched_flip_rules.append(matched_flip)
        return matched_flip_rules
    
    ##########################################################################
    # Pulled Interface Specific Methods
    ##########################################################################
    
    def listRemoteGatewayNames(self):
        '''
          Collects all gateways that it should watch for (i.e. those 
          currently handled by existing registrations).
          
          @return set of gateway string ids
          @rtype set of string
        '''
        gateways = []
        for connection_type in utils.connection_types:
            for registration in self.registrations[connection_type]:
                if registration.remote_gateway not in gateways:
                    gateways.append(registration.remote_gateway)
        return gateways
