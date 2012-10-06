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
           rule.remapped_name   == flip_rule.remapped_name and \
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
    def __init__(self):
        '''
          Initialises the flipped interface.
        '''
        self._namespace = "/" # namespace to root flips in
        self.flipped = utils.createEmptyConnectionTypeDictionary() # keys are connection_types, elements are lists of FlipRule objects 
        self.rules = utils.createEmptyConnectionTypeDictionary()
        self.patterns = utils.createEmptyConnectionTypeDictionary()
        
    def setDefaultRootNamespace(self, namespace):
        '''
          The namespace is used as a default setting to root 
          flips in the remote gateway's workspace (helps avoid conflicts)
          when no remapping argument is provided.
          
          @param namespace : default namespace to root flipped connections into
          @type str
        '''
        self._namespace = "/"+namespace

    def addRule(self, flip_rule):
        '''
          Generate the flip rule, taking care to provide a sensible
          default for the remapping (root it in this gateway's namespace
          on the remote system).
          
          @param gateway, type, name, node, remapped_name
          @type str
          
          @param type : connection type
          @type str : string constant from gateway_comms.msg.Connection
          
          @return the flip rule, or None if the rule already exists.
          @rtype Flip || None
        '''
        if not flip_rule.remapped_name:
            flip_rule.remapped_name = self._namespace + flip_rule.connection.name
        if flipRuleExists(flip_rule, self.rules[flip_rule.connection.type]):
            return None
        else:
            self.rules[flip_rule.connection.type].append(flip_rule)
            return flip_rule

    def matchesRule(self, type, name, node):
        '''
          Checks if a local connection (obtained from master.getSystemState) 
          is a suitable association with any of the rules. This can
          return multiple matches, since the same local connection 
          properties can be multiply flipped to different remote gateways.
            
          Used in the watcher thread.
          
          @param type : connection type
          @type str : string constant from gateway_comms.msg.Connection
          
          @param name : fully qualified topic, service or action name
          @type str
          
          @param node : ros node name (coming from master.getSystemState)
          @type str
          
          @return all the flip rules that match this local connection
          @return list of FlipRule copied objects from self.rules
        '''
        matched_flip_rules = []
        for rule in self.rules[type]:
            if name == rule.connection.name:
                if rule.connection.node and node == rule.connection.node:
                    matched_flip_rules.append(copy.deepcopy(rule))
                elif not rule.connection.node:
                    matched_flip = copy.deepcopy(rule)
                    matched_flip.connection.node = node
                    matched_flip_rules.append(matched_flip)
        return matched_flip_rules
                    
    def isFlipped(self, type, name, node):
        '''
          Checks if a local connection (obtained from master.getSystemState)
          is currently flipped and returns a list of the flipped instances
          (note that the same local connection may be multiply flipped
          to different remote gateways). 
          
          Used in the watcher thread. 
          
          @param type : connection type
          @type str : string constant from gateway_comms.msg.Connection
          
          @return all the flips that originate from this local connection
          @rtype list : list of FlipRule copied objects from self.flipped 
        '''
        matched_flips = []
        for flip in self.flipped[type]:
            if name == flip.connection.name and node == flip.connection.node:
                matched_flips.append(copy.deepcopy(flip))
        return matched_flips
        
    ##########################################################################
    # Utilities
    ##########################################################################
    
if __name__ == "__main__":
    
    flips = []
    dudes = ["dude","dudette"]
    if flips:
        print "true"
    else:
        print "false"
    print ""
    flips.extend(dudes)
    print dudes
    print flips
    if flips:
        print "true"
    else:
        print "false"
