#!/usr/bin/env python
#       
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_multimaster/master/multimaster_client/rocon_gateway/LICENSE 
#

##############################################################################
# Imports
##############################################################################

##############################################################################
# Flip
##############################################################################

class Flip(object):
    '''
      Holds a rule defining the properties for a single connection flip.
    '''
    def __init__(self, gateway, type, name, node = None, remapped_name = None):
        self.gateway = gateway
        self.type = type
        self.name = name
        self.node = node
        self.remapped_name = remapped_name
    
    # Need these for hashable containers (like sets), ugh!
    def __eq__(self, other):
        if isinstance(other, self.__class__):
            return self.__dict__ == other.__dict__
        else:
            return False
    
    def __ne__(self, other):
        return not self.__eq__(other)
    
    def __hash__(self):
        return hash(self.gateway) ^ hash(self.type) ^ hash(self.name) ^ hash(self.node) ^ hash(self.remapped_name)
    
    def __repr__(self):
        return '{ gateway: %s type: %s name: %s node name: %s remap: %s }'%(self.gateway,self.type,self.name, self.node, self.remapped_name)
    
    def copy(self):
        return Flip(self.gateway, self.type, self.name, self.node, self.remapped_name)
        
class FlipPattern(object):
    '''
      Holds a rule defining the properties for flips generated from a 
      single regex based connection name pattern.
    '''
    def __init__(self, type, regex, remapped_namespace = None):
        self.gateway = gateway
        self.type = type
        self.regex = regex
        self.remapped_namespace = remapped_namespace

##############################################################################
# Flipped Interface
##############################################################################

class FlippedInterface(object):
    '''
      The flipped interface is the set of connections 
      (pubs/subs/services/actions) and rules controlling flips
      to other gateways. 
    '''
    def __init__(self, namespace):
        '''
          Initialises the flipped interface.
          
          Note that the namespace is used as a default setting to root 
          flips in the remote gateway's workspace (helps avoid conflicts)
          when no remapping argument is provided.
          
          @param namespace : default namespace to root flipped connections into
          @type str
        '''
        self.namespace = "/"+namespace
        self.flipped = set() # set of currently flipped Flips
        self.rules = set() # set of Flip rules
        self.patterns = set()  # set of FlipPattern rules
        
    def addRule(self, gateway, type, name, node, remapped_name):
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
        if not remapped_name:
            remapped_name = self.namespace + name
        rule = Flip(gateway, type, name, node, remapped_name)
        if rule in self.rules:
            return None
        else:
            self.rules.add(rule)
            return rule

    def matchesRule(self, type, name, node):
        '''
          Checks if a local connection (obtained from master.getSystemState) matches a rule.
          
          @param type : connection type
          @type str : string constant from gateway_comms.msg.Connection
          
          @param name : fully qualified topic, service or action name
          @type str
          
          @param node : ros node name (coming from master.getSystemState)
          @type str
        '''
        matched_flip = None
        for rule in self.rules:
            if type == rule.type and name == rule.name:
                if rule.node and node == rule.node:
                    matched_flip = rule.copy()
                    break
                elif not rule.node:
                    matched_flip = rule.copy()
                    matched_flip.node = node
                    break
        return matched_flip
                    
    def isFlipped(self, type, name, node):
        '''
          Checks if this flip is currently flipped.
          
          @param type : connection type
          @type str : string constant from gateway_comms.msg.Connection
          
          @return The flip rule if found, nothing otherwise
          @rtype Flip || None
        '''
        for flip in self.flipped:
            if type == flip.type and name == flip.name and node == flip.node:
                return flip
        return None
        
    ##########################################################################
    # Flipped Interfaces
    ##########################################################################

    ##########################################################################
    # Filters
    ##########################################################################

    # ToDo

if __name__ == "__main__":
    
    print "yeah" 
    
    