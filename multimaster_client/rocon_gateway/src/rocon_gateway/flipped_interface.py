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

class FlipRule(object):
    '''
      Holds a rule defining the properties for a single connection flip.
    '''
    def __init__(self, gateway, type, name, node_name = None, remapped_name = None):
        self.gateway = gateway
        self.type = type
        self.name = name
        self.node_name = node_name
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
        return hash(self.gateway) ^ hash(self.type) ^ hash(self.name) ^ hash(self.node_name) ^ hash(self.remapped_name)
    
    def __repr__(self):
        return '{ gateway: %s type: %s name: %s node name: %s remap: %s }'%(self.gateway,self.type,self.name, self.node_name, self.remapped_name)
        
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
        self.flipped = set() # set of currently flipped, connection string representations
        self.rules = set() # set of FlipRule objects
        self.patterns = set()  # set of FlipPattern objects
        
    def addRule(self, gateway, type, name, node_name, remapped_name):
        '''
          Generate the flip rule, taking care to provide a sensible
          default for the remapping (root it in this gateway's namespace
          on the remote system).
          
          @param gateway, type, name, node_name, remapped_name
          @type str
          
          @param type : must be a string constant from gateway_comms.msg.Connection
          
          @return the flip rule, or None if the rule already exists.
          @rtype FlipRule || None
        '''
        if not remapped_name:
            remapped_name = self.namespace + name
        rule = FlipRule(gateway, type, name, node_name, remapped_name)
        if rule in self.rules:
            return None
        else:
            self.rules.add(rule)
            return rule
        
    def rulesByType(self):
        sorted_watchlist = {}
        sorted_watchlist[gateway_comms.Connection.PUBLISHER] = [] 
        sorted_watchlist[gateway_comms.Connection.SUBSCRIBER] = [] 
        sorted_watchlist[gateway_comms.Connection.SERVICE] = [] 
        sorted_watchlist[gateway_comms.Connection.ACTION_CLIENT] = [] 
        sorted_watchlist[gateway_comms.Connection.ACTION_SERVER] = []
        return sorted_watchlist 
    
    def watchlistByGateway(self):
        pass
        
    def addRule(self, gateway, type, name, node_name, remapped_name):
        '''
          Generate the flip rule, taking care to provide a sensible
          default for the remapping (root it in this gateway's namespace
          on the remote system).
          
          @param gateway, type, name, node_name, remapped_name
          @type str
          
          @param type : must be a string constant from gateway_comms.msg.Connection
          
          @return the flip rule, or None if the rule already exists.
          @rtype FlipRule || None
        '''
        if not remapped_name:
            remapped_name = self.namespace + name
        rule = FlipRule(gateway, type, name, node_name, remapped_name)
        if rule in self.watchlist:
            return None
        else:
            self.watchlist.add(rule)
            return rule
        
    ##########################################################################
    # Flipped Interfaces
    ##########################################################################

    ##########################################################################
    # Filters
    ##########################################################################

    # ToDo

if __name__ == "__main__":
    
    # Ugh, sets and hashes....!
    flip1 = FlipRule("gateway1","publisher","/chatter", None,"/gateway1/chatter")
    flip2 = FlipRule("gateway1","publisher","/chatter", None,"/gateway1/chatter")
    
    print ""
    print flip1
    print ""
    print flip2
    print ""
    if flip1 == flip2:
        print "eq"
    else:
        print "ne"
    print ""
    
    flipped = set()
    flipped.add(flip1)
    for flip in flipped:
        if flip == flip2:
            print "bugger is the same"
        else:
            print "bugger is the same not"
            print flip
            print ""
            print flip2
    print ""
    if flip2 in flipped:
        print "already exists"
    else:
        print "does not exist"
    print ""
    flipped.add(flip2)
    print flipped
    print ""

    str1 = "dude"
    str2 = "dude"
    if str1 == str2:
        print "eq"
    else:
        print "ne"
    print ""
    
    stringed = set()
    stringed.add(str1)
    if str2 in stringed:
        print "already exists"
    else:
        print "does not exist"
    print ""
        
    
    
