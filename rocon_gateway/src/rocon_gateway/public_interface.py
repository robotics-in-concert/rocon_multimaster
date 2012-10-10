#!/usr/bin/env python
#       
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_multimaster/master/multimaster_client/rocon_gateway/LICENSE 
#

##############################################################################
# Imports
##############################################################################

import copy
import yaml
import re

import rospy

# local imports
import utils
from gateway_comms.msg import PublicRule, Connection 

##############################################################################
# Functions
##############################################################################

def publicRuleExists(public_rule, public_rules):
    '''
      Checks that the public rule doesn't already exist in the list of public
      rules (which can represent the public interface or the rules themselves).
      We only need to compare the name/node as they uniquely identify the 
      name/regex
      
      @param public_rule : the rule to search for
      @type PublicRule
      
      @param public_rules : list of PublicRule (public, watchlist or blacklist)
      @type list : list of PublicRule objects
      
      @return True if the public rule exists, False otherwise
      @rtype bool
    '''
    for rule in public_rules:
        if rule.connection.name == public_rule.connection.name and \
           rule.connection.node == public_rule.connection.node:
            return True
    return False

def addUniquePublicRules(to_list, from_list):
    '''
      An inefficient but safe method of adding public rules while maintaining 
      uniqueness of the list.

      @param to_list : list to which rules need to be added
      @type : list of PublicRule objects

      @param from_list : list from which rules are added (may not be unique)
      @type : list of PublicRule obejcts
    '''
    for rule in from_list:
        if not publicRuleExists(to_list):
            to_list.add(rule)

def generatePublicRulesFromConnections(connections):
    '''
      Generate a public rules dictionary directly from a 
      connections dictionary.
    '''
    public_rules = utils.createEmptyConnectionTypeDictionary()
    if connections:
        for connection_type in utils.connection_types:
            for connection in connections[connection_type]:
                public_rules[connection_type].append(PublicRule(connection))
    return public_rules

##############################################################################
# Public Interface
##############################################################################

class PublicInterface(object):
    '''
      The public interface is the set of connections 
      (pubs/subs/services/actions) that are exposed and made available for 
      freely sharing with a multimaster system.
      
      It consists of: 
       * list of currently available connections to be shared 
       * list of connections and filters that will be watched 
         and shared if they become available 
      
    '''
    def __init__(self, default_connection_blacklist):
        '''
          Initialises the public interface
        '''
        # List of connections to be monitored and (un)advertised  as they 
        # become (un)available
        self.watchlist = utils.createEmptyConnectionTypeDictionary()

        # Default rules that cannot be advertised - used in AdvertiseAll mode
        self._default_blacklist = generatePublicRulesFromConnections(default_connection_blacklist)

        # Default + custom blacklist - used in AdvertiseAll mode
        self.blacklist = self._default_blacklist

        # list of fully qualified connections currently being advertised
        self.public = utils.createEmptyConnectionTypeDictionary()

    ##########################################################################
    # Public Interfaces
    ##########################################################################

    def addRule(self,rule):
        '''
        Watch for a new public rule, as described for by the incoming message.
        
        @param rule : a rule msg from the advertise call
        @type PublicRule
        @return the rule if added, or None if the rule exists already
        @rtype PublicRule || None
        '''
        if publicRuleExists(rule,self.watchlist[rule.connection.type]):
            return None

        rospy.loginfo("Gateway : (req) advertise %s"%utils.formatRule(rule))
        self.watchlist[rule.connection.type].append(rule)
        return rule

    def removeRule(self,rule):
        '''
        Attempt to remove a watchlist rule from the public interface. Removes
        only an exact match
        
        @param rule : a rule msg from the advertise call
        @type PublicRule
        @return the rule if removed, or None if the rule exists already
        @rtype PublicRule || None
        '''
        if rule not in self.watchlist[rule.connection.type]:
            return None

        self.watchlist[rule.connection.type].remove(rule)
        rospy.loginfo("Gateway : (req) remove advertisement of %s"%utils.formatRule(rule))
        return rule

    def allowAll(self, blacklist = None):
        '''
        Allow all rules apart from the ones in the provided blacklist + default
        blacklist

        @param blacklist : list of PublicRule objects
        @type list : list of PublicRule objects
        '''
        rospy.loginfo("Gateway : (req) advertise everything!")
        self.watchlist = utils.createEmptyConnectionTypeDictionary()
        allow_all_rule = PublicRule()
        allow_all_rule.connection.name = '.*'
        for connection_type in utils.connection_types:
            allow_all_rule.connection.type = connection_type
            self.watchlist[connection_type].append(allow_all_rule)
        self.blacklist = copy.deepcopy(self._default_blacklist)
        for b in blacklist:
            if not publicRuleExists(b, self.blacklist[b.connection.type]):
                self.blacklist[b.connection.type].append(b)

    def disallowAll(self, blacklist = None):
        '''
        Disallow all rules in watchlist, reset blacklist to default
        '''
        rospy.loginfo("Gateway : (req) remove all advertisements!")
        self.watchlist = utils.createEmptyConnectionTypeDictionary()
        self.blacklist = self._default_blacklist

    ##########################################################################
    # List getters
    ##########################################################################

    def getInterface(self):
        list = []
        for connection_type in utils.connection_types:
            list.extend(self.public[connection_type])
        return list

    def getWatchlist(self):
        list = []
        for connection_type in utils.connection_types:
            list.extend(self.watchlist[connection_type])
        return list

    def getBlacklist(self):
        list = []
        for connection_type in utils.connection_types:
            list.extend(self.blacklist[connection_type])
        return list

    ##########################################################################
    # File operations
    ##########################################################################

    def parseDefaultRulesFromFile(self,file):
        '''
          Parse a YAML file for the default Public Interface

          @param file : absolute file location of YAML file
          @type str
        '''
        connections = parseConnectionsFromFile(file)
        self.watchlist = generatePublicRulesFromConnections(connections)
        if connections:
            rospy.loginfo('Gateway : Default public interface parsed from yaml file [%s]'%file)
            return True
        else:
            rospy.logerr('Gateway : Error parsing default public interface from yaml file [%s]'%file)
            return False

    ##########################################################################
    # Filter
    ##########################################################################

    def _matchAgainstRuleList(self,rules,connection):
        '''
          Match a given connection/rule against a given rule list

          @param rules : the rules against which to match
          @type dict of list of PublicRule objects
          @param connection : the given connection/rule to match
          @type Connection
          @return the list of rules matched, None if no rules found
          @rtype list of PublicRules || None
        '''
        ret_list = []
        for r in rules[connection.type]:
            match_result = re.match(r.connection.name, connection.name)
            if match_result and match_result.end() == len(connection.name):
                if not r.connection.node or \
                   r.connection.node == connection.node:
                    ret_list.append(r);
        if len(ret_list) == 0:
            ret_list = None
        return ret_list

    def _allowConnection(self,connection):
        '''
          Determines whether a given connection should be allowed given the
          status of the current watchlist and blacklist

          @param connection : the given connection/rule to match
          @type Connection
          @return whether connection is allowed
          @rtype bool
        '''
        matched_rules = self._matchAgainstRuleList(self.watchlist,connection)
        matched_blacklisted_rules = self._matchAgainstRuleList(self.blacklist,connection)
        success = False
        if matched_rules and not matched_blacklisted_rules:
            success = True
        return success

    def _generatePublic(self, connection):
        '''
          Given a connection, determines if the connection is allowed. If it is 
          allowed, then returns the corresponding PublicRule object

          @param connections : the given connections to match
          @type Connection
          @return The generated PublicRule if allowed, None if no match
          @rtype PublicRule || None
        '''
        if self._allowConnection(connection):
            return PublicRule(connection)
        return None

    def update(self,connections):
        '''
          Checks a list of connections and determines which ones should be 
          added/removed to the public interface. Modifies the public interface
          accordingly, and returns the list of connections to the gateway for
          hub operations

          @param connections: the list of connections available locally
          @type dict of lists of Connection objects
        '''

        # SLOW, EASY METHOD
        public = utils.createEmptyConnectionTypeDictionary()
        new_public = utils.createEmptyConnectionTypeDictionary()
        removed_publics = utils.createEmptyConnectionTypeDictionary()
        diff = lambda l1,l2: [x for x in l1 if x not in l2] # diff of lists
        for connection_type in connections:
            for connection in connections[connection_type]:
                public_connection = self._generatePublic(connection)
                if public_connection:
                    public[connection_type].append(public_connection)
            new_public[connection_type] = diff(public[connection_type],self.public[connection_type])
            removed_publics[connection_type] = diff(self.public[connection_type],public[connection_type])
        self.public = public

        return new_public, removed_publics

        
