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

def parsePublicRuleFromYamlObject(yaml_obj):
    '''
      Parse a connection object from a dictionary extracted from a YAML
      file. Only the type/name are required fields, and node is optional.

      @param yaml_obj : a dictionary representing a PublicRule object
      @type dict
    '''
    connection = yaml_obj['connection']
    node = connection['node']
    if node == '':
        node = None
    return PublicRule(Connection(connection['type'],connection['name'],node))

def parsePublicRulesFromFile(self,file):
    '''
      Parse a YAML file for a list of Public rule object.

      @param file : absolute file location of YAML file
      @type str
      @return a list of PublicRule objects extracted from the YAML file, None on
              error
      @rtype list of PublicRule objects || None
    '''
    try:
        stream = open(file, 'r')
        list = yaml.load(stream)
    except:
        rospy.logerr('Gateway : Unable to load yaml from file [%s]'%file)
        return None

    rules = utils.createEmptyConnectionTypeDictionary()
    try:
        for l in list:
            rule = self._parseRuleFromYamlObject(l)
            rules[rule.connection.type].append(rule)
    except Exception as e:
        rospy.logerr('Gateway : Error parsing item in yaml file [%s]'%str(e))
        return None
    return rules

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
    def __init__(self):
        '''
          Initialises the public interface
        '''
        # List of connections that should be monitored, and flipped when they
        # become available
        self.watchlist = utils.createEmptyConnectionTypeDictionary()

        # Rules that cannot be flipped - regardless of whether they are in the
        # watchlist or not
        self._default_blacklist = utils.createEmptyConnectionTypeDictionary()

        # Overall blacklist used while flipping everything
        self.blacklist = self._default_blacklist

        # list of fully qualified connections currently being flipped
        # the public interface can be manipulated from both the main gateway
        # thread and the watcher thread
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

        rospy.loginfo("Gateway : advertising %s"%utils.formatRule(rule))
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
        rospy.loginfo("Gateway : removing advertisement of %s"%utils.formatRule(rule))
        return rule

    def allowAll(self, blacklist = None):
        '''
        Allow all rules apart from the ones in the provided blacklist + default
        blacklist

        @param blacklist : list of PublicRule objects
        @type list : list of PublicRule objects
        '''
        self.watchlist = []
        allow_all_rule = PublicRule()
        allow_all_rule.connection.name = '.*'
        for connection_type in utils.connection_types:
            allow_all_rule.connection.type = connection_type
            self.watchlist[connection_type].add(allow_all_rule)
        self.blacklist = copy.deepcopy(self._default_blacklist)
        for b in blacklist:
            if not publicRuleExists(b, self.blacklist[b.connection.type]):
                self.blacklist[b.connection.type].add(b)

    def disallowAll(self):
        '''
        Disallow all rules in watchlist, reset blacklist to default
        '''
        self.watchlist = []
        self.blacklist = self._default_blacklist

    # def addConnection(self,connection):
    #     '''
    #     Add a unique connection to the public interface

    #     @param rule : a rule msg containing the connection to be advertised
    #     @type PublicRule
    #     @return the connection if added, or None if the connection exists already
    #     @rtype PublicRule || None
    #     '''
    #     if publicRuleExists(connection,self.public[connection.connection.type]):
    #         return False

    #     rospy.loginfo("Gateway : adding connection to public interface %s"%self.formatConnection(connection.connection))
    #     self.public[connection.connection.type].add(connection)
    #     return connection

    # def removeConnection(self,connection):
    #     '''
    #     Remove a unique connection to the public interface

    #     @param rule : a rule msg containing the connection to be advertised
    #     @type PublicRule
    #     @return the connection if added, or None if the connection exists already
    #     @rtype PublicRule || None
    #     '''
    #     if connection not in self.public[connection.connection.type]:
    #         return None

    #     rospy.loginfo("Gateway : removing connection from public interface %s"%self.formatConnection(connection))
    #     self.public[connection.connection.type].remove(connection)
    #     return True

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
        rules = parsePublicRulesFromFile(file)
        if rules:
            self.watchlist = rules
            rospy.loginfo('Gateway : Default public interface parsed from yaml file [%s]'%file)
            return True
        else:
            rospy.logerr('Gateway : Error parsing default public interface from yaml file [%s]'%file)
            return False

    def parseBlacklistFromFile(self,file):
        '''
          Parse a YAML file for the default blacklist

          @param file : absolute file location of YAML file
          @type str
        '''
        rules = parsePublicRulesFromFile(file)
        if rules:
            rospy.loginfo('Gateway : Default blacklist parsed from yaml file [%s]'%file)
            self._default_blacklist = rules
            return True
        else:
            rospy.logerr('Gateway : Error parsing default blacklist from yaml file [%s]'%file)
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
            if re.match(r.connection.name, connection.name):
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

