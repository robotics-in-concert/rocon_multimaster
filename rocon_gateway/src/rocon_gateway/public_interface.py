#!/usr/bin/env python
#       
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_multimaster/master/multimaster_client/rocon_gateway/LICENSE 
#

##############################################################################
# Imports
##############################################################################

import copy
import re
import threading

import rospy

# local imports
import utils
from gateway_comms.msg import PublicRule

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

        self.advertise_all_enabled = False

        self.lock = threading.Lock()

    ##########################################################################
    # Public Interfaces
    ##########################################################################

    def addRule(self, rule):
        '''
        Watch for a new public rule, as described for by the incoming message.
        
        @param rule : a rule msg from the advertise call
        @type PublicRule

        @return the rule if added, or None if the rule exists already
        @rtype PublicRule || None
        '''
        result = None
        self.lock.acquire()
        if not publicRuleExists(rule,self.watchlist[rule.connection.type]):
            self.watchlist[rule.connection.type].append(rule)
            result = rule
        self.lock.release()
        rospy.logdebug("Gateway : (req) advertise %s"%utils.formatRule(rule))
        return result

    def removeRule(self, rule): 
        ''' 
        Attempt to remove a watchlist rule from the public interface. Be a
        bit careful looking for a rule to remove, depending on the node name,
        which can be set (exact rule/node name match) or None in which case all
        nodes of that kind of advertisement will match.
        
        @param rule : a rule to unadvertise
        @type PublicRule

        @return the list of rules removed
        @rtype PublicRule[]
        '''

        rospy.logdebug("Gateway : (req) unadvertise %s"%utils.formatRule(rule))

        if rule.connection.node:
            # This looks for *exact* matches.
            try:
                self.lock.acquire()
                self.watchlist[rule.connection.type].remove(rule)
                self.lock.release()
                return [rule]
            except ValueError:
                self.lock.release()
                return []
        else:
            # This looks for any flip rules which match except for the node name
            # also no need to check for type with the dic keys like they are
            existing_rules = []
            self.lock.acquire()
            for existing_rule in self.watchlist[rule.connection.type]:
                if (existing_rule.connection.name == rule.connection.name):
                    existing_rules.append(existing_rule)
            for rule in existing_rules:
                self.watchlist[rule.connection.type].remove(existing_rule) # not terribly optimal
            self.lock.release()
            return existing_rules

    def advertiseAll(self, blacklist):
        '''
          Allow all rules apart from the ones in the provided blacklist + 
          default blacklist

          @param blacklist : list of PublicRule objects
          @type list : list of PublicRule objects

          @return failure if already advertising all, success otherwise
          @rtype bool
        '''
        rospy.logdebug("Gateway : (req) advertise everything!")
        self.lock.acquire()

        # Check if advertise all already enabled 
        if self.advertise_all_enabled:
            self.lock.release()
            return False
        self.advertise_all_enabled = True

        # generate watchlist
        self.watchlist = utils.createEmptyConnectionTypeDictionary() #easy hack for getting a clean watchlist
        for connection_type in utils.connection_types:
            allow_all_rule = PublicRule()
            allow_all_rule.connection.name = '.*'
            allow_all_rule.connection.type = connection_type
            self.watchlist[connection_type].append(allow_all_rule)

        # generate blacklist (while making sure only unique rules get added)
        self.blacklist = copy.deepcopy(self._default_blacklist)
        for rule in blacklist:
            if not publicRuleExists(rule, self.blacklist[rule.connection.type]):
                self.blacklist[rule.connection.type].append(rule)

        self.lock.release()
        return True

    def unadvertiseAll(self):
        '''
          Disallow the allow all mode, if enabled. If allow all mode is not
          enabled, remove everything from the public interface
        '''
        rospy.logdebug("Gateway : (req) remove all advertisements!")
        self.lock.acquire()

        # stop advertising all
        self.advertise_all_enabled = False

        # easy hack for resetting the watchlist and blacklist
        self.watchlist = utils.createEmptyConnectionTypeDictionary()
        self.blacklist = self._default_blacklist

        self.lock.release()

    ##########################################################################
    # List Accessors
    ##########################################################################

    def getInterface(self):
        list = []
        self.lock.acquire()
        for connection_type in utils.connection_types:
            list.extend(self.public[connection_type])
        self.lock.release()
        return list

    def getWatchlist(self):
        list = []
        self.lock.acquire()
        for connection_type in utils.connection_types:
            list.extend(self.watchlist[connection_type])
        self.lock.release()
        return list

    def getBlacklist(self):
        list = []
        self.lock.acquire()
        for connection_type in utils.connection_types:
            list.extend(self.blacklist[connection_type])
        self.lock.release()
        return list

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
        matched = False
        for r in rules[connection.type]:
            name_match_result = re.match(r.connection.name, connection.name)
            if name_match_result and name_match_result.group() == connection.name:
                if r.connection.node:
                    node_match_result = re.match(r.connection.node, connection.node)
                    if node_match_result and node_match_result.group() == connection.node:
                        matched = True
                else:
                    matched = True
            if matched:
                break
        return matched

    def _allowConnection(self,connection):
        '''
          Determines whether a given connection should be allowed given the
          status of the current watchlist and blacklist

          @param connection : the given connection/rule to match
          @type Connection
          @return whether connection is allowed
          @rtype bool
        '''
        self.lock.acquire()
        matched_rules = self._matchAgainstRuleList(self.watchlist,connection)
        matched_blacklisted_rules = self._matchAgainstRuleList(self.blacklist,connection)
        self.lock.release()
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

          @return: new public connections, as well as connections to be removed
          @rtype: PublicRule[], PublicRule[]
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
        self.lock.acquire()
        self.public = public
        self.lock.release()

        return new_public, removed_publics
