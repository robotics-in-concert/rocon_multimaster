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
from gateway_comms.msg import Rule

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
      @type Rule
      
      @param public_rules : list of Rule (public, watchlist or blacklist)
      @type list : list of Rule objects
      
      @return True if the public rule exists, False otherwise
      @rtype bool
    '''
    for rule in public_rules:
        if rule.name == public_rule.name and \
           rule.node == public_rule.node:
            return True
    return False

##############################################################################
# Public Interface
##############################################################################

class PublicInterface(object):
    '''
      The public interface is the set of rules 
      (pubs/subs/services/actions) that are exposed and made available for 
      freely sharing with a multimaster system.
      
      It consists of: 
       * list of currently available rules to be shared 
       * list of rules and filters that will be watched 
         and shared if they become available 
      
    '''
    def __init__(self, default_rule_blacklist):
        '''
          Initialises the public interface
        '''
        # List of rules to be monitored and (un)advertised  as they 
        # become (un)available
        self.watchlist = utils.createEmptyConnectionTypeDictionary()

        # Default rules that cannot be advertised - used in AdvertiseAll mode
        self._default_blacklist = default_rule_blacklist

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
        @type Rule

        @return the rule if added, or None if the rule exists already
        @rtype Rule || None
        '''
        result = None
        self.lock.acquire()
        if not publicRuleExists(rule,self.watchlist[rule.type]):
            self.watchlist[rule.type].append(rule)
            result = rule
        self.lock.release()
        rospy.loginfo("Gateway : (req) advertise %s"%utils.formatRule(rule))
        return result

    def removeRule(self, rule): 
        ''' 
        Attempt to remove a watchlist rule from the public interface. Be a
        bit careful looking for a rule to remove, depending on the node name,
        which can be set (exact rule/node name match) or None in which case all
        nodes of that kind of advertisement will match.
        
        @param rule : a rule to unadvertise
        @type Rule

        @return the list of rules removed
        @rtype Rule[]
        '''

        rospy.loginfo("Gateway : (req) unadvertise %s"%utils.formatRule(rule))

        if rule.node:
            # This looks for *exact* matches.
            try:
                self.lock.acquire()
                self.watchlist[rule.type].remove(rule)
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
            for existing_rule in self.watchlist[rule.type]:
                if (existing_rule.name == rule.name):
                    existing_rules.append(existing_rule)
            for rule in existing_rules:
                self.watchlist[rule.type].remove(existing_rule) # not terribly optimal
            self.lock.release()
            return existing_rules

    def advertiseAll(self, blacklist):
        '''
          Allow all rules apart from the ones in the provided blacklist + 
          default blacklist

          @param blacklist : list of Rule objects
          @type list : list of Rule objects

          @return failure if already advertising all, success otherwise
          @rtype bool
        '''
        rospy.loginfo("Gateway : (req) advertise everything!")
        self.lock.acquire()

        # Check if advertise all already enabled 
        if self.advertise_all_enabled:
            self.lock.release()
            return False
        self.advertise_all_enabled = True

        # generate watchlist
        self.watchlist = utils.createEmptyConnectionTypeDictionary() #easy hack for getting a clean watchlist
        for connection_type in utils.connection_types:
            allow_all_rule = Rule()
            allow_all_rule.name = '.*'
            allow_all_rule.type = connection_type
            self.watchlist[connection_type].append(allow_all_rule)

        # generate blacklist (while making sure only unique rules get added)
        self.blacklist = copy.deepcopy(self._default_blacklist)
        for rule in blacklist:
            if not publicRuleExists(rule, self.blacklist[rule.type]):
                self.blacklist[rule.type].append(rule)

        self.lock.release()
        return True

    def unadvertiseAll(self):
        '''
          Disallow the allow all mode, if enabled. If allow all mode is not
          enabled, remove everything from the public interface
        '''
        rospy.loginfo("Gateway : (req) remove all advertisements!")
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
            list.extend([connection.rule for connection in self.public[connection_type]])
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

    def _matchAgainstRuleList(self,rules,rule):
        '''
          Match a given rule/rule against a given rule list

          @param rules : the rules against which to match
          @type dict of list of Rule objects
          @param rule : the given rule/rule to match
          @type Rule
          @return the list of rules matched, None if no rules found
          @rtype list of Rules || None
        '''
        matched = False
        for r in rules[rule.type]:
            name_match_result = re.match(r.name, rule.name)
            if name_match_result and name_match_result.group() == rule.name:
                if r.node:
                    node_match_result = re.match(r.node, rule.node)
                    if node_match_result and node_match_result.group() == rule.node:
                        matched = True
                else:
                    matched = True
            if matched:
                break
        return matched

    def _allowRule(self,rule):
        '''
          Determines whether a given rule should be allowed given the
          status of the current watchlist and blacklist

          @param rule : the given rule/rule to match
          @type Rule
          @return whether rule is allowed
          @rtype bool
        '''
        self.lock.acquire()
        matched_rules = self._matchAgainstRuleList(self.watchlist,rule)
        matched_blacklisted_rules = self._matchAgainstRuleList(self.blacklist,rule)
        self.lock.release()
        success = False
        if matched_rules and not matched_blacklisted_rules:
            success = True
        return success

    def _generatePublic(self, rule):
        '''
          Given a rule, determines if the rule is allowed. If it is 
          allowed, then returns the corresponding Rule object

          @param rules : the given rules to match
          @type Rule
          @return The generated Rule if allowed, None if no match
          @rtype Rule || None
        '''
        if self._allowRule(rule):
            return Rule(rule)
        return None

    def update(self,connections):
        '''
          Checks a list of rules and determines which ones should be 
          added/removed to the public interface. Modifies the public interface
          accordingly, and returns the list of rules to the gateway for
          hub operations

          @param rules: the list of rules available locally
          @type dict of lists of Rule objects

          @return: new public connections, as well as connections to be removed
          @rtype: Connection[], Connection[]
        '''

        # SLOW, EASY METHOD
        public = utils.createEmptyConnectionTypeDictionary()
        new_public = utils.createEmptyConnectionTypeDictionary()
        removed_public = utils.createEmptyConnectionTypeDictionary()
        diff = lambda l1,l2: [x for x in l1 if x not in l2] # diff of lists
        for connection_type in connections:
            for connection in connections[connection_type]:
                if self._allowRule(connection.rule):
                    public[connection_type].append(connection)
            new_public[connection_type] = diff(public[connection_type],self.public[connection_type])
            removed_public[connection_type] = diff(self.public[connection_type],public[connection_type])

        # print "old public"
        # for connection_type in self.public:
        #     for connection in self.public[connection_type]:
        #         print utils.formatConnection(connection)

        self.lock.acquire()
        self.public = public
        self.lock.release()

        # print "public"
        # for connection_type in public:
        #     for connection in public[connection_type]:
        #         print utils.formatConnection(connection)
        # print "new public"
        # for connection_type in new_public:
        #     for connection in new_public[connection_type]:
        #         print utils.formatConnection(connection)
        # print "removed public"
        # for connection_type in removed_public:
        #     for connection in removed_public[connection_type]:
        #         print utils.formatConnection(connection)

        return new_public, removed_public
