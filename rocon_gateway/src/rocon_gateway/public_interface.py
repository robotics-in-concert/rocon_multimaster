#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_multimaster/license/LICENSE
#

##############################################################################
# Imports
##############################################################################

import copy
import re
# Delete this once we upgrade (hopefully anything after precise)
# Refer to https://github.com/robotics-in-concert/rocon_multimaster/issues/248
import threading
threading._DummyThread._Thread__stop = lambda x: 42

import rospy
from gateway_msgs.msg import Rule

from . import utils

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

    def __init__(self, default_rule_blacklist, default_rules):
        '''
          Initialises the public interface

          @param default_rule_blacklist : connection type keyed dictionary of rules
          @type str keyed dictionary of gateway_msgs.msg.Rule[]

          @param default_rules : connection type keyed dictionary of rules
          @type str keyed dictionary of gateway_msgs.msg.Rule[]
        '''
        # List of rules to be monitored and (un)advertised  as they
        # become (un)available
        self.watchlist = utils.create_empty_connection_type_dictionary()

        # Default rules that cannot be advertised - used in AdvertiseAll mode
        self._default_blacklist = default_rule_blacklist

        # Default + custom blacklist - used in AdvertiseAll mode
        self.blacklist = self._default_blacklist

        # list Rules currently being advertised (gateway_msgs.Rule)
        # should we store utils.Connections instead?
        self.public = utils.create_empty_connection_type_dictionary()

        self.advertise_all_enabled = False

        self.lock = threading.Lock()

        # Load up static rules.
        for connection_type in utils.connection_types:
            for rule in default_rules[connection_type]:
                self.add_rule(rule)

    ##########################################################################
    # Public Interfaces
    ##########################################################################

    def add_rule(self, rule):
        '''
        Watch for a new public rule, as described for by the incoming message.

        @param rule : a rule msg from the advertise call
        @type Rule

        @return the rule if added, or None if the rule exists already
        @rtype Rule || None
        '''
        result = None
        self.lock.acquire()
        if not publicRuleExists(rule, self.watchlist[rule.type]):
            self.watchlist[rule.type].append(rule)
            result = rule
        self.lock.release()
        rospy.loginfo("Gateway : adding rule to public watchlist %s" % utils.format_rule(rule))
        return result

    def remove_rule(self, rule):
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

        rospy.loginfo("Gateway : (req) unadvertise %s" % utils.format_rule(rule))

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
                self.watchlist[rule.type].remove(existing_rule)  # not terribly optimal
            self.lock.release()
            return existing_rules

    def advertise_all(self, blacklist):
        '''
          Allow all rules apart from the ones in the provided blacklist +
          default blacklist

          @param blacklist : list of Rule objects
          @type list : list of Rule objects

          @return failure if already advertising all, success otherwise
          @rtype bool
        '''
        rospy.loginfo("Gateway : received a request advertise everything!")
        self.lock.acquire()

        # Check if advertise all already enabled
        if self.advertise_all_enabled:
            self.lock.release()
            return False
        self.advertise_all_enabled = True

        # generate watchlist
        self.watchlist = utils.create_empty_connection_type_dictionary()  # easy hack for getting a clean watchlist
        for connection_type in utils.connection_types:
            allow_all_rule = Rule()
            allow_all_rule.name = '.*'
            allow_all_rule.type = connection_type
            allow_all_rule.node = '.*'
            self.watchlist[connection_type].append(allow_all_rule)

        # generate blacklist (while making sure only unique rules get added)
        self.blacklist = copy.deepcopy(self._default_blacklist)
        for rule in blacklist:
            if not publicRuleExists(rule, self.blacklist[rule.type]):
                self.blacklist[rule.type].append(rule)

        self.lock.release()
        return True

    def unadvertise_all(self):
        '''
          Disallow the allow all mode, if enabled. If allow all mode is not
          enabled, remove everything from the public interface
        '''
        rospy.loginfo("Gateway : received a request to remove all advertisements!")
        self.lock.acquire()

        # stop advertising all
        self.advertise_all_enabled = False

        # easy hack for resetting the watchlist and blacklist
        self.watchlist = utils.create_empty_connection_type_dictionary()
        self.blacklist = self._default_blacklist

        self.lock.release()

    ##########################################################################
    # List Accessors
    ##########################################################################

    def getConnections(self):
        '''
          List of all rules with connection information that is being published.

          @return dictionary of utils.Connections keyed by type.
        '''
        return self.public

    def getInterface(self):
        '''
          List of all rules currently being advertised.

          @return list of all connections posted on hubs
          @rtype list of gateway_msgs.msg.Rule
        '''
        l = []
        self.lock.acquire()
        for connection_type in utils.connection_types:
            l.extend([connection.rule for connection in self.public[connection_type]])
        self.lock.release()
        return l

    def getWatchlist(self):
        l = []
        self.lock.acquire()
        for connection_type in utils.connection_types:
            l.extend(self.watchlist[connection_type])
        self.lock.release()
        return l

    def getBlacklist(self):
        l = []
        self.lock.acquire()
        for connection_type in utils.connection_types:
            l.extend(self.blacklist[connection_type])
        self.lock.release()
        return l

    ##########################################################################
    # Filter
    ##########################################################################

    def _matchAgainstRuleList(self, rules, rule):
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

    def _allowRule(self, rule):
        '''
          Determines whether a given rule should be allowed given the
          status of the current watchlist and blacklist

          @param rule : the given rule/rule to match
          @type Rule
          @return whether rule is allowed
          @rtype bool
        '''
        self.lock.acquire()
        matched_rules = self._matchAgainstRuleList(self.watchlist, rule)
        matched_blacklisted_rules = self._matchAgainstRuleList(self.blacklist, rule)
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

    def update(self, connections, generate_advertisement_connection_details):
        '''
          Checks a list of rules and determines which ones should be
          added/removed to the public interface. Modifies the public interface
          accordingly, and returns the list of rules to the gateway for
          hub operations

          @param rules: the list of rules available locally
          @type dict of lists of Rule objects

          @param generate_advertisement_connection_details : function from LocalMaster
          that generates Connection.type_info and Connection.xmlrpc_uri
          @type method (see LocalMaster.generate_advertisement_connection_details)

          @return: new public connections, as well as connections to be removed
          @rtype: utils.Connection[], utils.Connection[]
        '''
        # SLOW, EASY METHOD
        permitted_connections = utils.create_empty_connection_type_dictionary()
        new_public = utils.create_empty_connection_type_dictionary()
        removed_public = utils.create_empty_connection_type_dictionary()
        for connection_type in utils.connection_types:
            for connection in connections[connection_type]:
                if self._allowRule(connection.rule):
                    permitted_connections[connection_type].append(connection)
        self.lock.acquire()  # protect self.public
        for connection_type in utils.connection_types:
            for connection in permitted_connections[connection_type]:
                if not connection.inConnectionList(self.public[connection_type]):
                    new_connection = generate_advertisement_connection_details(
                        connection.rule.type, connection.rule.name, connection.rule.node)
                    # can happen if connection disappeared in between getting the connection
                    # index (watcher thread) and checking for the topic_type (preceding line)
                    if new_connection is not None:
                        new_public[connection_type].append(new_connection)
                        self.public[connection_type].append(new_connection)
            removed_public[connection_type][:] = [
                x for x in self.public[connection_type] if not x.inConnectionList(
                    permitted_connections[connection_type])]
            self.public[connection_type][:] = [
                x for x in self.public[connection_type] if not x.inConnectionList(removed_public[connection_type])]
        self.lock.release()
        return new_public, removed_public
