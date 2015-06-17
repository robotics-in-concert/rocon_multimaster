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
import rocon_gateway_utils

from . import utils
from . import interactive_interface

##############################################################################
# Pulled Interface
##############################################################################


class PulledInterface(interactive_interface.InteractiveInterface):

    '''
      The pulled interface is the set of rules
      (pubs/subs/services/actions) and rules controlling pulls from
      other gateways.
    '''

    def __init__(self, default_rule_blacklist, default_rules, all_targets):
        '''
          Initialises the pulled interface.

          @param default_rule_blacklist : used when in flip all mode
          @type dictionary of gateway
          @param default_rules : static rules to pull on startup
          @type gateway_msgs.msg.RemoteRule[]
          @param all_targets : static pull all targets to pull to on startup
          @type string[]
        '''
        interactive_interface.InteractiveInterface.__init__(self, default_rule_blacklist, default_rules, all_targets)

        # Function aliases
        self.pulled = self.active
        self.pull_all = self.add_all
        self.unpull_all = self.remove_all

    def update(self, remote_connections, unique_name):
        '''
          Computes a new pulled interface from the incoming connections list
           and returns two dictionaries -
          removed and newly added pulls so the watcher thread can take
          appropriate action ((un)registrations).

          This is run in the watcher thread (warning: take care - other
          additions come from ros service calls in different threads!)

          @param remote_gateway_hub_index : full gateway-hub database index to parse
          @type gateway hash names keyed into a dic with a list of their hubs
        '''
        # SLOW, EASY METHOD
        #   Totally regenerate a new pulled interface, compare with old
        pulled = utils.create_empty_connection_type_dictionary()
        new_pulls = utils.create_empty_connection_type_dictionary()
        removed_pulls = utils.create_empty_connection_type_dictionary()

        self._lock.acquire()
        # Totally regenerate a new pulled interface, compare with old
        for remote_gateway in remote_connections.keys():
            connections = remote_connections[remote_gateway]
            for connection_type in connections:
                for connection in connections[connection_type]:
                    pulled[connection_type].extend(
                        self._generate_pulls(
                            connection.rule.type,
                            connection.rule.name,
                            connection.rule.node,
                            remote_gateway,
                            unique_name))
        for connection_type in utils.connection_types:
            new_pulls[connection_type] = utils.difflist(pulled[connection_type], self.pulled[connection_type])
            removed_pulls[connection_type] = utils.difflist(self.pulled[connection_type], pulled[connection_type])
        self.pulled = copy.deepcopy(pulled)
        self._lock.release()
        return new_pulls, removed_pulls

        # OPTIMISED METHOD
        #   Keep old rule state and old flip rules/patterns around
        #
        #   1 - If flip rules/patterns disappeared [diff(old_rules,new_rules)]
        #         Check if the current flips are still valid
        #         If not all are, remove and unflip them
        #
        #   2 - If rules disappeared [diff(old_conns,new_conns)]
        #         If matching any in pulled, remove and unflip
        #
        #   3 - If flip rules/patterns appeared [diff(new_rules,old_rules)]
        #         parse all conns, if match found, flip
        #
        #   4 - If rules appeared [diff(new_conns,old_conns)]
        #         check for matches, if found, flou
        #
        # difflist = lambda l1,l2: [x for x in l1 if x not in l2] # diff of lists

    ##########################################################################
    # Utility Methods
    ##########################################################################

    def _generate_pulls(self, connection_type, name, node, gateway, unique_name):
        '''
          Checks if a local rule (obtained from master.get_system_state)
          is a suitable association with any of the rules or patterns. This can
          return multiple matches, since the same local rule
          properties can be multiply pulled to different remote gateways.

          Used in the update() call above that is run in the watcher thread.

          Note, don't need to lock here as the update() function takes care of it.

          @param connection_type : rule type
          @type str : string constant from gateway_msgs.Rule

          @param name : fully qualified topic, service or action name
          @type str

          @param node : ros node name (coming from master.get_system_state)
          @type str

          @param gateway : remote gateway hash name.
          @type str

          @return all the pull rules that match this local rule
          @return list of RemoteRule objects updated with node names from self.watchlist
        '''
        matched_pull_rules = []
        for rule in self.watchlist[connection_type]:
            # check for regular expression or perfect match
            gateway_match_result = re.match(rule.gateway, gateway)
            matched = False
            if gateway_match_result and gateway_match_result.group() == gateway:
                matched = True
            elif rule.gateway == rocon_gateway_utils.gateway_basename(gateway):
                matched = True
            if not matched:
                continue

            # Check names
            rule_name = rule.rule.name
            matched = self.is_matched(rule, rule_name, name, node)
            if not matched:
                rule_name = '/' + unique_name + '/' + rule.rule.name
                matched = self.is_matched(rule, rule_name, name, node)

            if not matched:
                rule_name = '/' + rule.rule.name
                matched = self.is_matched(rule, rule_name, name, node)

            if matched:
                matched_pull = copy.deepcopy(rule)
                matched_pull.gateway = gateway  # just in case we used a regex or matched basename
                matched_pull.rule.name = name   # just in case we used a regex
                matched_pull.rule.node = node   # just in case we used a regex
                matched_pull_rules.append(matched_pull)
        return matched_pull_rules

    ##########################################################################
    # Pulled Interface Specific Methods
    ##########################################################################

    def list_remote_gateway_names(self):
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
