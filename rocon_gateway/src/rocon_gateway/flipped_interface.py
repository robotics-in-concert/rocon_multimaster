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

import rospy
import rosgraph
import rocon_gateway_utils
from gateway_msgs.msg import RemoteRuleWithStatus

from . import utils
from . import interactive_interface

##############################################################################
# Flipped Interface
##############################################################################


class FlippedInterface(interactive_interface.InteractiveInterface):

    '''
      The flipped interface is the set of rules
      (pubs/subs/services/actions) and rules controlling flips
      to other gateways.
    '''

    def __init__(self, firewall, default_rule_blacklist, default_rules, all_targets):
        '''
          Initialises the flipped interface.

          @param firewall : flag to prevent this gateway from accepting flips
          @type Bool
          @param default_rule_blacklist : used when in flip all mode
          @type dictionary of gateway
          @param default_rules : static rules to flip on startup
          @type gateway_msgs.msg.RemoteRule[]
          @param all_targets : static flip all targets to flip to on startup
          @type string[]

        '''
        interactive_interface.InteractiveInterface.__init__(self, default_rule_blacklist, default_rules, all_targets)

        self.firewall = firewall

        # Function aliases
        self.flipped = self.active
        self.filtered_flips = []
        self.flip_status = utils.create_empty_connection_type_dictionary()
        self.flip_all = self.add_all
        self.unflip_all = self.remove_all

    ##########################################################################
    # Monitoring
    ##########################################################################

    def update(self, connections, remote_gateway_hub_index, unique_name, master):
        '''
          Computes a new flipped interface and returns two dictionaries -
          removed and newly added flips so the watcher thread can take
          appropriate action (inform the remote gateways).

          This is run in the watcher thread (warning: take care - other
          additions come from ros service calls in different threads!)

          @param connections : list of all the system state connections from the local master
          @type connection type keyed dictionary of utils.Connection lists.

          @param remote_gateway_hub_index : full gateway-hub database index to parse
          @type gateway hash names keyed into a dic with a list of their hubs

          @param unique_name : this gateway's unique hash name
          @type string
        
          @param master : local master
          @type rocon_gateway.LocalMaster

          @return new_flips, removed_flips (i.e. those that are no longer on the local master)
          @rtype pair of connection type keyed dictionary of gateway_msgs.msg.Rule lists.
        '''
        # SLOW, EASY METHOD
        remote_gateways = remote_gateway_hub_index.keys()

        self._lock.acquire()

        self.flipped = self._prune_unavailable_gateway_flips(self.flipped, remote_gateways) # Prune locally cached flip list for flips that have lost their remotes, keep the rules though

        new_flips, removed_flips, flipped = self._prepare_flips(connections, remote_gateways, unique_name, master)  # Totally regenerate a new flipped interface, compare with old
        flip_status = self._prepare_flip_status(flipped)
        new_flips, filtered_flips = self._filter_flipped_in_interfaces(new_flips, self.registrations)
        self.flipped = self._update_flipped(flipped, filtered_flips)
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
        # utils.difflist = lambda l1,l2: [x for x in l1 if x not in l2] # diff of lists

    def _update_flipped(self, flipped, filtered_flips):
        updated_flipped = {}
        for connection_type in flipped.keys():
            updated_flipped[connection_type] = [copy.deepcopy(r) for r in flipped[connection_type] if not r in filtered_flips[connection_type]]
        return updated_flipped

    def _filter_flipped_in_interfaces(self, new_flips, flipped_in_registrations):
        '''
          Gateway should not flip out the flipped-in interface.
        '''
        filtered_flips = utils.create_empty_connection_type_dictionary()
        for connection_type in utils.connection_types:
            for rule in flipped_in_registrations[connection_type]:
                r = self._is_registration_in_remote_rule(rule, new_flips[connection_type])
                if r:
                    filtered_flips[connection_type].append(r)

        for connection_type in new_flips.keys():
            new_flips[connection_type] = [r for r in new_flips[connection_type] if not r in filtered_flips[connection_type]]

        rospy.logdebug("Gateway : filtered flip list to prevent cyclic flipping - %s"%str(filtered_flips))

        return new_flips, filtered_flips

    def _is_registration_in_remote_rule(self, rule, new_flip_remote_rules):
        for r in new_flip_remote_rules:
            node = r.rule.node.split(",")[0]
            if rule.local_node == node and rule.remote_gateway == r.gateway and rule.connection.rule.name == r.rule.name:
                return r
        return None

    def update_flip_status(self, flip, status):
        '''
          Update the status of a flip from the hub. This should be called right
          after update once self.flipped is established

          @return True if status was indeed changed, False otherwise
          @rtype Boolean
        '''
        state_changed = False
        self._lock.acquire()
        try:
            index = self.flipped[flip.rule.type].index(flip)
            state_changed = (self.flip_status[flip.rule.type][index] != status)
            self.flip_status[flip.rule.type][index] = status
        except ValueError:
            pass
        self._lock.release()
        return state_changed

    def remove_flip(self, flip):
        '''
          Removes a flip, so that it can be resent as necessary
        '''
        self._lock.acquire()
        try:
            self.flipped[flip.rule.type].remove(flip)
        except ValueError:
            pass
        self._lock.release()

    ##########################################################################
    # Utility Methods
    ##########################################################################

    def _generate_flips(self, connection_type, name, node, remote_gateways, unique_name, master):
        '''
          Checks if a local rule (obtained from master.get_system_state)
          is a suitable association with any of the rules or patterns. This can
          return multiple matches, since the same local rule
          properties can be multiply flipped to different remote gateways.

          Used in the update() call above that is run in the watcher thread.
          Note, don't need to lock here as the update() function takes care of it.

          @param connection_type : rule type
          @type str : string constant from gateway_msgs.msg.Rule

          @param name : fully qualified topic, service or action name
          @type str

          @param node : ros node name (coming from master.get_system_state)
          @type str

          @param gateways : gateways that are available (registered on the hub)
          @type string

          @param master : local master
          @type rocon_gateway.LocalMaster

          @return all the flip rules that match this local rule
          @return list of RemoteRule objects updated with node names from self.watchlist
        '''
        matched_flip_rules = []
        for flip_rule in self.watchlist[connection_type]:
            # Check if the flip rule corresponds to an existing gateway
            matched_gateways = self._get_matched_gateways(flip_rule, remote_gateways)
            if not matched_gateways:
                continue

            # Check names
            rule_name = flip_rule.rule.name
            matched = self.is_matched(flip_rule, rule_name, name, node)

            if not utils.is_all_pattern(flip_rule.rule.name):
                if not matched:
                    rule_name = '/' + unique_name + '/' + flip_rule.rule.name
                    matched = self.is_matched(flip_rule, rule_name, name, node)

                # If it is still not matching
                if not matched:
                    rule_name = '/' + flip_rule.rule.name
                    matched = self.is_matched(flip_rule, rule_name, name, node)

            if matched:
                for gateway in matched_gateways:
                    try:
                        matched_flip = copy.deepcopy(flip_rule)
                        matched_flip.gateway = gateway  # just in case we used a regex or matched basename
                        matched_flip.rule.name = name  # just in case we used a regex
                        matched_flip.rule.node = "%s,%s"%(node, master.lookupNode(node)) # just in case we used a regex
                        matched_flip_rules.append(matched_flip)
                    except rosgraph.masterapi.MasterError as e:
                        # Node has been gone already. skips sliently
                        pass
        return matched_flip_rules

    def _prune_unavailable_gateway_flips(self, flipped, remote_gateways):
        # Prune locally cached flip list for flips that have lost their remotes, keep the rules though
        for connection_type in utils.connection_types:
            # flip.gateway is a hash name, so is the remote_gateways list
            flipped[connection_type] = [flip for flip in self.flipped[connection_type] if flip.gateway in remote_gateways]
        return flipped

    def _prepare_flips(self, connections, remote_gateways, unique_name, master):
        # Variable preparations
        flipped         = utils.create_empty_connection_type_dictionary()
        new_flips       = utils.create_empty_connection_type_dictionary()
        removed_flips   = utils.create_empty_connection_type_dictionary()

        for connection_type in connections:
            for connection in connections[connection_type]:
                matched_flip_rules = self._generate_flips(connection.rule.type, connection.rule.name, connection.rule.node, remote_gateways, unique_name, master)
                flipped[connection_type].extend(matched_flip_rules)

            new_flips[connection_type] = utils.difflist(flipped[connection_type], self.flipped[connection_type])
            removed_flips[connection_type] = utils.difflist(self.flipped[connection_type], flipped[connection_type])
        return new_flips, removed_flips, flipped

    def _prepare_flip_status(self, flipped):
        flip_status     = utils.create_empty_connection_type_dictionary()

        # set flip status to unknown first, and then read previous status if available
        for connection_type in utils.connection_types:
            flip_status[connection_type] = [RemoteRuleWithStatus.UNKNOWN] * len(flipped[connection_type])

        for connection_type in utils.connection_types:
            for new_index, flip in enumerate(flipped[connection_type]):
                try:
                    index = self.flipped[connection_type].index(flip)
                    flip_status[connection_type][new_index] = self.flip_status[connection_type][index]
                except:
                    # The new flip probably did not exist. Let it remain unknown
                    pass
        self.flip_status = copy.deepcopy(flip_status)

        return flip_status

    def _get_matched_gateways(self, flip_rule, remote_gateways):
        matched_gateways = []
        for gateway in remote_gateways:
            # check for regular expression or perfect match
            gateway_match_result = re.match(flip_rule.gateway, gateway)
            if gateway_match_result and gateway_match_result.group() == gateway:
                matched_gateways.append(gateway)
            elif flip_rule.gateway == rocon_gateway_utils.gateway_basename(gateway):
                matched_gateways.append(gateway)
        return matched_gateways


    ##########################################################################
    # Accessors for Gateway Info
    ##########################################################################
    def get_flipped_connections(self):
        '''
          Gets the flipped connections list for GatewayInfo consumption.

          @return the list of flip rules that are activated and have been flipped.
          @rtype RemoteRule[]
        '''
        flipped_connections = []
        for connection_type in utils.connection_types:
            for i, connection in enumerate(self.flipped[connection_type]):
                flipped_connections.append(RemoteRuleWithStatus(connection, self.flip_status[connection_type][i]))
        return flipped_connections


if __name__ == "__main__":

    gateways = ['dude', 'dudette']
    dudes = ['fred', 'dude']
    dudes[:] = [x for x in dudes if x in gateways]
    print dudes
