#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_multimaster/license/LICENSE
#

###############################################################################
# Imports
###############################################################################

import copy
import rospy
import gateway_msgs.msg as gateway_msgs
import gateway_msgs.srv as gateway_srvs

from gateway_msgs.msg import RemoteRuleWithStatus as FlipStatus

from . import utils
from . import ros_parameters
from .watcher_thread import WatcherThread
from .flipped_interface import FlippedInterface
from .public_interface import PublicInterface
from .pulled_interface import PulledInterface
from .master_api import LocalMaster
from .network_interface_manager import NetworkInterfaceManager

###############################################################################
# Thread
###############################################################################


class Gateway(object):

    '''
      Used to synchronise with hubs.
    '''

    def __init__(self, hub_manager, param, unique_name, publish_gateway_info_callback):
        '''
        @param hub_manager : container for all the hubs this gateway connects to
        @type hub_api.HubManmager

        @param param : parameters set by ros_parameters.py
        @type : dictionary of parameter key-value pairs

        @param unique_name : gateway name (param['name']) with unique uuid hash appended

        @param publish_gateway_info_callback : callback for publishing gateway info
        '''
        self.hub_manager = hub_manager
        self.master = LocalMaster()
        self.ip = self.master.get_ros_ip()  # gateway is always assumed to sit on the same ip as the master
        self._param = param
        self._unique_name = unique_name
        self._publish_gateway_info = publish_gateway_info_callback
        default_rule_blacklist = ros_parameters.generate_rules(self._param["default_blacklist"])
        default_rules, all_targets = ros_parameters.generate_remote_rules(self._param["default_flips"])
        self.flipped_interface = FlippedInterface(
            firewall=self._param['firewall'],
            default_rule_blacklist=default_rule_blacklist,
            default_rules=default_rules,
            all_targets=all_targets)
        default_rules, all_targets = ros_parameters.generate_remote_rules(self._param["default_pulls"])
        self.pulled_interface = PulledInterface(default_rule_blacklist=default_rule_blacklist,
                                                default_rules=default_rules,
                                                all_targets=all_targets)
        self.public_interface = PublicInterface(
            default_rule_blacklist=default_rule_blacklist,
            default_rules=ros_parameters.generate_rules(self._param['default_advertisements']))
        if self._param['advertise_all']:
            # no extra blacklist beyond the default (keeping it simple in yaml for now)
            self.public_interface.advertise_all([])

        self.network_interface_manager = NetworkInterfaceManager(self._param['network_interface'])
        self.watcher_thread = WatcherThread(self, self._param['watch_loop_period'])

    def spin(self):
        self.watcher_thread.start()

    def shutdown(self):
        for connection_type in utils.connection_types:
            for flip in self.flipped_interface.flipped[connection_type]:
                self.hub_manager.send_unflip_request(flip.gateway, flip.rule)
            for registration in self.flipped_interface.registrations[connection_type]:
                self.master.unregister(registration)
            for registration in self.pulled_interface.registrations[connection_type]:
                self.master.unregister(registration)

    def is_connected(self):
        '''
          We often check if we're connected to any hubs often just to ensure we
          don't waste time processing if there is no-one listening.

          @return True if at least one hub is connected, False otherwise
          @rtype Bool
        '''
        return self.hub_manager.is_connected()

    def disengage_hub(self, hub):
        '''
          Disengage from the specified hub. Don't actually need to clean up connections
          here like we do in shutdown - that can be handled from the watcher thread itself.

          @param hub : the hub that will be deleted.
        '''
        self.hub_manager.disengage_hub(hub)
        self._publish_gateway_info()

    ##########################################################################
    # Update interface states (jobs assigned from watcher thread)
    ##########################################################################

    def update_flipped_interface(self, local_connection_index, remote_gateway_hub_index):
        '''
          Process the list of local connections and check against
          the current flip rules and patterns for changes. If a rule
          has become (un)available take appropriate action.

          @param local_connection_index : list of current local connections parsed from the master
          @type : dictionary of ConnectionType.xxx keyed lists of utils.Connections

          @param gateways : list of remote gateway string id's
          @type string
        '''
        state_changed = False

        # Get flip status of existing requests, and remove those requests that need to be resent
        flipped_connections = self.flipped_interface.get_flipped_connections()
        for flip in flipped_connections:
            if flip.remote_rule.gateway in remote_gateway_hub_index:
                for hub in remote_gateway_hub_index[flip.remote_rule.gateway]:
                    status = hub.get_flip_request_status(flip.remote_rule)
                    if status == FlipStatus.RESEND:
                        rospy.loginfo("Gateway : resend requested for flip request [%s]%s" %
                                      (flip.remote_rule.gateway, utils.format_rule(flip.remote_rule.rule)))
                        # Remove the flip, so that it will be resent as part of new_flips
                        self.flipped_interface.remove_flip(flip.remote_rule)
                        hub.send_unflip_request(flip.remote_rule.gateway, flip.remote_rule.rule)
                        hub.remove_flip_details(flip.remote_rule.gateway,
                                                flip.remote_rule.rule.name,
                                                flip.remote_rule.rule.type,
                                                flip.remote_rule.rule.node)
                        break

        new_flips, lost_flips = self.flipped_interface.update(
            local_connection_index, remote_gateway_hub_index, self._unique_name, self.master)
        for connection_type in utils.connection_types:
            for flip in new_flips[connection_type]:
                firewall_flag = self.hub_manager.get_remote_gateway_firewall_flag(flip.gateway)
                if firewall_flag:
                    continue
                state_changed = True
                # for actions, need to post flip details here
                connections = self.master.generate_connection_details(flip.rule.type, flip.rule.name, flip.rule.node)
                if (connection_type == utils.ConnectionType.ACTION_CLIENT or
                        connection_type == utils.ConnectionType.ACTION_SERVER):
                    rospy.loginfo("Gateway : sending flip request [%s]%s" %
                                  (flip.gateway, utils.format_rule(flip.rule)))
                    hub = remote_gateway_hub_index[flip.gateway][0]
                    hub.post_flip_details(flip.gateway, flip.rule.name, flip.rule.type, flip.rule.node)
                    for connection in connections:
                        hub.send_flip_request(flip.gateway, connection)  # flip the individual pubs/subs
                else:
                    for connection in connections:
                        rospy.loginfo("Gateway : sending flip request [%s]%s" %
                                      (flip.gateway, utils.format_rule(flip.rule)))
                        hub = remote_gateway_hub_index[flip.gateway][0]
                        hub.send_flip_request(flip.gateway, connection)
                        hub.post_flip_details(
                            flip.gateway, flip.rule.name, flip.rule.type, flip.rule.node)
            for flip in lost_flips[connection_type]:
                state_changed = True
                rospy.loginfo("Gateway : sending unflip request [%s]%s" % (flip.gateway, utils.format_rule(flip.rule)))
                for hub in remote_gateway_hub_index[flip.gateway]:
                    rule = copy.deepcopy(flip.rule)
                    rule.node = rule.node.split(",")[0] # strip out xmlrpc uri to send unflip request
                    if hub.send_unflip_request(flip.gateway, rule):
                        # This hub was used to send the original flip request
                        hub.remove_flip_details(flip.gateway, flip.rule.name, flip.rule.type, flip.rule.node)
                        break

        # Update flip status
        flipped_connections = self.flipped_interface.get_flipped_connections()
        for flip in flipped_connections:
            for hub in remote_gateway_hub_index[flip.remote_rule.gateway]:
                rule = copy.deepcopy(flip.remote_rule)
                rule.rule.node = rule.rule.node.split(",")[0]
                status = hub.get_flip_request_status(flip.remote_rule)
                if status is not None:
                    flip_state_changed = self.flipped_interface.update_flip_status(flip.remote_rule, status)
                    state_changed = state_changed or flip_state_changed
                    break

        if state_changed:
            self._publish_gateway_info()

    def update_pulled_interface(self, unused_connections, remote_gateway_hub_index):
        '''
          Process the list of local connections and check against
          the current pull rules and patterns for changes. If a rule
          has become (un)available take appropriate action.

          This is called by the watcher thread. The remote_gateway_hub_index
          is always a full dictionary of all remote gateway and hub key-value
          pairs - it is only included as an argument here to save
          processing doubly in the watcher thread.

          @param connections : list of current local connections parsed from the master
          @type : dictionary of ConnectionType.xxx keyed lists of utils.Connections

          @param remote_gateway_hub_index : key-value remote gateway name-hub list pairs
          @type dictionary of remote_gateway_name-list of hub_api.Hub objects key-value pairs
        '''
        state_changed = False
        remote_connections = {}
        for remote_gateway in remote_gateway_hub_index.keys() + self.pulled_interface.list_remote_gateway_names():
            remote_connections[remote_gateway] = {}
            try:
                for hub in remote_gateway_hub_index[remote_gateway]:
                    remote_connections[remote_gateway].update(hub.get_remote_connection_state(remote_gateway))
            except KeyError:
                pass  # remote gateway no longer exists on the hub network
        new_pulls, lost_pulls = self.pulled_interface.update(remote_connections, self._unique_name)
        for connection_type in utils.connection_types:
            for pull in new_pulls[connection_type]:
                # dig out the corresponding connection (bit inefficient plouging back into this again
                connection = None
                for remote_gateway in remote_connections.keys():
                    for c in remote_connections[remote_gateway][pull.rule.type]:
                        if c.rule.name == pull.rule.name and \
                           c.rule.node == pull.rule.node:
                            connection = c
                            break
                    if connection:
                        break
                # Register this pull
                existing_registration = self.pulled_interface.find_registration_match(
                    pull.gateway, pull.rule.name, pull.rule.node, pull.rule.type)
                if not existing_registration:
                    rospy.loginfo("Gateway : pulling in connection %s[%s]" %
                                  (utils.format_rule(pull.rule), remote_gateway))
                    registration = utils.Registration(connection, pull.gateway)
                    new_registration = self.master.register(registration)
                    if new_registration is not None:
                        self.pulled_interface.registrations[registration.connection.rule.type].append(new_registration)
                        hub = remote_gateway_hub_index[pull.gateway][0]
                        hub.post_pull_details(pull.gateway, pull.rule.name, pull.rule.type, pull.rule.node)
                        state_changed = True
            for pull in lost_pulls[connection_type]:
                # Unregister this pull
                existing_registration = self.pulled_interface.find_registration_match(
                    pull.gateway, pull.rule.name, pull.rule.node, pull.rule.type)
                if existing_registration:
                    rospy.loginfo("Gateway : abandoning pulled connection %s[%s]" % (
                        utils.format_rule(pull.rule), pull.gateway))
                    self.master.unregister(existing_registration)
                    # This code was here, but causing bugs...actually it should never remove details from the hub,
                    # that is the responsibility of the advertising gateway. TODO confirm this.
                    #hub = remote_gateway_hub_index[pull.gateway][0]
                    # if hub:
                    #    hub.remove_pull_details(pull.gateway, pull.rule.name, pull.rule.type, pull.rule.node)
                    self.pulled_interface.registrations[
                        existing_registration.connection.rule.type].remove(existing_registration)
                    state_changed = True
        if state_changed:
            self._publish_gateway_info()

    def update_public_interface(self, local_connection_index):
        '''
          Process the list of local connections and check against
          the current rules and patterns for changes. If a rule
          has become (un)available take appropriate action.

          @param local_connection_index : list of current local connections parsed from the master
          @type : { utils.ConnectionType.xxx : utils.Connection[] } dictionaries
        '''
        state_changed = False
        # new_conns, lost_conns are of type { utils.ConnectionType.xxx : utils.Connection[] }
        new_conns, lost_conns = self.public_interface.update(
            local_connection_index, self.master.generate_advertisement_connection_details)
        # public_interface is of type gateway_msgs.Rule[]
        public_interface = self.public_interface.getInterface()
        for connection_type in utils.connection_types:
            for new_connection in new_conns[connection_type]:
                rospy.loginfo("Gateway : adding connection to public interface %s" %
                              utils.format_rule(new_connection.rule))
                self.hub_manager.advertise(new_connection)
                state_changed = True
            for lost_connection in lost_conns[connection_type]:
                rospy.loginfo("Gateway : removing connection from public interface %s" %
                              utils.format_rule(lost_connection.rule))
                self.hub_manager.unadvertise(lost_connection)
                state_changed = True
        if state_changed:
            self._publish_gateway_info()
        return public_interface

    def update_flipped_in_interface(self, registrations, remote_gateway_hub_index):
        '''
          Match the flipped in connections to supplied registrations using
          supplied registrations, flipping and unflipping as necessary.

          @param registrations : registrations (with status) to be processed
          @type list of (utils.Registration, str) where the str contains the status
        '''

        hubs = {}
        for gateway in remote_gateway_hub_index:
            for hub in remote_gateway_hub_index[gateway]:
                hubs[hub.uri] = hub

        update_flip_status = {}
        if self.flipped_interface.firewall:
            if len(registrations) != 0:
                rospy.logwarn("Gateway : firewalled, but received flip requests...")
                for (registration, status) in registrations:
                    for hub in remote_gateway_hub_index[registration.remote_gateway]:
                        if hub.uri not in update_flip_status:
                            update_flip_status[hub.uri] = []
                        update_flip_status[hub.uri].append((registration, FlipStatus.BLOCKED))

            # Mark all these registrations as blocked
            for hub_uri, hub in hubs.iteritems():
                if hub_uri in update_flip_status:
                    hub.update_multiple_flip_request_status(update_flip_status[hub_uri])
            return

        state_changed = False

        # Add new registrations
        for (registration, status) in registrations:
            # probably not necessary as the flipping gateway will already check this
            existing_registration = self.flipped_interface.find_registration_match(
                registration.remote_gateway,
                registration.connection.rule.name,
                registration.connection.rule.node,
                registration.connection.rule.type)
            if not existing_registration:
                rospy.loginfo("Gateway : received a flip request %s" % str(registration))
                state_changed = True
                new_registration = self.master.register(registration)
                if new_registration is not None:
                    self.flipped_interface.registrations[registration.connection.rule.type].append(new_registration)
                # Update this flip's status
                if status != FlipStatus.ACCEPTED:
                    for hub in remote_gateway_hub_index[registration.remote_gateway]:
                        if hub.uri not in update_flip_status:
                            update_flip_status[hub.uri] = []
                        update_flip_status[hub.uri].append((registration, FlipStatus.ACCEPTED))
            else:
                # Just make sure that this flip request is marked as accepted
                if status != FlipStatus.ACCEPTED:
                    for hub in remote_gateway_hub_index[registration.remote_gateway]:
                        if hub.uri not in update_flip_status:
                            update_flip_status[hub.uri] = []
                        update_flip_status[hub.uri].append((registration, FlipStatus.ACCEPTED))

        # Update the flip status for newly added registrations
        for hub_uri, hub in hubs.iteritems():
            if hub_uri in update_flip_status:
                hub.update_multiple_flip_request_status(update_flip_status[hub_uri])

        # Remove local registrations that are no longer flipped to this gateway
        local_registrations = copy.deepcopy(self.flipped_interface.registrations)
        for connection_type in utils.connection_types:
            for local_registration in local_registrations[connection_type]:
                matched_registration = None
                for (registration, status) in registrations:
                    if registration.connection == local_registration.connection and \
                       registration.remote_gateway == local_registration.remote_gateway:
                        matched_registration = registration
                        break
                    else:
                        continue
                if matched_registration is None:
                    state_changed = True
                    rospy.loginfo("Gateway : unflipping received flip %s" % str(local_registration))
                    self.master.unregister(local_registration)
                    self.flipped_interface.registrations[connection_type].remove(local_registration)

        if state_changed:
            self._publish_gateway_info()

    def update_network_information(self):
        '''
          If we are running over a wired connection, then do nothing.
          If over the wireless, updated data transfer rate and signal strength
          for this gateway on the hub
        '''
        statistics = self.network_interface_manager.get_statistics()
        self.hub_manager.publish_network_statistics(statistics)

    ##########################################################################
    # Incoming commands from local system (ros service callbacks)
    ##########################################################################

    def ros_service_set_watcher_period(self, request):
        '''
          Configures the watcher period. This is useful to slow/speed up the
          watcher loop. Quite often you want it polling quickly early while
          configuring connections, but on long loops later when it does not have
          to do very much except look for shutdown.

          @param request
          @type gateway_srvs.SetWatcherPeriodRequest
          @return service response
          @rtgateway_srvs.srv.SetWatcherPeriodResponse
        '''
        self.watcher_thread.set_watch_loop_period(request.period)
        return gateway_srvs.SetWatcherPeriodResponse(self.watcher_thread.get_watch_loop_period())

    def ros_subscriber_force_update(self, data):
        '''
          Trigger a watcher loop update
        '''
        self.watcher_thread.trigger_update = True

    def ros_service_advertise(self, request):
        '''
          Puts/Removes a number of rules on the public interface watchlist.
          As local rules matching these rules become available/go away,
          the public interface is modified accordingly. A manual update is done
          at the end of the advertise call to quickly capture existing
          rules

          @param request
          @type gateway_srvs.AdvertiseRequest
          @return service response
          @rtgateway_srvs.srv.AdvertiseReponse
        '''
        response = gateway_srvs.AdvertiseResponse()
        try:
            if not request.cancel:
                for rule in request.rules:
                    if not self.public_interface.add_rule(rule):
                        response.result = gateway_msgs.ErrorCodes.ADVERTISEMENT_EXISTS
                        response.error_message = "advertisment rule already exists [%s:(%s,%s)]" % (
                            rule.name, rule.type, rule.node)
            else:
                for rule in request.rules:
                    if not self.public_interface.remove_rule(rule):
                        response.result = gateway_msgs.ErrorCodes.ADVERTISEMENT_NOT_FOUND
                        response.error_message = "advertisment not found [%s:(%s,%s)]" % (
                            rule.name, rule.type, rule.node)
        except Exception as e:
            rospy.logerr("Gateway : unknown advertise error [%s]." % str(e))
            response.result = gateway_msgs.ErrorCodes.UNKNOWN_ADVERTISEMENT_ERROR

        # Let the watcher get on with the update asap
        if response.result == gateway_msgs.ErrorCodes.SUCCESS:
            self.watcher_thread.trigger_update = True
            self._publish_gateway_info()
        else:
            rospy.logerr("Gateway : %s." % response.error_message)
        response.watchlist = self.public_interface.getWatchlist()
        return response

    def ros_service_advertise_all(self, request):
        '''
          Toggles the advertise all mode. If advertising all, an additional
          blacklist parameter can be supplied which includes all the topics that
          will not be advertised/watched for. This blacklist is added to the
          default blacklist of the public interface

          @param request
          @type gateway_srvs.AdvertiseAllRequest
          @return service response
          @rtype gateway_srvs.AdvertiseAllReponse
        '''
        response = gateway_srvs.AdvertiseAllResponse()
        try:
            if not request.cancel:
                if not self.public_interface.advertise_all(request.blacklist):
                    response.result = gateway_msgs.ErrorCodes.ADVERTISEMENT_EXISTS
                    response.error_message = "already advertising all."
            else:
                self.public_interface.unadvertise_all()
        except Exception as e:
            response.result = gateway_msgs.ErrorCodes.UNKNOWN_ADVERTISEMENT_ERROR
            response.error_message = "unknown advertise all error [%s]" % (str(e))

        # Let the watcher get on with the update asap
        if response.result == gateway_msgs.ErrorCodes.SUCCESS:
            self.watcher_thread.trigger_update = True
            self._publish_gateway_info()
        else:
            rospy.logerr("Gateway : %s." % response.error_message)
        response.blacklist = self.public_interface.getBlacklist()
        return response

    def ros_service_flip(self, request):
        '''
          Puts flip rules on a watchlist which (un)flips them when they
          become (un)available.

          @param request
          @type gateway_srvs.RemoteRequest
          @return service response
          @rtype gateway_srvs.RemoteResponse
        '''
        # could move this below and if any are fails, just abort adding the rules.
        # Check if the target remote gateway is valid.
        response = self._check_remote_gateways(request.remotes)
        if response:
            return response

        response = gateway_srvs.RemoteResponse(gateway_msgs.ErrorCodes.SUCCESS, "")

        # result is currently SUCCESS
        # Process all add/remove flip requests
        if not request.cancel: # Rule add request
            response = self._add_flip_rules(request.remotes)
        else: # Rule remove request
            response = self._remove_flip_rules(request.remotes)

        # Post processing
        if response.result == gateway_msgs.ErrorCodes.SUCCESS:
            self._publish_gateway_info()
            self.watcher_thread.trigger_update = True
        else:
            rospy.logerr("Gateway : %s." % response.error_message)
        return response

    def ros_service_flip_all(self, request):
        '''
          Flips everything except a specified blacklist to a particular gateway,
          or if the cancel flag is set, clears all flips to that gateway.

          @param request
          @type gateway_srvs.RemoteAllRequest
          @return service response
          @rtype gateway_srvs.RemoteAllResponse
        '''
        response = gateway_srvs.RemoteAllResponse()
        remote_gateway_target_hash_name, response.result, response.error_message = self._ros_service_remote_checks(
            request.gateway)
        if response.result == gateway_msgs.ErrorCodes.SUCCESS:
            if not request.cancel:
                if self.flipped_interface.flip_all(remote_gateway_target_hash_name, request.blacklist):
                    rospy.loginfo("Gateway : flipping all to gateway '%s'" % (remote_gateway_target_hash_name))
                else:
                    response.result = gateway_msgs.ErrorCodes.FLIP_RULE_ALREADY_EXISTS
                    response.error_message = "already flipping all to gateway '%s' " + remote_gateway_target_hash_name
            else:  # request.cancel
                self.flipped_interface.unflip_all(remote_gateway_target_hash_name)
                rospy.loginfo("Gateway : cancelling a previous flip all request [%s]" % (request.gateway))
        if response.result == gateway_msgs.ErrorCodes.SUCCESS:
            self._publish_gateway_info()
            self.watcher_thread.trigger_update = True
        else:
            rospy.logerr("Gateway : %s." % response.error_message)
        return response

    def ros_service_pull(self, request):
        '''
          Puts a single rule on a watchlist and pulls it from a particular
          gateway when it becomes (un)available.

          @param request
          @type gateway_srvs.RemoteRequest
          @return service response
          @rtype gateway_srvs.RemoteResponse
        '''
        # could move this below and if any are fails, just abort adding the rules.
        # Check if the target remote gateway is valid.
        response = self._check_remote_gateways(request.remotes)
        if response:
            return response

        response = gateway_srvs.RemoteResponse(gateway_msgs.ErrorCodes.SUCCESS, "")

        # result is currently SUCCESS
        added_rules = []
        for remote in request.remotes:
            if not request.cancel:
                pull_rule = self.pulled_interface.add_rule(remote)
                if pull_rule:
                    added_rules.append(pull_rule)
                    rospy.loginfo("Gateway : added pull rule [%s:(%s,%s)]" %
                                  (pull_rule.gateway, pull_rule.rule.name, pull_rule.rule.type))
                else:
                    response.result = gateway_msgs.ErrorCodes.PULL_RULE_ALREADY_EXISTS
                    response.error_message = "pull rule already exists [%s:(%s,%s)]" % (
                        remote.gateway, remote.rule.name, remote.rule.type)
                    break
            else:  # request.cancel
                for remote in request.remotes:
                    removed_pull_rules = self.pulled_interface.remove_rule(remote)
                    if removed_pull_rules:
                        rospy.loginfo("Gateway : removed pull rule [%s:%s]" % (remote.gateway, remote.rule.name))
        if response.result == gateway_msgs.ErrorCodes.SUCCESS:
            self._publish_gateway_info()
            self.watcher_thread.trigger_update = True
        else:
            if added_rules:  # completely abort any added rules
                for added_rule in added_rules:
                    self.pulled_interface.remove_rule(added_rule)
            rospy.logerr("Gateway : %s." % response.error_message)
        return response

    def ros_service_pull_all(self, request):
        '''
          Pull everything except a specified blacklist from a particular gateway,
          or if the cancel flag is set, clears all pulls from that gateway.

          @param request
          @type gateway_srvs.RemoteAllRequest
          @return service response
          @rtype gateway_srvs.RemoteAllResponse
        '''
        response = gateway_srvs.RemoteAllResponse()
        remote_gateway_target_hash_name, response.result, response.error_message = self._ros_service_remote_checks(
            request.gateway)
        if response.result == gateway_msgs.ErrorCodes.SUCCESS:
            if not request.cancel:
                if self.pulled_interface.pull_all(remote_gateway_target_hash_name, request.blacklist):
                    rospy.loginfo("Gateway : pulling all from gateway '%s'" % (request.gateway))
                else:
                    response.result = gateway_msgs.ErrorCodes.FLIP_RULE_ALREADY_EXISTS
                    response.error_message = "already pulling all from gateway '%s' " + request.gateway
            else:  # request.cancel
                self.pulled_interface.unpull_all(remote_gateway_target_hash_name)
                rospy.loginfo("Gateway : cancelling a previous pull all request [%s]" % (request.gateway))
        if response.result == gateway_msgs.ErrorCodes.SUCCESS:
            self._publish_gateway_info()
            self.watcher_thread.trigger_update = True
        else:
            rospy.logerr("Gateway : %s." % response.error_message)
        return response

    def _ros_service_remote_checks(self, gateway):
        '''
          Some simple checks when pulling or flipping to make sure that the remote gateway is visible. It
          does a strict check on the hash names first, then falls back to looking for weak matches on the
          human friendly name.

          @param gateway : remote gateway target name (can be hash name, basename or regex pattern)
          @type string
          @return pair of result type and message
          @rtype gateway_msgs.ErrorCodes.xxx, string
        '''
        if not self.is_connected():
            return None, gateway_msgs.ErrorCodes.NO_HUB_CONNECTION, "not connected to hub, aborting"
        if gateway == self._unique_name:
            return None, gateway_msgs.ErrorCodes.REMOTE_GATEWAY_SELF_IS_NOT, "gateway cannot flip/pull to itself"
        return gateway, gateway_msgs.ErrorCodes.SUCCESS, ""
#        matches, weak_matches = self.hub_manager.match_remote_gateway_name(gateway)
#        if len(matches) > 1:
#            return None, gateway_msgs.ErrorCodes.REMOTE_GATEWAY_TARGET_HAS_MULTIPLE_MATCHES, \
#                "remote gateway target has multiple matches, invalid [%s][%s]" % (gateway, matches)
#        elif len(matches) == 1:
#            return matches[0], gateway_msgs.ErrorCodes.SUCCESS, ""
# Fallback to checking for weak matches
#        if len(weak_matches) > 1:
#            return None, gateway_msgs.ErrorCodes.REMOTE_GATEWAY_TARGET_HAS_MULTIPLE_MATCHES, \
#                "remote gateway target has multiple matches against hashed names, invalid [%s]" % weak_matches
#        elif len(weak_matches) == 1:
#            return weak_matches[0], gateway_msgs.ErrorCodes.SUCCESS, ""
# Not visible
# return None, gateway_msgs.ErrorCodes.REMOTE_GATEWAY_NOT_VISIBLE, "remote
# gateway is currently not visible on the hubs [%s]" % gateway

    def _check_remote_gateways(self, remotes):
        """
          Check given gateways in remote rules are valid

          :param remotes: remote rules
          :type remotes: gateway_msgs.RemoteRule[]

          :return: whether it is valid, error message if it failes
          :rtypes: None or gateway_srvs.RemoteResponse
        """
        response = gateway_srvs.RemoteResponse()
        for remote in remotes:
            remote.gateway, response.result, response.error_message = self._ros_service_remote_checks(remote.gateway)
            if response.result != gateway_msgs.ErrorCodes.SUCCESS:
                rospy.logerr("Gateway : %s." % response.error_message)
                return response
        return None

    def _add_flip_rules(self, remotes):
        """
          Add given rules into watcher list

          :param remotes: remote rules
          :type remotes: gateway_msgs.RemoteRule[]
          :return: whether it is successful
          :rtypes: gateway_srvs.RemoteResponse
        """
        response = gateway_srvs.RemoteResponse()
        response.result = gateway_msgs.ErrorCodes.SUCCESS

        added_rules = []
        for remote in remotes:
            flip_rule = self.flipped_interface.add_rule(remote)
            if flip_rule:
                added_rules.append(flip_rule)
                rospy.loginfo("Gateway : added flip rule [%s:(%s,%s)]" % (flip_rule.gateway, flip_rule.rule.name, flip_rule.rule.type))
            else:
                response.result = gateway_msgs.ErrorCodes.FLIP_RULE_ALREADY_EXISTS
                response.error_message = "flip rule already exists [%s:(%s,%s)]" % (remote.gateway, remote.rule.name, remote.rule.type)

        if response.result != gateway_msgs.ErrorCodes.SUCCESS:
            # completely abort any added rules
            for added_rule in added_rules:
                self.flipped_interface.remove_rule(added_rule)
        return response

    def _remove_flip_rules(self, remotes):
        """
          remove given rules into watcher list

          :param remotes: remote rules
          :type remotes: gateway_msgs.RemoteRule[]
          :return: whether it is successful
          :rtypes: gateway_srvs.RemoteResponse
        """
        response = gateway_srvs.RemoteResponse()
        response.result = gateway_msgs.ErrorCodes.SUCCESS

        for remote in remotes:
            removed_flip_rules = self.flipped_interface.remove_rule(remote)
            if removed_flip_rules:
                rospy.loginfo("Gateway : removed flip rule [%s:(%s,%s)]" % (remote.gateway, remote.rule.name, remote.rule.type))
        return response
