#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_multimaster/hydro-devel/rocon_gateway/LICENSE
#

###############################################################################
# Imports
###############################################################################

import rospy
import gateway_msgs.srv as gateway_srvs

# local imports
import utils
import ros_parameters
from .watcher_thread import WatcherThread
from .flipped_interface import FlippedInterface
from .public_interface import PublicInterface
from .pulled_interface import PulledInterface
from .master_api import LocalMaster

###############################################################################
# Thread
###############################################################################


class Gateway(object):
    '''
      Used to synchronise with hubs.
    '''
    def __init__(self, hub_manager, param, publish_gateway_info_callback):
        '''
        @param param : a dictionary of many parameters set in ros_parameters.py

        @param hub_manager : container for all the hubs this gateway connects to
        @type hub_api.HubManmager

        @param publish_gateway_info_callback : callback for publishing gateway info
        '''
        self._param = param
        self._hub_manager = hub_manager
        self._publish_gateway_info = publish_gateway_info_callback
#        self._ip = None
#        self.is_connected = False
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
        self.public_interface = PublicInterface(default_rule_blacklist=default_rule_blacklist,
                                                default_rules=ros_parameters.generate_rules(self._param['default_advertisements'])
                                                )
        if self._param['advertise_all']:
            self.public_interface.advertiseAll([])  # no extra blacklist beyond the default (keeping it simple in yaml for now)
        self.master = LocalMaster()
        self.remote_gateway_request_callbacks = {}
        self.remote_gateway_request_callbacks['flip'] = self.process_remote_gateway_flip_request
        self.remote_gateway_request_callbacks['unflip'] = self.process_remote_gateway_unflip_request

        # create a thread to watch local rule states
        #self.watcher_thread = WatcherThread(self, self._param['watch_loop_period'])

    def shutdown(self):
        pass
        #self.watcher_thread.shutdown()
#        for connection_type in utils.connection_types:
#            for flip in self.flipped_interface.flipped[connection_type]:
#                self.hub.send_unflip_request(flip.gateway, flip.rule)
#            for registration in self.flipped_interface.registrations[connection_type]:
#                self.master.unregister(registration)

    ##########################################################################
    # Incoming commands from remote gateways
    ##########################################################################

    def process_remote_gateway_flip_request(self, registration):
        '''
          Used as a callback for incoming requests on redis pubsub channels.
          It gets assigned to RedisManager.callback.

          @param registration : fully detailed registration to be processed
          @type utils.Registration
        '''
        pass
#        if self.flipped_interface.firewall:
#            rospy.logwarn("Gateway : firewalling a flip request %s" % registration)
#        else:
#            rospy.loginfo("Gateway : received a flip request %s" % registration)
#            # probably not necessary as the flipping gateway will already check this
#            existing_registration = self.flipped_interface.findRegistrationMatch(registration.remote_gateway, registration.connection.rule.name, registration.connection.rule.node, registration.connection.rule.type)
#            if not existing_registration:
#                new_registration = self.master.register(registration)
#                if new_registration:
#                    self.flipped_interface.registrations[registration.connection.rule.type].append(new_registration)
#                    self._publish_gateway_info()

    def process_remote_gateway_unflip_request(self, rule, remote_gateway):
        rospy.loginfo("Gateway : received an unflip request from gateway %s: %s" % (remote_gateway, utils.formatRule(rule)))
#        existing_registration = self.flipped_interface.findRegistrationMatch(remote_gateway, rule.name, rule.node, rule.type)
#        if existing_registration:
#            self.master.unregister(existing_registration)
#            self.flipped_interface.registrations[existing_registration.connection.rule.type].remove(existing_registration)
#            self._publish_gateway_info()

    ##########################################################################
    # Incoming commands from local system (ros service callbacks)
    ##########################################################################

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
#        response.result, response.error_message = self._ros_service_advertise_checks()
#        if response.result == gateway_msgs.msg.Result.SUCCESS:
#            try:
#                if not request.cancel:
#                    for rule in request.rules:
#                        if not self.public_interface.add_rule(rule):
#                            response.result = gateway_msgs.msg.Result.ADVERTISEMENT_EXISTS
#                            response.error_message = "advertisment rule already exists [%s:(%s,%s)]" % (rule.name, rule.type, rule.node)
#                else:
#                    for rule in request.rules:
#                        if not self.public_interface.remove_rule(rule):
#                            response.result = gateway_msgs.msg.Result.ADVERTISEMENT_NOT_FOUND
#                            response.error_message = "advertisment not found [%s:(%s,%s)]" % (rule.name, rule.type, rule.node)
#            except Exception as e:
#                rospy.logerr("Gateway : unknown advertise error [%s]." % str(e))
#                response.result = gateway_msgs.msg.Result.UNKNOWN_ADVERTISEMENT_ERROR
#
#        # Let the watcher get on with the update asap
#        if response.result == gateway_msgs.msg.Result.SUCCESS:
#            self.watcher_thread.trigger_update = True
#            self._publish_gateway_info()
#        else:
#            rospy.logerr("Gateway : %s." % response.error_message)
#        response.watchlist = self.public_interface.getWatchlist()
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
#        response.result, response.error_message = self._ros_service_advertise_checks()
#        if response.result == gateway_msgs.msg.Result.SUCCESS:
#            try:
#                if not request.cancel:
#                    if not self.public_interface.advertiseAll(request.blacklist):
#                        response.result = gateway_msgs.msg.Result.ADVERTISEMENT_EXISTS
#                        response.error_message = "already advertising all."
#                else:
#                    self.public_interface.unadvertiseAll()
#            except Exception as e:
#                response.result = gateway_msgs.msg.Result.UNKNOWN_ADVERTISEMENT_ERROR
#                response.error_message = "unknown advertise all error [%s]" % (str(e))
#
#        # Let the watcher get on with the update asap
#        if response.result == gateway_msgs.msg.Result.SUCCESS:
#            self.watcher_thread.trigger_update = True
#            self._publish_gateway_info()
#        else:
#            rospy.logerr("Gateway : %s." % response.error_message)
#        response.blacklist = self.public_interface.getBlacklist()
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
        response = gateway_srvs.RemoteResponse()
#        for remote in request.remotes:
#            response.result, response.error_message = self._ros_service_flip_checks(remote.gateway)
#            if response.result != gateway_msgs.msg.Result.SUCCESS:
#                rospy.logerr("Gateway : %s." % response.error_message)
#                return response
#
#        # result is currently SUCCESS
#        added_rules = []
#        for remote in request.remotes:
#            if not request.cancel:
#                flip_rule = self.flipped_interface.add_rule(remote)
#                if flip_rule:
#                    added_rules.append(flip_rule)
#                    rospy.loginfo("Gateway : added flip rule [%s:(%s,%s)]" % (flip_rule.gateway, flip_rule.rule.name, flip_rule.rule.type))
#                else:
#                    response.result = gateway_msgs.msg.Result.FLIP_RULE_ALREADY_EXISTS
#                    response.error_message = "flip rule already exists [%s:(%s,%s)]" % (remote.gateway, remote.rule.name, remote.rule.type)
#                    break
#            else:  # request.cancel
#                removed_flip_rules = self.flipped_interface.remove_rule(remote)
#                if removed_flip_rules:
#                    rospy.loginfo("Gateway : removed flip rule [%s:(%s,%s)]" % (remote.gateway, remote.rule.name, remote.rule.type))
#
#        if response.result == gateway_msgs.msg.Result.SUCCESS:
#            self._publish_gateway_info()
#            self.watcher_thread.trigger_update = True
#        else:
#            if added_rules:  # completely abort any added rules
#                for added_rule in added_rules:
#                    self.flipped_interface.remove_rule(added_rule)
#            rospy.logerr("Gateway : %s." % response.error_message)
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
#        response.result, response.error_message = self._ros_service_flip_checks(request.gateway)
#        if response.result == gateway_msgs.msg.Result.SUCCESS:
#            if not request.cancel:
#                if self.flipped_interface.flip_all(request.gateway, request.blacklist):
#                    rospy.loginfo("Gateway : flipping all to gateway '%s'" % (request.gateway))
#                else:
#                    response.result = gateway_msgs.msg.Result.FLIP_RULE_ALREADY_EXISTS
#                    response.error_message = "already flipping all to gateway '%s' " + request.gateway
#            else:  # request.cancel
#                self.flipped_interface.un_flip_all(request.gateway)
#                rospy.loginfo("Gateway : cancelling a previous flip all request [%s]" % (request.gateway))
#        if response.result == gateway_msgs.msg.Result.SUCCESS:
#            self._publish_gateway_info()
#            self.watcher_thread.trigger_update = True
#        else:
#            rospy.logerr("Gateway : %s." % response.error_message)
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
        response = gateway_srvs.RemoteResponse()
#
#        for remote in request.remotes:
#            response.result, response.error_message = self._ros_service_remote_checks(remote.gateway)
#            if response.result != gateway_msgs.msg.Result.SUCCESS:
#                rospy.logerr("Gateway : %s." % response.error_message)
#                return response
#
#        # result is currently SUCCESS
#        added_rules = []
#        for remote in request.remotes:
#            if not request.cancel:
#                pull_rule = self.pulled_interface.add_rule(remote)
#                if pull_rule:
#                    added_rules.append(pull_rule)
#                    rospy.loginfo("Gateway : added pull rule [%s:(%s,%s)]" % (pull_rule.gateway, pull_rule.rule.name, pull_rule.rule.type))
#                else:
#                    response.result = gateway_msgs.msg.Result.PULL_RULE_ALREADY_EXISTS
#                    response.error_message = "pull rule already exists [%s:(%s,%s)]" % (remote.gateway, remote.rule.name, remote.rule.type)
#                    break
#            else:  # request.cancel
#                for remote in request.remotes:
#                    removed_pull_rules = self.pulled_interface.remove_rule(remote)
#                    if removed_pull_rules:
#                        rospy.loginfo("Gateway : removed pull rule [%s:%s]" % (remote.gateway, remote.rule.name))
#        if response.result == gateway_msgs.msg.Result.SUCCESS:
#            self._publish_gateway_info()
#            self.watcher_thread.trigger_update = True
#        else:
#            if added_rules:  # completely abort any added rules
#                for added_rule in added_rules:
#                    self.pulled_interface.remove_rule(added_rule)
#            rospy.logerr("Gateway : %s." % response.error_message)
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
#        response.result, response.error_message = self._ros_service_remote_checks(request.gateway)
#        if response.result == gateway_msgs.msg.Result.SUCCESS:
#            if not request.cancel:
#                if self.pulled_interface.pull_all(request.gateway, request.blacklist):
#                    rospy.loginfo("Gateway : pulling all from gateway '%s'" % (request.gateway))
#                else:
#                    response.result = gateway_msgs.msg.Result.FLIP_RULE_ALREADY_EXISTS
#                    response.error_message = "already pulling all from gateway '%s' " + request.gateway
#            else:  # request.cancel
#                self.pulled_interface.unpull_all(request.gateway)
#                rospy.loginfo("Gateway : cancelling a previous pull all request [%s]" % (request.gateway))
#        if response.result == gateway_msgs.msg.Result.SUCCESS:
#            self._publish_gateway_info()
#            self.watcher_thread.trigger_update = True
#        else:
#            rospy.logerr("Gateway : %s." % response.error_message)
        return response

    def _ros_service_advertise_checks(self):
#        if not self.is_connected:
#            return gateway_msgs.msg.Result.NO_HUB_CONNECTION, "not connected to hub, aborting"
#        else:
#            return gateway_msgs.msg.Result.SUCCESS, ""
        return None

    def _ros_service_flip_checks(self, gateway):
        '''
          Some simple checks for ros service flips.

          @param gateway : target gateway string of the flip
          @type string
          @return pair of result type and message
          @rtype gateway_msgs.msg.Result.xxx, string
        '''
#        result, error_message = self._ros_service_remote_checks(gateway)
#        if result == gateway_msgs.msg.Result.SUCCESS:
#            firewall_flag = False
#            try:
#                firewall_flag = self.hub.get_remote_gateway_firewall_flag(gateway)
#                if firewall_flag:
#                    return gateway_msgs.msg.Result.FLIP_REMOTE_GATEWAY_FIREWALLING, "remote gateway is firewalling flip requests, aborting [%s]" % gateway
#            except GatewayUnavailableError:
#                pass  # handled earlier in rosServiceRemoteChecks
#        return result, error_message
        return None

    def _ros_service_remote_checks(self, gateway):
        '''
          Some simple checks for ros service pulls

          @param gateway : target gateway string of the pull
          @type string
          @return pair of result type and message
          @rtype gateway_msgs.msg.Result.xxx, string
        '''
#        if not self.is_connected:
#            return gateway_msgs.msg.Result.NO_HUB_CONNECTION, "not connected to hub, aborting"
#        elif gateway == self.unique_name:
#            return gateway_msgs.msg.Result.FLIP_NO_TO_SELF, "gateway cannot flip to itself"
#        elif not self.hub.matches_remote_gateway_name(gateway):
#            return gateway_msgs.msg.Result.FLIP_REMOTE_GATEWAY_NOT_CONNECTED, "remote gateway is currently not connected [%s]" % gateway
#        else:
#            return gateway_msgs.msg.Result.SUCCESS, ""
        return None


###############################################################################
# Internal M ethods
###############################################################################
