#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_multimaster/hydro_devel/rocon_gateway/LICENSE
#
##############################################################################
# Imports
##############################################################################

import rospy
import gateway_msgs.msg as gateway_msgs
import gateway_msgs.srv as gateway_srvs

# local imports
from .exceptions import GatewaySampleRuntimeError
from utils import connection_types

##############################################################################
# Constants
##############################################################################

_gateway_namespace = '/gateway'
_tutorial_names = {gateway_msgs.ConnectionType.PUBLISHER: '/chatter',
         gateway_msgs.ConnectionType.SUBSCRIBER: '/chatter',
         gateway_msgs.ConnectionType.SERVICE: '/add_two_ints',
         gateway_msgs.ConnectionType.ACTION_CLIENT: '/fibonacci/client',
         gateway_msgs.ConnectionType.ACTION_SERVER: '/fibonacci/server'
        }
_tutorial_nodes = {gateway_msgs.ConnectionType.PUBLISHER: '',
         gateway_msgs.ConnectionType.SUBSCRIBER: '',
         gateway_msgs.ConnectionType.SERVICE: '',
         gateway_msgs.ConnectionType.ACTION_CLIENT: '',
         gateway_msgs.ConnectionType.ACTION_SERVER: ''
         }
_tutorial_regex_names = {
         gateway_msgs.ConnectionType.PUBLISHER: '.*ter',
         gateway_msgs.ConnectionType.SUBSCRIBER: '.*ter',
         gateway_msgs.ConnectionType.SERVICE: '/add_two_.*',
         gateway_msgs.ConnectionType.ACTION_CLIENT: '/fibonacci/cli.*',
         gateway_msgs.ConnectionType.ACTION_SERVER: '/fibonacci/ser.*'
        }
_tutorial_regex_nodes = {
         gateway_msgs.ConnectionType.PUBLISHER: '/t.*er',
         gateway_msgs.ConnectionType.SUBSCRIBER: '',
         gateway_msgs.ConnectionType.SERVICE: '',
         gateway_msgs.ConnectionType.ACTION_CLIENT: '',
         gateway_msgs.ConnectionType.ACTION_SERVER: ''
         }

##############################################################################
# Methods
##############################################################################
#
# Notes about these methods:
#  Ros is already running (i.e. rospy.init_node has been called
#  Use the gateway namespace above (could probably make it smarter by hunting)
#
##############################################################################


def _action_text(cancel=False, msg='acting'):
    text = "cancelling" if cancel else msg
    return text


def advertise_all(cancel=False, ns=_gateway_namespace):
    '''
      Sends a rule for advertising everything except the default blacklist.
    '''
    rospy.wait_for_service(ns + '/advertise_all')
    advertise_all = rospy.ServiceProxy(ns + '/advertise_all', gateway_srvs.AdvertiseAll)
    req = gateway_srvs.AdvertiseAllRequest()
    req.cancel = cancel
    req.blacklist = []
    rospy.loginfo("Advertise All : %s all." % _action_text(cancel, 'advertising'))
    resp = advertise_all(req)
    if resp.result != 0:
        raise GatewaySampleRuntimeError("failed to advertise all (todo: no error message yet)")


def advertise_tutorials(cancel=False, regex_patterns=False, ns=_gateway_namespace):
    rospy.wait_for_service(ns + '/advertise')
    advertise = rospy.ServiceProxy(ns + '/advertise', gateway_srvs.Advertise)
    req = gateway_srvs.AdvertiseRequest()
    req.cancel = cancel
    rule = gateway_msgs.Rule()
    if regex_patterns:
        names = _tutorial_regex_names
        nodes = _tutorial_regex_nodes
    else:
        names = _tutorial_names
        nodes = _tutorial_nodes
    for connection_type in connection_types:
        req.rules = []
        rule.name = names[connection_type]
        rule.type = connection_type
        rule.node = nodes[connection_type]
        rospy.loginfo("Advertise : %s [%s,%s,%s]." % (_action_text(cancel, 'advertising'), rule.type, rule.name, rule.node or 'None'))
        req.rules.append(rule)
        resp = advertise(req)
        if resp.result != 0:
            raise GatewaySampleRuntimeError("failed to advertise %s [%s]" % (rule.name, resp.error_message))


def pull_all(remote_gateway_name=None, cancel=False, ns=_gateway_namespace):
    '''
      Sends a rule for pulling everything from the specified remote gateway.
    '''
    rospy.wait_for_service(ns + '/pull_all')
    pull_all = rospy.ServiceProxy(ns + '/pull_all', gateway_srvs.RemoteAll)
    if not remote_gateway_name:
        remote_gateway_name = find_first_remote_gateway()
    req = gateway_srvs.RemoteAllRequest()
    req.gateway = remote_gateway_name
    req.cancel = cancel
    req.blacklist = []
    rospy.loginfo("Pull All : %s." % _action_text(cancel, 'sending pull rule for all to the gateway'))
    resp = pull_all(req)
    if resp.result != 0:
        raise GatewaySampleRuntimeError("failed to pull all from %s [%s]" % (remote_gateway_name, resp.error_message))


def pull_tutorials(remote_gateway_name=None, cancel=False, regex_patterns=False, ns=_gateway_namespace):
    rospy.wait_for_service(ns + '/pull')
    pull = rospy.ServiceProxy(ns + '/pull', gateway_srvs.Remote)
    if not remote_gateway_name:
        remote_gateway_name = find_first_remote_gateway()
    req = gateway_srvs.RemoteRequest()
    req.cancel = cancel
    if regex_patterns:
        names = _tutorial_regex_names
        nodes = _tutorial_regex_nodes
    else:
        names = _tutorial_names
        nodes = _tutorial_nodes
    req.remotes = []
    for connection_type in connection_types:
        rule = gateway_msgs.Rule()
        rule.name = names[connection_type]
        rule.type = connection_type
        rule.node = nodes[connection_type]
        rospy.loginfo("Pull : %s [%s,%s,%s][%s]." % (_action_text(cancel, 'sending pull rule to the gateway'), rule.type, rule.name, rule.node or 'None', remote_gateway_name))
        req.remotes.append(gateway_msgs.RemoteRule(remote_gateway_name, rule))
    resp = pull(req)
    if resp.result != 0:
        raise GatewaySampleRuntimeError("failed to advertise %s [%s]" % (rule.name, resp.error_message))


def flip_all(remote_gateway_name=None, cancel=False, ns=_gateway_namespace):
    '''
      Sends a rule for flipping everything to the specified remote gateway.
    '''
    rospy.wait_for_service(ns + '/flip_all')
    flip_all = rospy.ServiceProxy(ns + '/flip_all', gateway_srvs.RemoteAll)
    if not remote_gateway_name:
        remote_gateway_name = find_first_remote_gateway()
    req = gateway_srvs.RemoteAllRequest()
    req.gateway = remote_gateway_name
    req.cancel = cancel
    req.blacklist = []
    rospy.loginfo("Flip All : %s [%s]." % (_action_text(cancel, 'sending flip rule for all to the gateway'), remote_gateway_name))
    resp = flip_all(req)
    if resp.result != 0:
        raise GatewaySampleRuntimeError("failed to flip all to %s [%s]" % (remote_gateway_name, resp.error_message))



##############################################################################
# Utility functions
##############################################################################


def find_first_remote_gateway(ns=_gateway_namespace):
    '''
      Parses the remote gateway list to find a gateway to use for testing.

      It's a dumb hack to make testing quite convenient.

      @return gateway string name
      @rtype string
    '''
    remote_gateway_info = rospy.ServiceProxy(ns + '/remote_gateway_info', gateway_srvs.RemoteGatewayInfo)
    req = gateway_srvs.RemoteGatewayInfoRequest()
    req.gateways = []
    resp = remote_gateway_info(req)
    if len(resp.gateways) == 0:
        raise GatewaySampleRuntimeError("no remote gateways available")
    else:
        return resp.gateways[0].name
