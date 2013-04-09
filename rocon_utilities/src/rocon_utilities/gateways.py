#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_multimaster/master/rocon_utilities/LICENSE
#

##############################################################################
# Imports
##############################################################################

import gateway_msgs.msg as gateway_msgs

##############################################################################
# Methods
##############################################################################


def create_gateway_remote_rule(gateway, rule):
    r = gateway_msgs.RemoteRule()
    r.gateway = gateway
    r.rule = rule
    return r


def create_gateway_rule(name, connection_type, node_name=''):
    '''
      Quickly hack a gateway rule.

      @param name : connection name (e.g. /chatter)
      @type string
      @param connection_type : one of pub, sub, etc.
      @type gateway_msgs.ConnectionType,XXX constants
    '''
    r = gateway_msgs.Rule()
    r.name = name
    r.type = connection_type
    r.node = node_name
    return r


def basename(gateway_name):
    '''
      Strips the 16 byte hash (in hex format) from a gateway name, leaving the base name.
      Note, 16 hex values represents 32 characters

      @param gateway_name : base_name + 16 byte hex formatted hash
      @type str
      @return base name without the hash
      @rtype str
    '''
    return gateway_name[:-32]
