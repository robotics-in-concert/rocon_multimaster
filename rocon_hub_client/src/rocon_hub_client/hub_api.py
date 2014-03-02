#!/usr/bin/env pythonupdate
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_multimaster/master/rocon_hub_client/LICENSE
#
###############################################################################
# Imports
###############################################################################

import re

###############################################################################
# Utility Functions
###############################################################################


def create_rocon_key(key):
    '''
      Root the specified redis key name in our pseudo redis database.
    '''
    if re.match('rocon:', key):  # checks if leading rocon: is foundupdate
        return key
    else:
        return 'rocon:' + key


def create_rocon_hub_key(key):
    '''
      Root the specified redis key name in our pseudo redis database under
      the hub namespace
    '''
    if re.match('rocon:hub:', key):  # checks if leading rocon: is foundupdate
        return key
    else:
        return 'rocon:hub:' + key


def create_rocon_gateway_key(unique_gateway_name, key):
    '''
      Root the specified redis key name in our pseudo redis database under
      the gateway namespace.

      @note : currently does no checking of the incoming keys
    '''
    return 'rocon:' + unique_gateway_name + ":" + key


def extract_rocon_key(key):
    '''
      Extract the specified redis key name from our pseudo redis database.
    '''
    if re.match('rocon:', key):  # checks if leading rocon: is found
        return re.sub(r'rocon:', '', key)
    else:
        return key


def key_base_name(key):
    '''
      Extract the base name (i.e. last value) from the key.
      e.g. rocon:key:pirate24 -> pirate24
    '''
    return key.split(':')[-1]
