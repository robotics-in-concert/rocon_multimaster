#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_multimaster/license/LICENSE
#

##############################################################################
# Imports
##############################################################################

import re

##############################################################################
# Methods
##############################################################################

valid_uuid_pattern = re.compile("^[0-9a-f]{32}$")


def is_uuid_postfixed(name):
    '''
      A not very reliable way to check if the last 32 characters of a string
      are a uuid postfix. Not reliable because if the prefix characters are
      [0-9a-f] then it will accept shorter combinations of strings. Practical
      because who will give a gateway a uuid style name?
    '''
    if len(name) <= 32:
        return False
    uuid_potential_part = name[-32:]
    match = valid_uuid_pattern.match(uuid_potential_part)
    return False if match is None else True


def gateway_basename(gateway_name):
    '''
      Strips the 16 byte hash (in hex format) from a gateway name, leaving the base name.
      Note, 16 hex values represents 32 characters

      @param gateway_name : base_name + 16 byte hex formatted hash
      @type str
      @return base name without the hash
      @rtype str
    '''
    # Need to check if it actually has a uuid postfix since gateways can disable uuid's.
    if is_uuid_postfixed(gateway_name):
        return gateway_name[:-32]
    else:
        return gateway_name

if __name__ == '__main__':
    print gateway_basename("dude")
    print gateway_basename("dude8bd699042519416d88722e8b0611d43")
    print gateway_basename("dude8bd699042519416d88722e8b0611d43b")
