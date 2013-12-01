#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_multimaster/license/LICENSE
#
##############################################################################
# Imports
##############################################################################

import os
import rocon_std_msgs.msg as rocon_std_msgs
from ros_utilities import find_resource_from_string

##############################################################################
# Functions
##############################################################################


def icon_to_msg(filename):
    '''
      Loads the specified filename and puts in
      rocon_std_msgs.Icon format

      @param : filename to the icon resource.
      @type : string
    '''
    icon = rocon_std_msgs.Icon()
    if filename == None or filename == "":
        return icon
    unused_basename, extension = os.path.splitext(filename)
    if extension.lower() == ".jpg" or extension.lower() == ".jpeg":
        icon.format = "jpeg"
    elif extension.lower() == ".png":
        icon.format = "png"
    else:
        icon.format = ""
        return icon
    icon.data = open(filename, "rb").read()
    return icon


def icon_resource_to_msg(resource):
    '''
      Loads the icon resource and puts in
      rocon_std_msgs.Icon format

      @param resource : resource identifier (package/name)
      @type : string

      @return the icon message
      @rtype rocon_std_msgs.msg.Icon
    '''
    filename = find_resource_from_string(resource)
    icon = icon_to_msg(filename)
    icon.resource_name = resource
    return icon
