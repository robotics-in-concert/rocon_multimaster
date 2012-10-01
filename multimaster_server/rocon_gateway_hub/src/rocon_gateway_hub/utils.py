#!/usr/bin/env python
#       
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_multimaster/master/multimaster_server/rocon_gateway_hub/LICENSE 
# Copyright (c) 2012, Yujin Robot, Daniel Stonier
#

import os
import socket
import roslib; roslib.load_manifest('rocon_gateway_hub')
import rosgraph

##############################################################################
# Logging
##############################################################################

class Console:
    bold = "\033[1m"
    reset = "\033[0;0m"
    red = "\033[31m"

def red_string(msg):
    """bound string with console symbols for red output"""
    return Console.red + msg + Console.reset

def bold_string(msg):
    """bound string with console symbols for bold output"""
    return Console.bold + msg + Console.reset

def loginfo(message):
    print("[ INFO] "+message+"\n")

def logerror(message):
    print(red_string("[ERROR] "+message))

def logfatal(message):
    print(red_string("[FATAL] "+message))
    
##############################################################################
# Other
##############################################################################

def check_master():
    """
    Make sure that master is available
    :raises: :exc:`ROSTopicException` If unable to successfully communicate with master
    """
    try:
        rosgraph.Master('dude').getPid()
        return True
    except socket.error:
        return False

def which(program):
    '''
    Emulate in a cross platform way the linux shell command
    '''
    def is_exe(fpath):
        return os.path.exists(fpath) and os.access(fpath, os.X_OK)
 
    fpath, unused_fname = os.path.split(program)
    if fpath:
        if is_exe(program):
            return program
    else:
        for path in os.environ["PATH"].split(os.pathsep):
            exe_file = os.path.join(path, program)
            if is_exe(exe_file):
                return exe_file
 
    return None