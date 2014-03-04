#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_multimaster/license/LICENSE
#

##############################################################################
# Imports
##############################################################################

import os
import socket
import sys
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
    print("[ INFO] " + message + "\n")


def logerror(message):
    print(red_string("[ERROR] " + message))


def logfatal(message):
    print(red_string("[FATAL] " + message))


##############################################################################
# Ros
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

##############################################################################
# System
##############################################################################


def which(program):
    '''
    Emulate in a cross platform way the linux shell command

    @TODO: replace this with rocon_python_utils' tool
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


def check_if_executable_available(name):
    '''
      Ensure a particular executable is available on the system.

      Could use package names and python-apt here to find if the package is
      available, but more reliable and general - just check if program binary
      is available.

      Aborts program execution with fatal error if not found.
    '''
    if which(name) is None:
        sys.exit(logfatal("Hub : " + name + " not installed - hint 'rosdep install rocon_hub'."))

##############################################################################
# File Handling
##############################################################################


def read_template(template_filename):
    '''
      Convenience function for opening a file.
    '''
    f = open(template_filename, 'r')
    try:
        t = f.read()
    finally:
        f.close()
    return t
