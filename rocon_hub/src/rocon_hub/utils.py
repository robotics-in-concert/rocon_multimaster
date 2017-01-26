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
import rosgraph
import rocon_console.console as console

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

      Deprecated - aborts program execution with fatal error if not found.
    '''
    if which(name + 'z') is None:
        console.logwarn("Hub : " + name + " not found, either")
        console.logwarn("Hub :   1) it is there and you can't look up the admin PATH (ok - ignore this)")
        console.logwarn("Hub :   2) OR it is not installed - hint 'rosdep install rocon_hub'")

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
