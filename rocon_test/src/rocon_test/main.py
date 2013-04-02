#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_multimaster/hydro-devel/rocon_test/LICENSE
#

##############################################################################
# Imports
##############################################################################

from __future__ import print_function

import roslaunch.rlutil
import roslaunch.core
import logging
import rospkg
import argparse
from argparse import RawTextHelpFormatter

##############################################################################
# Methods
##############################################################################

def help_string():
    overview = 'Launches a rocon multi-launch test.\n\n'
    instructions = " \
 - 'rocon-test xxx' : create an empty workspace in ./ecl.\n \
 With some extra info (todo).\n\n \
 "
    return overview + instructions

def _parse_arguments():
    parser = argparse.ArgumentParser(description=help_string(), formatter_class=RawTextHelpFormatter)
    parser.add_argument('package', nargs='?', default=None, help='name of the package in which to find the test configuration')
    parser.add_argument('test', nargs=1, help='name of the test configuration (xml) file')

#    parser.add_argument('-j', '--jobs', type=int, metavar='JOBS', default=None, nargs='?', help='Specifies the number of jobs (commands) to run simultaneously. Defaults to the environment variable ROS_PARALLEL_JOBS and falls back to the number of CPU cores.')
#    parser.add_argument('--force-cmake', action='store_true', help='Invoke "cmake" even if it has been executed before [false]')
#    parser.add_argument('-p', '--pre-clean', action='store_true', help='Clean build temporaries before making [false]')
#    group = parser.add_mutually_exclusive_group()
#    group.add_argument('-i', '--install', action='store_true', help='Run install step after making [false]')
#    group.add_argument('-t', '--tests', action='store_true', help='Make tests [false]')
#    group.add_argument('-r', '--run_tests', action='store_true', help='Make and run tests [false]')
#    parser.add_argument('--no-color', action='store_true', help='Disables colored ouput')
#    parser.add_argument('--pkg', help='Invoke "make" on a specific package only')
#    parser.add_argument('--cmake-args', dest='cmake_args', nargs='*', type=str,
#        help='Arbitrary arguments which are passes to CMake. It must be passed after other arguments since it collects all following options.')
#    parser.add_argument('--make-args', dest='make_args', nargs='*', type=str,
#        help='Arbitrary arguments which are passes to make. It must be passed after other arguments since it collects all following options. This is only necessary in combination with --cmake-args since else all unknown arguments are passed to make anyway.')
    args = parser.parse_args()
    #(options, args) = parser.parse_args()
    print("Args...........%s"%args) 
#    try:
#        args = roslaunch.rlutil.resolve_launch_arguments(args)
#    except roslaunch.core.RLException as e:
#        raise RuntimeError(str(e))
#    return (options, args)
    return args

def test_main():
    args = _parse_arguments()
    #logfile_name = configure_logging()
    logger = logging.getLogger('rocon_test')
    roslaunch.core.add_printlog_handler(logger.info)
    roslaunch.core.add_printerrlog_handler(logger.error)
    pkg = rospkg.get_package_name(args.filename)
    rospack = rospkg.RosPack()
    pkg_dir = rospack.get_path(pkg)
    print("Pkg...........%s"%pkg)
    print("Pkg Dir...........%s"%pkg_dir)
    print("Filename..........%s"%args.filename)

