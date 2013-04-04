#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_multimaster/hydro-devel/rocon_test/LICENSE
#

##############################################################################
# Imports
##############################################################################

from __future__ import print_function

import os
import sys
import unittest
import rostest.runner
import roslaunch.rlutil
import roslaunch.core
import logging
import rospkg
import argparse
from argparse import RawTextHelpFormatter
from rostest.rostestutil import printRostestSummary, xmlResultsFile, \
                                createXMLRunner, rostest_name_from_path

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
    # Stop it from being a list (happens when nargs is an integer)
    args.test = args.test[0]
    #(options, args) = parser.parse_args()
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
    if not args.package:
        if not os.path.isfile(args.test):
            raise RuntimeError("Test launcher file does not exist [%s]." % args.test)
        else:
            args.package = rospkg.get_package_name(args.test)
    r = rospkg.RosPack()
    pkg_dir = r.get_path(args.package)
    outname = rostest_name_from_path(pkg_dir, args.test)  # underscored pkg_dir relative name (e.g. launch/pirate_chatter.multilaunch -> launch_pirate_chatter
    try:
        test_case = rostest.runner.createUnitTest(args.package, args.test)
        suite = unittest.TestLoader().loadTestsFromTestCase(test_case)

        is_rostest = True
        results_file = xmlResultsFile(args.package, outname, is_rostest)        
        xml_runner = createXMLRunner(args.package, outname, \
                                         results_file=results_file, \
                                         is_rostest=is_rostest)
        result = xml_runner.run(suite)
    finally:
        # really make sure that all of our processes have been killed
        test_parents = rostest.runner.getRostestParents()
        for r in test_parents:
            logger.info("finally rostest parent tearDown [%s]", r)
            r.tearDown()
        del test_parents[:]
        from roslaunch.pmon import pmon_shutdown
        logger.info("calling pmon_shutdown")
        pmon_shutdown()
        logger.info("... done calling pmon_shutdown")
    # print config errors after test has run so that we don't get caught up in .xml results
    config = rostest.runner.getConfig()
    if config:
        if config.config_errors:
            print("\n[ROSTEST WARNINGS]"+'-'*62+'\n', file=sys.stderr)
        for err in config.config_errors:
            print(" * %s"%err, file=sys.stderr)
        print('')

    # summary is worthless if textMode is on as we cannot scrape .xml results
    subtest_results = rostest.runner.getResults()
    printRostestSummary(result, subtest_results)

    if not result.wasSuccessful():
        sys.exit(1)
    elif subtest_results.num_errors or subtest_results.num_failures:
        sys.exit(2)
