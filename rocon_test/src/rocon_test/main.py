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
import rocon_utilities
import rospkg
import argparse
from argparse import RawTextHelpFormatter
from rostest.rostestutil import printRostestSummary, xmlResultsFile, \
                                createXMLRunner

# Local imports
import loggers
import runner

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
    parser.add_argument('--screen', action='store_true', help='run each roslaunch with the --screen option')
    args = parser.parse_args()
    # Stop it from being a list (happens when nargs is an integer)
    args.test = args.test[0]
    if args.screen:
        args.screen = "--screen"
    else:
        args.screen = ""
    return args


def test_main():
    args = _parse_arguments()
    if not args.package:
        if not os.path.isfile(args.test):
            raise RuntimeError("Test launcher file does not exist [%s]." % args.test)
        else:
            args.package = rospkg.get_package_name(args.test)
    rocon_launcher = rocon_utilities.find_resource(args.package, args.test)  # raises an IO error if there is a problem.
    (logger, log_name) = loggers.generate_log_name(args.package, rocon_launcher)
    #test_case = runner.create_unit_test(rocon_launcher, args.screen)
    launchers = rocon_utilities.parse_rocon_launcher(rocon_launcher, args.screen)
    for launcher in launchers:
        print("%s" % launcher)
        try:
            test_case = runner.create_unit_test(launcher['package'], launcher['path'])
            suite = unittest.TestLoader().loadTestsFromTestCase(test_case)

            is_rostest = True
            results_file = xmlResultsFile(args.package, log_name, is_rostest)
            xml_runner = createXMLRunner(args.package, log_name, \
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
            print("\n[ROSTEST WARNINGS]" + '-' * 62 + '\n', file=sys.stderr)
        for err in config.config_errors:
            print(" * %s" % err, file=sys.stderr)
        print('')

    # summary is worthless if textMode is on as we cannot scrape .xml results
    subtest_results = rostest.runner.getResults()
    printRostestSummary(result, subtest_results)

    if not result.wasSuccessful():
        sys.exit(1)
    elif subtest_results.num_errors or subtest_results.num_failures:
        sys.exit(2)
