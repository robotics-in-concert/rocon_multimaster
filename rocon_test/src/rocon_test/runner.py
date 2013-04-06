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
import itertools
from urlparse import urlparse
import rospkg
import roslib
import rostest.runner
import roslaunch
from rostest.rostest_parent import ROSTestLaunchParent
import rocon_utilities
import rosunit

# Local imports
import loggers
from loggers import printlog, printlogerr

##############################################################################
# Globals
##############################################################################

_DEFAULT_TEST_PORT = 22422
logger = loggers.logger
_results = rosunit.junitxml.Result('rocon_test', 0, 0, 0)


def get_results():
    return _results


def _accumulate_results(results):
    _results.accumulate(results)


##############################################################################
# Parents
##############################################################################

# global store of all ROSLaunchRunners so we can do an extra shutdown
# in the rare event a tearDown fails to execute
_test_parents = []
# Is _config actually used anywhere?
_config = None


def _add_rocon_test_parent(runner):
    global _test_parents, _config
    logger.info("_addRocontestParent [%s]", runner)
    _test_parents.append(runner)
    _config = runner.config


def get_rocon_test_parents():
    return _test_parents


##############################################################################
# Construction Methods
##############################################################################

class RoconTestLaunchConfiguration():
    '''
      Provides configuration details for a rocon test launch configuration
      associated with a single master.
    '''
    def __init__(self):
        self.launcher = None  # path to the launcher
        self.launch_configuration = None  # ros launch configuration info
        self.test_parent = None  # ros test launcher parent, handles start, stop, shutdown


def setUp(self):
    '''
      Function that becomes TestCase.setUp()

      For each launcher representing a launch on a single master in the rocon
      launcher, this makes sure there is an entity that is to be the 'parent'
      who will later be responsible for shutting everything down.

      This could be prone to problems if someone puts multiple launchers in
      rocon launcher with the same master port (untested).
    '''
    # new test_parent for each run. we are a bit inefficient as it would be possible to
    # reuse the roslaunch base infrastructure for each test, but the roslaunch code
    # is not abstracted well enough yet
    self.test_parents = []
    for (launcher, config) in itertools.izip(self.launchers, self.configs):
        o = urlparse(config.master.uri)
        test_parent = ROSTestLaunchParent(config, [launcher["path"]], port=o.port)
        #test_parent = ROSTestLaunchParent(config, [self.test_file], port=o.port)
        self.test_parents.append(test_parent)
        test_parent.setUp()
        # the config attribute makes it easy for tests to access the ROSLaunchConfig instance
        config = test_parent.config
        _add_rocon_test_parent(test_parent)
        printlog("Setup Test Parent ..................%s" % self.test_file)
        printlog("  Run Id............................%s" % test_parent.run_id)
        printlog("  File..............................%s" % launcher["path"])
        printlog("  Port..............................%s" % o.port)


def tearDown(self):
    '''
      Function that becomes TestCase.tearDown()
    '''
    printlog("Tear Down...........................%s" % self.test_file)
    if self.test_parents:
        for test_parent in self.test_parents:
            test_parent.tearDown()
    printlog("  Status............................complete")


## generate test failure if tests with same name in launch file
def fail_duplicate_runner(test_name):
    def fn(self):
        print("Duplicate tests named [%s] in rostest suite" % test_name)
        self.fail("Duplicate tests named [%s] in rostest suite" % test_name)
    return fn


def fail_runner(test_name, message):
    def fn(self):
        print(message, file=sys.stderr)
        self.fail(message)
    return fn


def rocon_test_runner(test, test_config, test_pkg):
    """
    Test function generator that takes in a roslaunch Test object and
    returns a class instance method that runs the test. TestCase
    setUp() is responsible for ensuring that the rest of the roslaunch
    state is correct and tearDown() is responsible for tearing
    everything down cleanly.
    @param test: ros test to run
    @type  test: roslaunch.Test
    @return: function object to run testObj
    @rtype: fn
    """

    ## test case pass/fail is a measure of whether or not the test ran
    def fn(self):
        printlog("Launching tests")
        done = False
        while not done:
            self.assert_(self.test_parents is not None, "ROSTestParent initialization failed")
            test_name = test.test_name
            printlog("  Name..............................%s" % test_name)

            #launch the other nodes
            for test_parent in self.test_parents:
                unused_succeeded, failed = test_parent.launch()
                self.assert_(not failed, "Test Fixture Nodes %s failed to launch" % failed)

            #setup the test
            # - we pass in the output test_file name so we can scrape it
            test_file = rosunit.xml_results_file(test_pkg, test_name, False)
            if os.path.exists(test_file):
                printlog("removing previous test results file [%s]", test_file)
                os.remove(test_file)

            # TODO: have to redeclare this due to a bug -- this file
            # needs to be renamed as it aliases the module where the
            # constant is elsewhere defined. The fix is to rename
            # rostest.py
            XML_OUTPUT_FLAG = '--gtest_output=xml:'  # use gtest-compatible flag

            test.args = "%s %s%s" % (test.args, XML_OUTPUT_FLAG, test_file)

            # run the test, blocks until completion
            printlog("Running test %s" % test_name)
            timeout_failure = False
            for test_parent in self.test_parents:
                try:
                    printlog("Test parent......................%s" % test_parent)
                    test_parent.run_test(test)
                except roslaunch.launch.RLTestTimeoutException as unused_e:
                    if test.retry:
                        timeout_failure = True
                    else:
                        raise

            if not timeout_failure:
                printlog("test [%s] finished" % test_name)
            else:
                printlogerr("test [%s] timed out" % test_name)

            # load in test_file
            if timeout_failure:

                if not timeout_failure:
                    self.assert_(os.path.isfile(test_file), "test [%s] did not generate test results" % test_name)
                    printlog("test [%s] results are in [%s]", test_name, test_file)
                    results = rosunit.junitxml.read(test_file, test_name)
                    test_fail = results.num_errors or results.num_failures
                else:
                    test_fail = True

                if test.retry > 0 and test_fail:
                    test.retry -= 1
                    printlog("test [%s] failed, retrying. Retries left: %s" % (test_name, test.retry))
                    self.tearDown()
                    self.setUp()
                else:
                    done = True
                    _accumulate_results(results)
                    printlog("test [%s] results summary: %s errors, %s failures, %s tests",
                             test_name, results.num_errors, results.num_failures, results.num_tests)

                    #self.assertEquals(0, results.num_errors, "unit test reported errors")
                    #self.assertEquals(0, results.num_failures, "unit test reported failures")
            else:
                if test.retry:
                    printlogerr("retry is disabled in --text mode")
                done = True
        printlog("[ROCON_TEST] test [%s] done", test_name)
    return fn


def create_unit_rocon_test(rocon_launcher, launchers):
    '''
      Constructs the python unit test class.

      @param rocon_launcher : absolute path to the rocon test launcher
      @param launchers : list of individual launcher configurations (name, path, port etc) in rocon_launcher
      @return unittest.TestCase : unit test class
    '''
    for launcher in launchers:
        print("  Path: %s" % launcher['path'])
        print("  Port: %s" % launcher['port'])
    configs = [roslaunch.parent.load_config_default([launcher['path']], launcher['port']) for launcher in launchers]
    # pass in config to class as a property so that test_parent can be initialized
    classdict = {'setUp': setUp, 'tearDown': tearDown,
                 'launchers': launchers, 'configs': configs,
                  'test_parents': None, 'test_file': rocon_launcher}

    # add in the tests
    test_names = []
    for (launcher, config) in itertools.izip(launchers, configs):
        for test in config.tests:
            print("test")
            # #1989: find test first to make sure it exists and is executable
            err_msg = None
            try:
                rp = rospkg.RosPack()
                cmd = roslib.packages.find_node(test.package, test.type, rp)
                if not cmd:
                    err_msg = "Test node [%s/%s] does not exist or is not executable" % (test.package, test.type)
            except rospkg.ResourceNotFound:
                err_msg = "Package [%s] for test node [%s/%s] does not exist" % (test.package, test.package, test.type)

            test_name = 'test%s' % (test.test_name)
            if err_msg:
                classdict[test_name] = fail_runner(test.test_name, err_msg)
            elif test_name in test_names:
                classdict[test_name] = fail_duplicate_runner(test.test_name)
            else:
                classdict[test_name] = rocon_test_runner(test, config, launcher["package"])
                test_names.append(test_name)

    # instantiate the TestCase instance with our magically-created tests
    return type('RoconTest', (unittest.TestCase,), classdict)


# test_case = runner.create_unit_test(launcher['package'], launcher['path'])

def create_unit_test(pkg, test_file):
    """
    Unit test factory. Constructs a unittest class based on the rocon_launch

    @param pkg: package name
    @type  pkg: str
    @param test_file: rostest filename
    @type  test_file: str
    """
    # parse the config to find the test files
    config = roslaunch.parent.load_config_default([test_file], _DEFAULT_TEST_PORT)
    # pass in config to class as a property so that test_parent can be initialized
    classdict = {'setUp': rostest.runner.setUp, 'tearDown': rostest.runner.tearDown, 'config': config,
                  'test_parent': None, 'test_file': test_file}

    # add in the tests
    test_names = []
    for test in config.tests:
        # #1989: find test first to make sure it exists and is executable
        err_msg = None
        try:
            rp = rospkg.RosPack()
            cmd = roslib.packages.find_node(test.package, test.type, rp)
            if not cmd:
                err_msg = "Test node [%s/%s] does not exist or is not executable" % (test.package, test.type)
        except rospkg.ResourceNotFound:
            err_msg = "Package [%s] for test node [%s/%s] does not exist" % (test.package, test.package, test.type)

        test_name = 'test%s' % (test.test_name)
        if err_msg:
            classdict[test_name] = rostest.runner.FailRunner(test.test_name, err_msg)
        elif test_name in test_names:
            classdict[test_name] = rostest.runner.failDuplicateRunner(test.test_name)
        else:
            classdict[test_name] = rostest.runner.rostestRunner(test, pkg)
            test_names.append(test_name)

    # instantiate the TestCase instance with our magically-created tests
    return type('RosTest', (unittest.TestCase,), classdict)
