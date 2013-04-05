#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_multimaster/hydro-devel/rocon_test/LICENSE
#

##############################################################################
# Imports
##############################################################################

import unittest
import rospkg
import roslib
import rostest.runner
import roslaunch

##############################################################################
# Methods
##############################################################################


def create_unit_test(pkg, test_file):
    """
    Unit test factory. Constructs a unittest class based on the rocon_launch

    @param pkg: package name
    @type  pkg: str
    @param test_file: rostest filename
    @type  test_file: str
    """
    # parse the config to find the test files
    config = roslaunch.parent.load_config_default([test_file], rostest.runner._DEFAULT_TEST_PORT)

    # pass in config to class as a property so that test_parent can be initialized
    classdict = {'setUp': rostest.runner.setUp, 'tearDown': rostest.runner.tearDown, 'config': config,
                  'test_parent': None, 'test_file': test_file}

    # add in the tests
    testNames = []
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

        testName = 'test%s' % (test.test_name)
        if err_msg:
            classdict[testName] = rostest.runner.failRunner(test.test_name, err_msg)
        elif testName in testNames:
            classdict[testName] = rostest.runner.failDuplicateRunner(test.test_name)
        else:
            classdict[testName] = rostest.runner.rostestRunner(test, pkg)
            testNames.append(testName)

    # instantiate the TestCase instance with our magically-created tests
    return type('RosTest', (unittest.TestCase, ), classdict)
