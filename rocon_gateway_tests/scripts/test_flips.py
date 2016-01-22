#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_multimaster/hydro_devel/rocon_gateway_tests/LICENSE
#
##############################################################################
# Imports
##############################################################################

import time
import os
import sys
try:
    import pyros_setup
    import rospy
    import rocon_console.console as console
    from rocon_gateway import samples
    from rocon_gateway import GatewaySampleRuntimeError
    from rocon_gateway import Graph
    from rocon_gateway import GatewayError
    import unittest
    import rosunit
    import roslaunch
except ImportError:
    raise

    # import pyros_setup
    # sys.modules["pyros_setup"] = pyros_setup.delayed_import_auto(base_path=os.path.join(os.path.dirname(__file__), '..', '..', '..', '..'))
    # import rospy
    # import rocon_console.console as console
    # from rocon_gateway import samples
    # from rocon_gateway import GatewaySampleRuntimeError
    # from rocon_gateway import Graph
    # from rocon_gateway import GatewayError
    # import unittest
    # import rosunit
    # import roslaunch

# roscore_process = None
# master = None
# launch = None


# def setup_module():
#     global master
#     global roscore_process
#     os.environ['ROS_MASTER_URI'] = "http://localhost:11331"
#     master, roscore_process = pyros_setup.get_master()
#     assert master.is_online()
#
#     global launch
#     # initialize the launch environment first
#     launch = roslaunch.scriptapi.ROSLaunch()
#     launch.start()


# def teardown_module():
#     # finishing all process
#     if roscore_process is not None:
#         roscore_process.terminate()  # make sure everything is stopped
#     rospy.signal_shutdown('test complete')
#     while roscore_process and roscore_process.is_alive():
#         time.sleep(0.2)  # waiting for roscore to die
#     assert not (roscore_process and master.is_online())


##############################################################################
# Main
##############################################################################

class TestFlips(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        # Note this should be called only once by process.
        # We only run one process here, for all 3 tests
        rospy.init_node('test_pullments')

    @classmethod
    def tearDownClass(cls):
        # shutting down process here
        pass

    def setUp(self):
        self.graph = Graph()

    def test_flip_all(self):
        print(console.bold + "\n********************************************************************" + console.reset)
        print(console.bold + "* Flip All" + console.reset)
        print(console.bold + "********************************************************************" + console.reset)
        try:
            samples.flip_all()
        except GatewaySampleRuntimeError as e:
            self.fail("Runtime error caught when flipping all connections [%s]" % str(e))
        flipped_interface = self._wait_for_flipped_interface()
        rospy.loginfo(console.cyan + "  - local gateway graph : \n%s" % self.graph._local_gateway + console.reset)
        self.assertEquals("5", str(len(flipped_interface)))

        self.assertEquals(len([flip for flip in flipped_interface if flip.remote_rule.rule.name == "/add_two_ints" and flip.remote_rule.rule.node.split(',')[0] == "/add_two_ints_server" and flip.remote_rule.rule.type == "service"]), 1)
        self.assertEquals(len([flip for flip in flipped_interface if flip.remote_rule.rule.name == "/chatter" and flip.remote_rule.rule.node.split(',')[0] == "/talker" and flip.remote_rule.rule.type == "publisher"]), 1)
        self.assertEquals(len([flip for flip in flipped_interface if flip.remote_rule.rule.name == "/chatter" and flip.remote_rule.rule.node.split(',')[0] == "/listener" and flip.remote_rule.rule.type == "subscriber"]), 1)
        self.assertEquals(len([flip for flip in flipped_interface if flip.remote_rule.rule.name == "/fibonacci/server" and flip.remote_rule.rule.node.split(',')[0] == "/fibonacci_server" and flip.remote_rule.rule.type == "action_server"]), 1)
        self.assertEquals(len([flip for flip in flipped_interface if flip.remote_rule.rule.name == "/fibonacci/client" and flip.remote_rule.rule.node.split(',')[0] == "/fibonacci_client" and flip.remote_rule.rule.type == "action_client"]), 1)

        result = self._wait_for_accepted_flipped_interface()
        self.assertTrue(result)
        # Revert state
        try:
            samples.flip_all(cancel=True)
        except GatewaySampleRuntimeError as e:
            self.fail("Runtime error caught when unflipping all connections [%s]" % str(e))
        self._assert_cleared_flipped_interface()

    def test_flip_tutorials(self):
        print(console.bold + "\n********************************************************************" + console.reset)
        print(console.bold + "* Flip Tutorials" + console.reset)
        print(console.bold + "********************************************************************" + console.reset)
        try:
            samples.flip_tutorials()
        except GatewaySampleRuntimeError as e:
            self.fail("Runtime error caught when flipping tutorial connections.")
        flipped_interface = self._wait_for_flipped_interface()
        rospy.loginfo(console.cyan + "  - local gateway graph : \n%s" % self.graph._local_gateway + console.reset)
        self.assertEquals("5", str(len(flipped_interface)))

        self.assertEquals(len([remote_rule for remote_rule in flipped_interface if remote_rule.remote_rule.rule.name == "/add_two_ints" and remote_rule.remote_rule.rule.node.split(',')[0] == "/add_two_ints_server" and remote_rule.remote_rule.rule.type == "service"]), 1)
        self.assertEquals(len([remote_rule for remote_rule in flipped_interface if remote_rule.remote_rule.rule.name == "/chatter" and remote_rule.remote_rule.rule.node.split(',')[0] == "/talker" and remote_rule.remote_rule.rule.type == "publisher"]), 1)
        self.assertEquals(len([remote_rule for remote_rule in flipped_interface if remote_rule.remote_rule.rule.name == "/chatter" and remote_rule.remote_rule.rule.node.split(',')[0] == "/listener" and remote_rule.remote_rule.rule.type == "subscriber"]), 1)
        self.assertEquals(len([remote_rule for remote_rule in flipped_interface if remote_rule.remote_rule.rule.name == "/fibonacci/server" and remote_rule.remote_rule.rule.node.split(',')[0] == "/fibonacci_server" and remote_rule.remote_rule.rule.type == "action_server"]), 1)
        self.assertEquals(len([remote_rule for remote_rule in flipped_interface if remote_rule.remote_rule.rule.name == "/fibonacci/client" and remote_rule.remote_rule.rule.node.split(',')[0] == "/fibonacci_client" and remote_rule.remote_rule.rule.type == "action_client"]), 1)

        result = self._wait_for_accepted_flipped_interface()
        self.assertTrue(result)
        try:
            samples.flip_tutorials(cancel=True)
        except GatewaySampleRuntimeError as e:
            self.fail("Runtime error caught when unflipping tutorial connections.")
        self._assert_cleared_flipped_interface()

    def test_flip_regex_tutorials(self):
        print(console.bold + "\n********************************************************************" + console.reset)
        print(console.bold + "* Flip Regex Tutorials" + console.reset)
        print(console.bold + "********************************************************************" + console.reset)
        try:
            samples.flip_tutorials(regex_patterns=True)
        except GatewaySampleRuntimeError as e:
            self.fail("Runtime error caught when flipping tutorial connections.")
        flipped_interface = self._wait_for_flipped_interface()
        rospy.loginfo(console.cyan + "  - local gateway graph : \n%s" % self.graph._local_gateway + console.reset)
        self.assertEquals("5", str(len(flipped_interface)))

        self.assertEquals(len([remote_rule for remote_rule in flipped_interface if remote_rule.remote_rule.rule.name == "/add_two_ints" and remote_rule.remote_rule.rule.node.split(',')[0] == "/add_two_ints_server" and remote_rule.remote_rule.rule.type == "service"]), 1)
        self.assertEquals(len([remote_rule for remote_rule in flipped_interface if remote_rule.remote_rule.rule.name == "/chatter" and remote_rule.remote_rule.rule.node.split(',')[0] == "/talker" and remote_rule.remote_rule.rule.type == "publisher"]), 1)
        self.assertEquals(len([remote_rule for remote_rule in flipped_interface if remote_rule.remote_rule.rule.name == "/chatter" and remote_rule.remote_rule.rule.node.split(',')[0] == "/listener" and remote_rule.remote_rule.rule.type == "subscriber"]), 1)
        self.assertEquals(len([remote_rule for remote_rule in flipped_interface if remote_rule.remote_rule.rule.name == "/fibonacci/server" and remote_rule.remote_rule.rule.node.split(',')[0] == "/fibonacci_server" and remote_rule.remote_rule.rule.type == "action_server"]), 1)
        self.assertEquals(len([remote_rule for remote_rule in flipped_interface if remote_rule.remote_rule.rule.name == "/fibonacci/client" and remote_rule.remote_rule.rule.node.split(',')[0] == "/fibonacci_client" and remote_rule.remote_rule.rule.type == "action_client"]), 1)

        result = self._wait_for_accepted_flipped_interface()
        self.assertTrue(result)
        try:
            samples.flip_tutorials(cancel=True, regex_patterns=True)
        except GatewaySampleRuntimeError as e:
            self.fail("Runtime error caught when unflipping tutorial connections.")
        self._assert_cleared_flipped_interface()

    def tearDown(self):
        pass

    ##########################################################################
    # Utility methods
    ##########################################################################

    def _wait_for_accepted_flipped_interface(self):
        result = False
        start_time = rospy.Time.now()
        while not result and (rospy.Time.now() - start_time) < rospy.Duration(5.0):
            rospy.sleep(0.1)
            self.graph.update()
            accepted = True
            flipped_interface = self.graph._local_gateway.flipped_connections
            if not flipped_interface:
                continue
            for remote_rule in flipped_interface:
                result = True
                if remote_rule.status != 'accepted':
                    result = False
                    break
        return result

    def _wait_for_flipped_interface(self):
        flipped_interface = None
        while not flipped_interface:
            self.graph.update()
            flipped_interface = self.graph._local_gateway.flipped_connections
            rospy.sleep(0.2)
        return flipped_interface

    def _assert_cleared_flipped_interface(self):
        start_time = rospy.Time.now()
        while True:
            self.graph.update()
            flipped_interface = self.graph._local_gateway.flipped_connections
            if not flipped_interface:
                result = "cleared"
                break
            else:
                rospy.sleep(0.2)
            if rospy.Time.now() - start_time > rospy.Duration(1.0):
                result = "timed out waiting for flipped interface to clear"
                break
        self.assertEqual("cleared", result)

NAME = 'test_flips'
if __name__ == '__main__':
    #setup_module()  # because this is not nose
    rosunit.unitrun('test_flips', NAME, TestFlips, sys.argv, coverage_packages=['rocon_gateway'])
    #teardown_module()  # because this is not nose
