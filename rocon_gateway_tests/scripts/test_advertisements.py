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
#     os.environ['ROS_MASTER_URI'] = "http://localhost:11301"
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

class TestAdvertisements(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        # Note this should be called only once by process.
        # We only run one process here, for all 3 tests
        rospy.init_node('test_advertisements')

    @classmethod
    def tearDownClass(cls):
        # shutting down process here
        pass

    def setUp(self):
        try:
            self.graph = Graph()
        except Exception as exc:
            self.fail()

    def test_advertise_all(self):
        rospy.loginfo(console.cyan + "********************************************************************" + console.reset)
        rospy.loginfo(console.cyan + "* Advertise All" + console.reset)
        rospy.loginfo(console.cyan + "********************************************************************" + console.reset)
        try:
            samples.advertise_all()
        except GatewaySampleRuntimeError as e:
            self.fail("Runtime error caught when advertising all connections.")
        time.sleep(2)
        public_interface = self._wait_for_public_interface()
        rospy.loginfo(console.cyan + "  - local gateway graph : \n%s" % self.graph._local_gateway + console.reset)
        self.assertEquals("5", str(len(public_interface)))

        self.assertEquals(len([rule for rule in public_interface if rule.name == "/add_two_ints" and rule.node == "/add_two_ints_server" and rule.type == "service"]), 1)
        self.assertEquals(len([rule for rule in public_interface if rule.name == "/chatter" and rule.node == "/talker" and rule.type == "publisher"]), 1)
        self.assertEquals(len([rule for rule in public_interface if rule.name == "/chatter" and rule.node == "/listener" and rule.type == "subscriber"]), 1)
        self.assertEquals(len([rule for rule in public_interface if rule.name == "/fibonacci/server" and rule.node == "/fibonacci_server" and rule.type == "action_server"]), 1)
        self.assertEquals(len([rule for rule in public_interface if rule.name == "/fibonacci/client" and rule.node == "/fibonacci_client" and rule.type == "action_client"]), 1)

        # Revert state
        try:
            samples.advertise_all(cancel=True)
        except GatewaySampleRuntimeError as e:
            self.fail("Runtime error caught when unadvertising all connections.")
        self._assert_cleared_public_interface()

    def test_advertise_tutorials(self):
        rospy.loginfo(console.cyan + "********************************************************************" + console.reset)
        rospy.loginfo(console.cyan + "* Advertise Tutorials" + console.reset)
        rospy.loginfo(console.cyan + "********************************************************************" + console.reset)
        try:
            samples.advertise_tutorials() 
        except GatewaySampleRuntimeError as e:
            self.fail("Runtime error caught when advertising tutorial connections.")
        time.sleep(2)
        public_interface = self._wait_for_public_interface()
        rospy.loginfo(console.cyan + "  - local gateway graph : \n%s" % self.graph._local_gateway + console.reset)
        self.assertEquals("5", str(len(public_interface)))

        self.assertEquals(len([rule for rule in public_interface if rule.name == "/add_two_ints" and rule.node == "/add_two_ints_server" and rule.type == "service"]), 1)
        self.assertEquals(len([rule for rule in public_interface if rule.name == "/chatter" and rule.node == "/talker" and rule.type == "publisher"]), 1)
        self.assertEquals(len([rule for rule in public_interface if rule.name == "/chatter" and rule.node == "/listener" and rule.type == "subscriber"]), 1)
        self.assertEquals(len([rule for rule in public_interface if rule.name == "/fibonacci/server" and rule.node == "/fibonacci_server" and rule.type == "action_server"]), 1)
        self.assertEquals(len([rule for rule in public_interface if rule.name == "/fibonacci/client" and rule.node == "/fibonacci_client" and rule.type == "action_client"]), 1)

        try:
            samples.advertise_tutorials(cancel=True) 
        except GatewaySampleRuntimeError as e:
            self.fail("Runtime error caught when unadvertising tutorial connections.")
        self._assert_cleared_public_interface()

    def test_advertise_regex_tutorials(self):
        print("\n********************************************************************")
        print("* Advertise Regex Tutorials")
        print("********************************************************************")
        try:
            samples.advertise_tutorials(regex_patterns=True) 
        except GatewaySampleRuntimeError as e:
            self.fail("Runtime error caught when advertising tutorial connections.")
        time.sleep(2)
        public_interface = self._wait_for_public_interface()
        print("%s" % self.graph._local_gateway)
        self.assertEquals("5", str(len(public_interface)))

        self.assertEquals(len([rule for rule in public_interface if rule.name == "/add_two_ints" and rule.node == "/add_two_ints_server" and rule.type == "service"]), 1)
        self.assertEquals(len([rule for rule in public_interface if rule.name == "/chatter" and rule.node == "/talker" and rule.type == "publisher"]), 1)
        self.assertEquals(len([rule for rule in public_interface if rule.name == "/chatter" and rule.node == "/listener" and rule.type == "subscriber"]), 1)
        self.assertEquals(len([rule for rule in public_interface if rule.name == "/fibonacci/server" and rule.node == "/fibonacci_server" and rule.type == "action_server"]), 1)
        self.assertEquals(len([rule for rule in public_interface if rule.name == "/fibonacci/client" and rule.node == "/fibonacci_client" and rule.type == "action_client"]), 1)

        try:
            samples.advertise_tutorials(cancel=True, regex_patterns=True) 
        except GatewaySampleRuntimeError as e:
            self.fail("Runtime error caught when unadvertising tutorial connections.")
        self._assert_cleared_public_interface()
        
    def tearDown(self):
        pass

    ##########################################################################
    # Utility methods
    ##########################################################################

    def _wait_for_public_interface(self):
        public_interface = None
        while not public_interface:
            self.graph.update()
            public_interface = self.graph._local_gateway.public_interface
            rospy.rostime.wallsleep(0.2)
        return public_interface

    def _assert_cleared_public_interface(self):
        start_time = rospy.Time.now()
        while True:
            self.graph.update()
            rospy.loginfo(console.cyan + "  - getting public interface" + console.reset)
            public_interface = self.graph._local_gateway.public_interface
            rospy.loginfo(console.cyan + "  - public interface: \n" + console.reset + "%s" % public_interface)
            if not public_interface:
                result = "cleared"
                break
            else:
                rospy.rostime.wallsleep(0.2)
            if rospy.Time.now() - start_time > rospy.Duration(10.0):
                result = "timed out waiting for public interface to clear"
                break
        self.assertEqual("cleared", result)

NAME = 'test_advertisements'
if __name__ == '__main__':
    #setup_module()  # because this is not nose
    rosunit.unitrun('test_advertisements', NAME, TestAdvertisements, sys.argv, coverage_packages=['rocon_gateway'])
    #teardown_module()  # because this is not nose

