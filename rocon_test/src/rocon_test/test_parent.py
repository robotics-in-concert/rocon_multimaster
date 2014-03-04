#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_multimaster/license/LICENSE
#
##############################################################################
# Imports
##############################################################################

from __future__ import print_function

import roslaunch

##############################################################################
# Test Launch Parent
##############################################################################


class RoconTestLaunchParent(roslaunch.parent.ROSLaunchParent):
    '''
      Drop in replacement for ROSTestLaunchParent that lets us manage
      the run id ourselves.
    '''

    def __init__(self, run_id, config, roslaunch_files, port):
        if config is None:
            raise Exception("config not initialized")
        # DJS : I'm turning off the is_core flag here as we don't ever seem
        # to need it and it messes with things (usual parallel roslaunches
        # that start up their own masters do not use it, only roscore).
        super(RoconTestLaunchParent, self).__init__(run_id, roslaunch_files, is_core=False, port=port, is_rostest=True)
        self.config = config

    def _load_config(self):
        # disable super, just in case, though this shouldn't get called
        pass

    def setUp(self):
        """
        initializes self.config and xmlrpc infrastructure
        """
        self._start_infrastructure()
        self._init_runner()

    def tearDown(self):
        if self.runner is not None:
            runner = self.runner
            runner.stop()
        self._stop_infrastructure()

    def launch(self):
        """
        perform launch of nodes, does not launch tests.  rostest_parent
        follows a different pattern of init/run than the normal
        roslaunch, which is why it does not reuse start()/spin()
        """
        if self.runner is not None:
            return self.runner.launch()
        else:
            raise Exception("no runner to launch")

    def run_test(self, test):
        """
        run the test, blocks until completion
        """
        if self.runner is not None:
            # run the test, blocks until completion
            return self.runner.run_test(test)
        else:
            raise Exception("no runner")
