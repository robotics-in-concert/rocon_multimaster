Changelog
=========

0.7.10 (2015-07-09)
-------------------

0.7.9 (2015-07-09)
------------------

0.7.8 (2015-04-27)
------------------

0.7.7 (2015-03-23)
------------------

0.7.6 (2014-11-21)
------------------

0.7.5 (2014-09-23)
------------------

0.7.4 (2014-08-25)
------------------
* Merge branch 'indigo' into hydro-devel
* adding back the rostest in rocon_test
* unconfigured build_depend on 'rostest
* Contributors: Daniel Stonier, Jihoon Lee

0.7.3 (2014-05-26)
------------------
* update for the new rocon launch api.
* Contributors: Daniel Stonier

0.7.0 (2014-04-16)
------------------
* bugfix rocon test breakage due to rocon_launch arg mapping api change.
* update for recently moved modules to rocon_tools.
* utilities broken up and moved to rocon tools.
* fix rocon_test logging for ctest execution, `#200 <https://github.com/robotics-in-concert/rocon_multimaster/issues/200>`_
* fix rocon_tests, closes `#157 <https://github.com/robotics-in-concert/rocon_multimaster/issues/157>`_
* using wallsleep clock instead of rospy.sleep. progress towards `#191 <https://github.com/robotics-in-concert/rocon_multimaster/issues/191>`_
* Contributors: Daniel Stonier, Piyush Khandelwal

0.5.0 (2013-05-27 11:03)
------------------------
* trivial readme update - more explanatory first line.
* Follow up for test parent launchers to allow multiple launching on
  the one ros system for rocon tests. I did it previoously for roslaunch
  parents - this one is for test parents.
* make sure uuid's follow pre-launched configuration.
* disabling core services for parent roslaunchers.
* pause mode for rocon test implemented.
* text mode for rocon_test implemented (quite fugly though, but better than nothing).
* trivial printlog debugging for roslaunch parents in rocon_test.
* 0.4.0
* readme documentation.
* test_parent -> parent.
* regular launch parent for non-test launchers.
* bugfix a share directory variable typo.
* file shuffle.
* cmake module for add_rocon_test.
* wooha...rocon_test, a multimaster rostest is working. Actually lots of previous commits leading to this, and probably lots of bugs...but its working
* about to embark on a radical update to rocon tests now I know what's going on.
* test runner is working on the individual launchers.
* more on rocon_test, automatic enumeration of ports for rocon_launch
* basically rocon-test = rostest.
* rocon test development.
* starting a rocon test framework.

0.3.0 (2013-02-05)
------------------

0.2.2 (2013-01-31)
------------------

0.2.1 (2012-12-24)
------------------

0.2.0 (2012-12-23 14:05)
------------------------

0.1.9 (2012-12-25)
------------------

0.1.8 (2012-12-23 13:59)
------------------------

0.1.7 (2012-12-13)
------------------

0.1.6 (2012-12-12)
------------------

0.1.5 (2012-12-09)
------------------

0.1.4 (2012-12-08)
------------------

0.1.3 (2012-12-07)
------------------

0.1.2 (2012-11-24 18:09)
------------------------

0.1.1 (2012-11-24 15:57)
------------------------

0.1.0 (2012-11-14)
------------------
