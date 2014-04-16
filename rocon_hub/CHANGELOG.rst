Changelog
=========

0.7.0 (2014-04-16)
------------------
* support for redis server v2.6
* update for recently moved modules to rocon_tools, also platform_tuples module refactoring.
* interpret time_since_last_seen as an int, not a float. `#233 <https://github.com/robotics-in-concert/rocon_multimaster/issues/233>`_
* pep8
* bugfix wired alive ping status' on the hub, closes `#224 <https://github.com/robotics-in-concert/rocon_multimaster/issues/224>`_
* gateway to hub pinger
* parameterise the external shutdown timeout.
* external shutdown hooks for gateway and hub.
* variable length split, so pull from the start, not the back, closes `#211 <https://github.com/robotics-in-concert/rocon_multimaster/issues/211>`_
* better pretty printing in remote_gateway_info_script. update time_last_seen after the first failure as well
* graceful error handling for ping output parse failure. closes `#185 <https://github.com/robotics-in-concert/rocon_multimaster/issues/185>`_
* connection statistics on the hub.
* reverting default gateway dropout time to 2 hours
* clean out old redis home directory when restarting the hub, closes `#166 <https://github.com/robotics-in-concert/rocon_multimaster/issues/166>`_
* ensure that gateway_info is completely available before adding a ping to that machine. closes `#198 <https://github.com/robotics-in-concert/rocon_multimaster/issues/198>`_
* first draft of the hub sidekick is complete. closes `#182 <https://github.com/robotics-in-concert/rocon_multimaster/issues/182>`_. closes `#183 <https://github.com/robotics-in-concert/rocon_multimaster/issues/183>`_
* using wallsleep clock instead of rospy.sleep. progress towards `#191 <https://github.com/robotics-in-concert/rocon_multimaster/issues/191>`_
* Contributors: Daniel Stonier, Piyush Khandelwal

0.5.0 (2013-05-27 11:03)
------------------------
* direct and zeroconf hubs now resolving.
* trivial comment update.
* background hub discovery thread.
* use args instead of reproducing launcher content.
* uuid's now used to generate unique gateway names.
* 0.4.0

0.3.0 (2013-02-05)
------------------
* update jihoon e-mail

0.2.2 (2013-01-31)
------------------
* fix wiki links.
* catkinized.

0.2.1 (2012-12-24)
------------------

0.2.0 (2012-12-23 14:05)
------------------------

0.1.9 (2012-12-25)
------------------

0.1.8 (2012-12-23 13:59)
------------------------
* some exception and license handling

0.1.7 (2012-12-13)
------------------

0.1.6 (2012-12-12)
------------------
* avoid building, rosbuild takes ages now.

0.1.5 (2012-12-09)
------------------

0.1.4 (2012-12-08)
------------------
* generic default hub name.

0.1.3 (2012-12-07)
------------------
* rocon_gateway_hub->rocon_hub, client also.

0.1.2 (2012-11-24 18:09)
------------------------

0.1.1 (2012-11-24 15:57)
------------------------

0.1.0 (2012-11-14)
------------------
