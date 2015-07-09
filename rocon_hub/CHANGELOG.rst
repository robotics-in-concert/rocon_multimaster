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
* expose hidden param in the code
* Contributors: Jihoon Lee

0.7.5 (2014-09-23)
------------------
* ttl returns -2 if key does not exist from redis 2.8
* Contributors: Jihoon Lee

0.7.4 (2014-08-25)
------------------

0.7.3 (2014-05-26)
------------------
* support for redis on trusty, version 2.8.
* remove unused gateway_ping_frequency parameter, `#271 <https://github.com/robotics-in-concert/rocon_multimaster/issues/271>`_.
* Contributors: Daniel Stonier

0.7.1 (2014-05-06)
------------------
* Expose ``gateway_unavailable_timeout`` as a configurable parameter.
* Expose ``gateway_unavailable_timeout`` as an arg to roslaunch.
* Contributors: Daniel Stonier

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

0.3.0 (2013-02-05)
------------------
* update jihoon e-mail

0.2.2 (2013-01-31)
------------------
* fix wiki links.
* catkinized.

0.1.8 (2012-12-23 13:59)
------------------------
* some exception and license handling

0.1.6 (2012-12-12)
------------------
* avoid building, rosbuild takes ages now.

0.1.4 (2012-12-08)
------------------
* generic default hub name.

0.1.3 (2012-12-07)
------------------
* rocon_gateway_hub->rocon_hub, client also.
