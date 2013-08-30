^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rocon_utilities
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.6.0 (2013-08-30)
------------------
* set a /rocon/screen parameter if rocon_launch is launching with --screen (app manager passes this down to its apps)

0.5.5 (2013-08-07)
------------------
* add rosbash as dependency

0.5.4 (2013-07-19)
------------------
* rocon_launch with a --no-terminals option.

0.5.3 (2013-07-17)
------------------

0.5.2 (2013-06-10)
------------------
* 0.5.1

0.5.1 (2013-05-27 11:48)
------------------------
* fixing roconbash
* fixing roconbash
* 0.5.0

0.5.0 (2013-05-27 11:03)
------------------------
* fix problems with new skeletonised connection index for advertising.
* formatting markers now work for strings as well (not whole lines)
* absolute path check for ros names.
* fallbacks to gnome-terminal and konsole if esoteric terminal detected.
* hack to get gnome-terminal to inherit its environment.
* search and run gnome-terminal, not gnome.
* trivial commit - pep8 updates, eclipse settings and cleaned out test graveyard.
* remove unused re import.
* consistent naming for gateway utility functions.
* restrict tab completion to .concert and .test for rocon_launch and
  rocon_test respectively.
* Checks ubuntu's x-terminal-emulator (from update-alternatives) to
  intelligently choose the terminal to use.
* tab completion for rocon_test.
* tab completion for rocon_launch.
* using uuid's for unique gateway names now.
* remove debugging prints.
* 0.4.0
* fix time issues and add wait_for_publishers method to subscriber proxy.
* make sure we return none if nothing is picked up by the subscriber.
* about to embark on a radical update to rocon tests now I know what's going on.
* test runner is working on the individual launchers.
* renamed test to launch, bit easier to track it and since it's in a test
  folder, there's no doubt that it's a launcher.
* more on rocon_test, automatic enumeration of ports for rocon_launch
* first rostest working for subscriber_proxy.
* fix test configuration file.
* first dummy python nosetest.
* more convenient structure for subscriber proxy'ing.
* new gateway info with latched subscriber, working...just some odd implementations to update yet.
* started nosetests

0.3.0 (2013-02-05)
------------------
* gateway utility functions.
* common create_rule code moved here.

0.2.2 (2013-01-31)
------------------
* removing debug print
* konsole gnome terminal working. default konsole
* Don't abort if a roslaunch process has already terminated.
* fix wiki links.
* catkinized.
* roconlaunch now enabling multiple launchers on the same port using --wait.
* make port optinal in concert launchers, with default of 11311.
* added port specification and also --screen option.
* multimaster roslaunch working for a single ros subsystem and in konsole.
* starting the multimaster launcher.
* git ignores, comments and remove unused build infra.

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
* slave api example.

0.1.6 (2012-12-12)
------------------
* remove catkin tags, how did they get in there?

0.1.5 (2012-12-09)
------------------

0.1.4 (2012-12-08)
------------------

0.1.3 (2012-12-07)
------------------
* pep8 and started adding a graph class (aka rosgraph.graph).
* rocon_utilities with the logger console added.

0.1.2 (2012-11-24 18:09)
------------------------

0.1.1 (2012-11-24 15:57)
------------------------

0.1.0 (2012-11-14)
------------------
