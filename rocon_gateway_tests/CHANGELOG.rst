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
* rocon_gateway_tests: CMakeLists.txt(12): error: missing COMPONENTS keyword before 'rocon_test'
* Contributors: Jihoon Lee

0.7.3 (2014-05-26)
------------------
* reduce the gateway unavailable timeout for convenient wireless testing.
* fix inconsistently referenced flip_all.yaml.
* Contributors: Daniel Stonier

0.7.2 (2014-05-06)
------------------
* added missing zeroconf_avahi dependency.
* relax timeout durations for tests.

0.5.0 (2013-05-27 11:03)
------------------------
* loading up the heavy tutorials.
* add missing tutorials light launcher
* fix problems with new skeletonised connection index for advertising.
* removal of almost all xmlrpc calls when indexing local connections, and a nosetest.
* Benchmarking tests for the gateway master api.
* move watcher thread into main thread.
* benchmarking utilities.
* connect hub by service test.
* fix remote gateway info name-type swapped wrong.
* handle disconnecting hubs and proper lock protection on the hubs variable.
* fix gateway shutdown process for flips.
* single flip working.
* checked flip all cancelling while implementing the flip all part of a flip test.
* trivial commit - pep8 updates, eclipse settings and cleaned out test graveyard.
* flip all is working, fixed a bug in the 'is all' matching as well.
* rocon pull test added.
* single pulls working.
* pull all working.
* early work on pulling.
* add a remote gateway for testing
* trivial tests script, xml cleanup.
* regex pattern advertisements test.
* advertisements rocon test finished.
* text mode for rocon_test implemented (quite fugly though, but better than nothing). 
* advertise and graph tests working, but not fully functional yet.
* rename to avoid tab completion.
* publish gateway info if public interface updates.
* advertise all watchlist working.
* local gateway info is working.
* direct and zeroconf hubs now resolving.
* background hub discovery thread.
* use args instead of reproducing launcher content.
* 0.4.0
* build depend on rocon_test
* add the rocon test into cmake.
* remove mainpage.dox
* first gateway test working - tests a graph update.
* graph test, not yet working.
* loop with a connection timeout for direct connections.
* test runner is working on the individual launchers.

0.3.0 (2013-02-05)
------------------
* update jihoon e-mail

0.2.2 (2013-01-31)
------------------
* comment out non-catkin actionlib_tutorials for now.
* fix wiki links.
* catkinized.

0.1.7 (2012-12-13)
------------------
* disabling tests temporarily.

0.1.3 (2012-12-07)
------------------
* starting to get my head into rosgraph, about to apply.
* pep8 and started adding a graph class (aka rosgraph.graph).
* rocon_gateway_hub->rocon_hub, client also.

0.1.1 (2012-11-24 15:57)
------------------------
* updated tutorial stack dependencies.

0.1.0 (2012-11-14)
------------------
* comms to msgs
* fix the test for the updated remote arrays, again.
* minor modifications to tests for multiple flip/pull updates.
* all demo code now moved to rocon_gateway_demos.
* multiple rules for pulls now working as well.
* flipping with multiple flip rule service argument (array) working, pulling broken.
* minor fixes to launchers and manifests.
* correcting error in usage
* remove old readme
* some pull testing, all seems ok.
* simple launchers to test the action_client and server fibonacci flip/pulls.
* test fibonacci server, averaging seems broken.
* bugfix typo for spelling of fibbonaci, err fibonacci.
* added a pull all test -- currently succeeds.  
* fixing some small bugs in the pull calls
* fixed test name in CMakelists.txt
* renamed test - adding pull tests to same file now
* enabled testing advertisements remotely -- will add tests for pull interface shortly
* simple text console output update.
* some initial work towards unit tests testing advertisements remotely - after this is done will proceed to pull
* reorganized launch file structure
* a large number of different advertise calls are tested locally through the public interface, also added to CMakelists.txt
* finally got the tests working to a level I like -- will shortly replicate tests for testing advertisements remotely and pull
* fixed the automated unit tests -- a lot of work to be done
* move pirate launchers back for now, too much referencing them.
* adding some action client/server tests.
* merged last of common code from flipped and pulled interfaces.
* More merging of flipped and pulled interfaces.
* trivial comment update.
* fast pull updates.
* advertise all test script.
* fast updates for advertisements.
* advertise_chatter test -> advertise_tutorials test.
* firewall flag on the redis server. Also cleaned up some redis handling.
* removed graveyard tests, started rocon_gateway_tests module for reusing test code.
* fix flip tests, also update for rule.rule.xxx -> remote.rule.xxx in Remote.srv
* still bugs in unit test - now working.
* simplifying - moved type constants out to their own message type.
* re-enabled getting watchlist/blacklist back in advertise/advertiseall req. this was a useful feature and does not require a manual update
* refactored basic structures. advertisements working, will test flipping next.
* default connections blacklist from ros param list, but not using yet.
* got the test to a somewhat acceptable level. also fixed public in getGatewayInfo
* flip rules can now take node arguments of 1) node name, 2) regex, 3) None.
* checking in a couple of minor things left behind in manual merge
* flip services working (unflip services broken)
* simplify directory structure.
