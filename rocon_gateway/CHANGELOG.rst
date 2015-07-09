Changelog
=========

0.7.10 (2015-07-09)
-------------------

0.7.9 (2015-07-09)
------------------
* add rosgraph dependency
* import rosgraph to handle MasterError
* skipps if node is not availalble
* update
* xmlrpc uri is also recorded now
* add xmlrpc in node
* refactoring
* refactoring flipp updates
* refactoring ros service callback functions closes `#307 <https://github.com/robotics-in-concert/rocon_multimaster/issues/307>`_
* add more prints to investigate insecure pickle closes `#302 <https://github.com/robotics-in-concert/rocon_multimaster/issues/302>`_
* increase hubconnection socket timeout to resolve frequent hub disengagement closes `#301 <https://github.com/robotics-in-concert/rocon_multimaster/issues/301>`_
* [rocon_gateway] some new convenience args.
* Contributors: Daniel Stonier, Jihoon Lee, dwlee

0.7.8 (2015-04-27)
------------------
* fix the wrong message format parsing. closes `#297 <https://github.com/robotics-in-concert/rocon_multimaster/issues/297>`_
* Contributors: Jihoon Lee

0.7.7 (2015-03-23)
------------------
* [rocon_gateway] make local gateway information available immediately.
* Contributors: Daniel Stonier

0.7.6 (2014-11-21)
------------------
* [rocon_gateway] bugfix gateway param setting
* [rocon_gateway] hint to the user what environment variable needs to be
  set if using multiple interfaces.
* Contributors: Daniel Stonier

0.7.5 (2014-09-23)
------------------
* Merge branch 'cyclic_flip' of https://github.com/robotics-in-concert/rocon_multimaster into cyclic_flip
* updating logic to store filtered flip
* Merge branch 'indigo' into cyclic_flip
* Hints for the ignorant.
* cleanup
* now it prevents cyclic flips. it addresses `#283 <https://github.com/robotics-in-concert/rocon_multimaster/issues/283>`_. Fix `#283 <https://github.com/robotics-in-concert/rocon_multimaster/issues/283>`_
* it was silently ignoring if topic being flipped does not have type information. Now it provides warning that it is not flippable
* Contributors: Daniel Stonier, Jihoon Lee

0.7.4 (2014-08-25)
------------------

0.7.3 (2014-05-26)
------------------
* lists instead of semi-colon separated strings for hub whitelist/blacklist parameters.
* keep trying to resolve zeroconf hubs instead of blacklisting them so we can come back from wireless dropouts, `#271 <https://github.com/robotics-in-concert/rocon_multimaster/issues/271>`_.
* update publisher queue_size to avoid warning in indigo.
* Contributors: Daniel Stonier

0.7.1 (2014-05-06)
------------------
* fix minor graph variable refactoring bugs.
* optimized updating flipped_in_connections. closes `#267 <https://github.com/robotics-in-concert/rocon_multimaster/issues/267>`_
* remote_gateway_info needs more time to find the concert if launched with
  other nodes at startup.
* make remote_gateway_info roslaunchable.
* use a loop period in the ``remote_gateway_info`` script.
* remove the redundant (is in rocon_gateway_utils) gateway resolver from the master api
* bugfix erroneous interpretation of boolean keys on the hub, fixes `#263 <https://github.com/robotics-in-concert/rocon_multimaster/issues/263>`_.
* handle a socket error with a warning when user sets a wireless ip to ROS_IP.
* option for putting ``remote_gateway_info`` on a loop.
* Contributors: Daniel Stonier, Jack O'Quin, Piyush Khandelwal

0.7.0 (2014-04-16)
------------------
* client to hub connection statistics
* move flip handling from pubsub channels to encrypted posting on the hubs
* disconnect/reconnect to the hub handling
* pep8 and code analysis
* catch and handle many exceptions
* multi-version support for redis servers, currently v2.2 and v2.6
* Contributors: Daniel Stonier, Jorge Santos, Marcus Liebhardt, Piyush Khandelwal, piyushk

0.6.0 (2013-08-30)
------------------
* disable uuids, tested with turtlebot, but not with turtle or chatter concerts.
* more info for error checking.
* error checking for master api register service bug.

0.5.3 (2013-07-17)
------------------
* handles for configuring and triggering the watcher loop updates.
* don't try and delete a remote gateway's advertisements for it.
* add missing dependencies on rostopic and rosservice.
* bugfix unregister subcriber update faults when subscriber has disappeared.

0.5.0 (2013-05-27 11:03)
------------------------
* fix problems with new skeletonised connection index for advertising.
* better error feedback on sample errors.
* removal of almost all xmlrpc calls when indexing local connections, and a nosetest.
* move watcher thread into main thread.
* pep8 updates
* included commented disable_zeroconf in the default parameter file.
* trivial comment clearing.
* action search algorithm got patched
* udpate master connection
* gateway master connections script.
* value error handling in the gateway master api just in case someone sends us something rotten.
* trivial pep8 naming
* big cleanupa and fix of pulled interface updates.
* better remote gateway name matching for flips.
* utility wait for remote gateway.
* bugfix missing return element in ros remote service checks.
* flip updates for actions hadn't updated for multihub - bugfixed.
* bugfix remote name matching - wasn't returning with the hash name.
* use same style for names and nodes from rocon gateway tutorials.
* trivial logging update.
* add option for connect hub test in samples.
* bugfix broken hub connections via ros service.
* match uri's in the white/blacklist correctly using urlparse
* remove legacy comment.
* properly modularising rocon_gateway and rocon_hub_client
* handle uri's in the hub blacklist as well.
* handle uri's in the whitelist.
* fix error variable bug, also convert hub lists to semi-colon separated strings so we can use args in launch files.
* parameter to force disabling zeroconf if desired.
* trivial debug message formatting.
* logging to display unique name once generated.
* fix vague debug message.
* return proper objects when aborting zeroconf_scan.
* more streamlined debugging for pulls.
* avoid shutdown exceptions when calling zeroconf services.
* remove redundant unregistered message.
* remove a debug print
* convince redis to let us die peacefully.
* more streamlined debugging messages for advertisements.
* remove a print debugger
* do not add hubs that are already connected.
* no longer used
* hub_manager split into its own module.
* direct discovery merged with zeroconf discovery.
* fix remote gateway info name-type swapped wrong.
* handle disconnecting hubs and proper lock protection on the hubs variable.
* fix gateway shutdown process for flips.
* single flip working.
* bugfix typos from pep8 updates.
* checked flip all cancelling while implementing the flip all part of a flip test.
* trivial commit - pep8 updates, eclipse settings and cleaned out test graveyard.
* flip all is working, fixed a bug in the 'is all' matching as well.
* trivial pep8 updates.
* rocon pull test added.
* single pulls working.
* pull all working.
* early work on pulling.
* provide remote gateway hash names alongside regular names
* cleaned up logging, fixed cancel bug in advertise script.
* regex pattern advertisements test.
* advertisements rocon test finished.
* various patches around pulled interfaces and remote gateway info.
* python complains if you join when a thread isn't (yet) started.
* blacklist the subscribers who eventually connect to the gateway
  publishers as well.
* add publisher for gateway info to default interface, remove some debug prints.
* publish gateway info if public interface updates.
* advertise all watchlist working.
* local gateway info is working.
* direct and zeroconf hubs now resolving.
* more updates for multihub.
* zeroconf threading working, hub sync started.
* zeroconf threading working, hub sync started.
* background hub discovery thread.
* background hub discovery thread.
* pep8 update.
* uuid's now used to generate unique gateway names.
* 0.4.0
* remove accidentally leftover debugging prints.
* graph test, not yet working.
* bugfix gateway info for the gateway graph.
* fix error when server is found, but no hub name yet set.
* loop with a connection timeout for direct connections.
* last of gateway info latched publisher changes.
* updated subscriber proxy api for flip.
* new gateway info with latched subscriber, working...just some odd implementations to update yet.

0.3.0 (2013-02-05)
------------------
* update jihoon e-mail

0.2.2 (2013-01-31)
------------------
* bugfix, was accidentally using the firewall flag for checks while pulling.
* fix wiki links.
* catkinized.
* git ignores, comments and remove unused build infra.
* fix gateway matching for pulled interfaces.
* fix remote gateway matching for flips.

0.1.8 (2012-12-23 13:59)
------------------------
* ungracefuly handling hub connection exceptions.
* eradicate unused imports.
* applied register_subscriber to the action subscribers as well.
* more expansive feedback.
* catch some errors when registering subscribers.
* typo'd some of the variable names.
* bugfix for when subscriber disappears before it can receive a server update when unflipping.
* pep8 and some cleanup.
* some exception and license handling

0.1.7 (2012-12-13)
------------------
* added xmlrpcapi calls to the action subscriber registrations.
* cancelling topics on unregistering a subscriber.
* notifies subscriber nodes of existing publishers, still to do actions and cancelling properly.

0.1.6 (2012-12-12)
------------------
* avoid building, rosbuild takes ages now.

0.1.5 (2012-12-09)
------------------
* manifest description.

0.1.4 (2012-12-08)
------------------
* bugfixes.

0.1.3 (2012-12-07)
------------------
* pep8 following... 
* resolveHub to resolve_hub. resolveAddress to resolve_address
* ip advertising, uses ROS_MASTER_URI, then ROS_IP, then ROS_HOSTNAME.
* alot of pep8, also bugfix unique name prefix '/'.
* tooltips, also highlighted local gateway.
* hide/show unused advertisements working.
* it catches topic with no leading /.
* generalize re-gex matching function
* looking good, but barely done.
* bugfix flipped in connections, it was listing flipped connections.
* pep8 and started adding a graph class (aka rosgraph.graph).
* pep8 stuff.
* bugfix rocon_hub_client rename.
* rocon_utilities with the logger console added.
* pep8 for rocon_hub.
* rocon_gateway_hub->rocon_hub, client also.
* unflipping two flips at once failed, bugfixed bad variable reference.
* gateway module in src
* gateway_info now publishes huburi as well
* starting rqt graphing, but groovy is mad right now.
* convenience/prettified gateway info script.
* convenience remote gateway info script.
* move demos to tutorials, more consistent with ros conventions.
* interactive script for pulls done.
* started on the interactive pull, but getting tangled in sleep.
* advertise script done.
* interactive flip script finished.
* hacks to fix empty nodes.
* script almost where I want it, for unflips need to parse watchlist though, not flips.
* more scripting.
* some docs for master api and also clean up remote gateway info for actions.
* bugfix pruning of publishers after action list parsing.
* working towards the convenient flipper.

0.1.2 (2012-11-24 18:09)
------------------------

0.1.1 (2012-11-24 15:57)
------------------------
* got started, but not gotten very far with the flip script.
* advertise_all in yaml.
* implemented flip_all/pull_all in yaml, advertise_all.
* started laying out what will be used for advertise/flip/pull all from parameter configuration.
* probably buggy, but regex'd gateways seems to be working with surprisingly little work.
* probably buggy, but regex'd gateways seems to be working with surprisingly little work.
* bugfix res -> python style regular expressions

0.1.0 (2012-11-14)
------------------
* comms to msgs
* flip and pull service back to using remote rules instead of gateway, rule[] combination.
* multiple rules for pulls now working as well.
* flipping with multiple flip rule service argument (array) working, pulling broken.
* started work on the demo launchers.
* added pulled interface for the remote gateway info.
* solve th gateway registration racing condition.
* commented the wrong one
* Merge branch 'master' of https://github.com/robotics-in-concert/rocon_multimaster
* comments about the gateway registration racing condition, 105.
* remove zeroconf avahi dependency.
* flipped interface information now on the redis server.
* more logical rocon:gateway:advertisements key for redis, instead of rocon:gateway:connections.
* bugfix in pull error handling, also some comment fixes
* unflipping for actions.
* renaming misleading action_interface to interactive_interface
* test fibonacci server, averaging seems broken.
* flipping fibonacci action client and successfully ran server on the other end, but unflipping fails. Also regex'ing action patterns to work with fibonacci, not fibonacci/.
* remap averaging server so averaging client works (this actionlib
  tutorial is buggy?)
* fixed bug -- logical error in remote service checks if remote gateway does not exist
* removed inapplicable print statement
* removed empty stubs for actions in master api -- action servers/clients can now be registered/unregistered with the local master
* fixing some small bugs in the pull calls
* reordered hub api shutdown. useful incase the hub shuts down. 
* moving pulled watch update alongside the public update.
* moving flip watch update alongside the public update.
* move pirate launchers back for now, too much referencing them.
* adding some action client/server tests.
* error message handling for advertise call.
* bugfix for resolving our new private hub (was by default pointing to the system hub).
* deactive flipped list for gateways that have disappeared.
* merged last of common code from flipped and pulled interfaces.
* More merging of flipped and pulled interfaces.
* starting the common active ancestor interface.
* removed a rather unused pair of functions.
* static public interfaces from yaml.
* pull interface information.
* default rules for flips and pulls. Also cleaning up gateway info ready for pull interfaces.
* remove debugging print command.
* do not uniqueify the gateway name if not necessary.
* partially doing static parameterised pulls, flips etc.
* removing cruft from master_api
* check for local service name before registering.
* synchronising lost pulls for lost gateways.
* fast pull updates.
* fast updates for advertisements.
* add firewall to the gateway info and fix a firewall bug (string to int conversion).
* early bird flip firewalling error messages.
* firewall flag on the redis server. Also cleaned up some redis handling.
* privatising hub variables.
* remove depracated hub code, broadcastTopicUpdate.
* firewall flag.
* improved watcher sleep and shutdown.
* privatising watcher thread variables.
* clearing flips and local flip registrations on shutdown.
* removed graveyard tests, started rocon_gateway_tests module for reusing test code.
* fixed license locations.
* fix flip tests, also update for rule.rule.xxx -> remote.rule.xxx in Remote.srv
* still bugs in unit test - now working.
* simplifying - moved type constants out to their own message type.
* the pull api. tested using pull-all/pull-all-cancel only. mostly a direct copy of the flip interface, though the watcher thread logic is somewhat different. seems to work fine.
* merged with upstream repo. rolled back my blacklist code as already implemented. silly me.
* 1) refactored basic messages as per discussion. 
  2) enabled blacklists in flipped interface (plus a couple of minor bug fixes)
  3) improved some of the utils serialization/deserialization functions
* publishers and subscribers no longer contain the part connections for action_clients/action_servers
* remote gateway info now working
* re-enabled getting watchlist/blacklist back in advertise/advertiseall req. this was a useful feature and does not require a manual update
* a bit more cleanup with the advertise call
* a bit of cleanup
* refactored basic structures. advertisements working, will test flipping next. 
* flipAll, unFlipAll now working with merged blacklists and updated unflipall interface.
* public_interface cleanup and multi-threaded safety, also fixed 2 minor bugs in FlippedInterface that were spotted
* merged with upstream repo
* more work on flip, flipall - almost there.
* less verbose.
* default connections blacklist from ros param list, but not using yet
* a number of bug fixes. 
* got the test to a somewhat acceptable level. also fixed public in getGatewayInfo
* flip rules can now take node arguments of 1) node name, 2) regex, 3) None.
* fixed regex matching in public interface. 
* checking in a couple of minor things left behind in manual merge
* flipped in registrations added to gateway info.
* manually accepting piyush's pull request (https://github.com/robotics-in-concert/rocon_multimaster/pull/81) since I forgot to actually press the pull request button.
* flip services fully working (unflip too).
* flip services working (unflip services broken).
* simplify directory structure.
* moving old implementation to graveyard. and resturcture the stack
* eclipse projects and gateway hub script started.
* initial package structure.
