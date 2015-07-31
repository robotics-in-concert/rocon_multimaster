Changelog
=========

0.7.10 (2015-07-09)
-------------------
* [rocon_hub_client] bugfix boolean check of paired ping_hub return value.
  This fixes `#312 <https://github.com/robotics-in-concert/rocon_multimaster/issues/312>`_.
* Contributors: Daniel Stonier

0.7.9 (2015-07-09)
------------------
* uses rospy wall time sleep instead of python sleep `#308 <https://github.com/robotics-in-concert/rocon_multimaster/issues/308>`_
* adding a 5s retry in case we couldnt read the rocon:hub:name from redis yet.
* update timeouts `#302 <https://github.com/robotics-in-concert/rocon_multimaster/issues/302>`_
* increate socket timeout to ping frequency
* setting default timeout in ping hub
* increase hubconnection socket timeout to resolve frequent hub disengagement closes `#301 <https://github.com/robotics-in-concert/rocon_multimaster/issues/301>`_
* Contributors: AlexV, Jihoon Lee, dwlee

0.7.8 (2015-04-27)
------------------

0.7.7 (2015-03-23)
------------------

0.7.6 (2014-11-21)
------------------
* [rocon_gateway] bugfix gateway param setting
* Contributors: Daniel Stonier

0.7.5 (2014-09-23)
------------------
* rename _ to reason
* add logic to handle hub connection
* increase the timeout for late launching zero conf than gateway
* Contributors: Jihoon Lee, dwlee

0.7.4 (2014-08-25)
------------------

0.7.3 (2014-05-26)
------------------
* lists instead of semi-colon separated strings for hub whitelist/blacklist parameters.
* keep trying to resolve zeroconf hubs instead of blacklisting them so we can come back from wireless dropouts, `#271 <https://github.com/robotics-in-concert/rocon_multimaster/issues/271>`_.
* Contributors: Daniel Stonier

0.6.1 (2013-09-11)
------------------
* allow hostname:port without scheme for direct hub identification. Better handling of poorly formatted direct hub uris.
* rediscover direct hubs if lost. 

0.5.0 (2013-05-27 11:03)
------------------------
* revert back to faster looping - optimisation issues were not here.
* bugfix for when zeroconf is not around.
* persistent service proxy.
* trivial documentation updates.
* hub white/blacklist checks for hubs.
* graveyard is rip.
* match uri's in the white/blacklist correctly using urlparse
* properly modularising rocon_gateway and rocon_hub_client
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
* pep8 stuff.
* some exception and license handling

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
* resolveHub to resolve_hub. resolveAddress to resolve_address
* pep8 stuff.
* bugfix rocon_hub_client rename.
* rocon_gateway_hub->rocon_hub, client also.

0.1.2 (2012-11-24 18:09)
------------------------

0.1.1 (2012-11-24 15:57)
------------------------

0.1.0 (2012-11-14)
------------------
