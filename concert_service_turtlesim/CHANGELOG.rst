^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package concert_service_turtlesim
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.12 (2015-07-09)
-------------------

0.1.11 (2015-04-27)
-------------------

0.1.10 (2015-04-06)
-------------------

0.1.9 (2015-03-23)
------------------

0.1.8 (2015-02-27)
------------------
* provides args for shutdown subscriber closes `#41 <https://github.com/robotics-in-concert/concert_services/issues/41>`_
* Contributors: Jihoon Lee

0.1.7 (2015-02-09)
------------------

0.1.6 (2015-01-05)
------------------

0.1.5 (2014-12-05)
------------------

0.1.4 (2014-11-21)
------------------

0.1.3 (2014-11-21)
------------------

0.1.2 (2014-11-21)
------------------

0.1.1 (2014-08-26)
------------------
* hatchling no longer installable.
* full launcher configuration via service parameters, closes `#17 <https://github.com/robotics-in-concert/concert_services/issues/17>`_.
* use proper lists for hubs/concerts now roslaunch can handle it.
* turtlesim2 moving in, closes `#16 <https://github.com/robotics-in-concert/concert_services/issues/16>`_
* updated to use the new rocon launch api.
* upgrades for the rapp manager launch overhaul.
* minor bugfix for the teleop app due to recent refactoring.
* bugfix hatchling to work without the deprecated remote_controller publisher from the app manager, fixes `#11 <https://github.com/robotics-in-concert/concert_services/issues/11>`_.
* local machine args as for chatter concert, `#7 <https://github.com/robotics-in-concert/concert_services/issues/7>`_.
* locks around threaded worker functions which can multiply call over a single ros xmlrpc service tunnel (and thus mangle interleaving requests/responses, fixes `#9 <https://github.com/robotics-in-concert/concert_services/issues/9>`_
* remove some debug comments.
* usable configuration for service priorities.
* longer timeout for gateway discovery.
* rocon_service -> concert_service
* Contributors: Daniel Stonier
