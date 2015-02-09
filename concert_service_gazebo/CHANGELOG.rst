^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package concert_service_gazebo
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.7 (2015-02-09)
------------------

0.1.6 (2015-01-05)
------------------
* add robot gazebos as runpdend fixes `#36 <https://github.com/robotics-in-concert/concert_services/issues/36>`_
* update install rule for concert service gazebo fixes `#34 <https://github.com/robotics-in-concert/concert_services/issues/34>`_
* remove topic_relay closes `#35 <https://github.com/robotics-in-concert/concert_services/issues/35>`_
* Contributors: Jihoon Lee

0.1.5 (2014-12-05)
------------------
* rename gazebo service
* Contributors: Jihoon Lee

0.1.4 (2014-11-21)
------------------

0.1.3 (2014-11-21)
------------------

0.1.2 (2014-11-21)
------------------
* add workflow description closes `#29 <https://github.com/robotics-in-concert/concert_services/issues/29>`_
* rename sim to simulation
* remove unncessary rapps. they are now together with rapps for real robots
* now use turtlebot_rapps. add playground
* sim param passing
* typo in camera flipping
* Merge branch 'indigo' into use_package_export
* topic_relay script has migrated to rocon_python_utils
* leave segbot export as an example
* use package export to load robot types
* disable mux for smoother for now.
* make a map service with scan and robot pose remapping
* add make a map service
* now velocity smoother works in turtlebot. /clock flips by default
* now gazebo uses world_file in service parameter
* update readme
* tf fix
* now teleoping three robots are working
* Merge branch 'indigo' into gazebo_upgrade
* flips working
* flip_rule added
* now all three models are loading
* Contributors: Daniel Stonier, Jihoon Lee

0.1.1 (2014-08-26)
------------------
* update for the new rocon launch api, closes `#15 <https://github.com/robotics-in-concert/concert_services/issues/15>`_
* use proper lists for hubs/concerts now roslaunch can handle it.
* updated robot.launch to reflect recent changes to concert_client.launch
* Updating readme to reflect package name changes
  After a bit of thought, I've renamed the segbot specific packages. I've updated the readme here to reflect that change.
* minor bugfix for the teleop app due to recent refactoring.
* eclipse files and add to metapackage.
* Revert "renamed segbot_rocon to rocon_segbot"
  This reverts commit 31fa619b08f7798f48bef7f8004a618c3f106c17.
* renamed segbot_rocon to rocon_segbot
* changed to an unordered list
* added a README
* added documentation
* all concerts started through this service should use simulated time
* fixed package.xml and remove dependencies that were not needed
* removed segbot and turtlebot specific components, moved segbot components to segbot specific repository
* added new icon file
* gazebo service now up and running! need to orchestrate teleoperation and navigation
* ready to test with full concert service scenario
* basic version is working, but correct running is order dependent. needs investigation...
* started some initial tests of the gazebo segbot manager, although outside the services framework
* some changes from yesterday for the gazebo service. Need to look at this in more detail tonight
* ready for testing on the concert
* a bit more work towards adding the gazebo service
* separating robot specific components from the gazebo_robot_manager
* separated turtlebot and segbot files. now ready to implement segbot_robot_manager
* some initial testing with launch multiple turtlebots and segbots in gazebo outside the concert framework
* initial commit for concert_service_gazebo
* Contributors: Daniel Stonier, Piyush Khandelwal
