# Using the Concert Framework with Gazebo (the short short version)

There is some code that is robot-agnostic. All that code is in this package in the [GazeboRobotManager Module](src/concert_service_gazebo/gazebo_robot_manager.py)

## GazeboRobotManager

* Creates concert clients for each robot
* Flips necessary information to that robot's concert client, so that it can pretend to be a real robot.

Unfortunately, a lot of code depends on the robot that you are going to simulate. For each robot, you'll have to implement a module specific to that robot. An abstract module detailing this API is [RobotManager](src/concert_service_gazebo/robot_manager.py)  

## RobotManager

* Specifies how to spawn the robot in Gazebo
* Specifies how to delete the robot in Gazebo
* Specifies which information needs to be flipped over. If you're doing teleop, cmd_vel is sufficient. If you want to do full navigation, you'll have to supply tf,map,sensory information and odometry as well.
* Specifies how each concert client for these simulated robots should be created.

# Some other things to consider

* The concert solution and all subsequent concert clients need to be started with /use_sim_time set to true.
* Since all robots are being simulated on a single master, the tf tree needs tobe properly namespaced for each robot. Furthermore, applications run by this spawned concert clients need to use this namespaced tf tree. 

# An example

Take a look at the [segbot_gazebo_concert](https://github.com/utexas-bwi/segbot_rocon/tree/master/segbot_gazebo_concert) demo.

* Install all the UTexas code:
```
sudo apt-get install ros-hydro-segbot* ros-hydro-bwi*
```
* Install the UTexas Rocon code using this rosinstall entry:
```
- git: {local-name: segbot_rocon, uri: 'https://github.com/utexas-bwi/segbot_rocon.git'}
```
* Run the demo:
```
rocon_launch segbot_gazebo_concert gazebo.concert --screen
```

The implementation for RobotManager for the Segbot can be found in the [concert_service_segbot_gazebo](https://github.com/utexas-bwi/segbot_rocon/tree/master/concert_service_segbot_gazebo) repository.
