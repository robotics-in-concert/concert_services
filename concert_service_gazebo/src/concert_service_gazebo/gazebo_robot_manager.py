#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/concert_services/license/LICENSE
#
##############################################################################
# About
##############################################################################

# Simple script to manage spawning and killing of simulated gazebo robots
# for a concert. This node can be requested to trigger a rocon_launch'ed style
# terminal which embeds a standard concert client for each gazebo robot. It
# then flips across the necessary gazebo simulated handles to that concert
# client

##############################################################################
# Imports
##############################################################################

import copy
import os
import tempfile

import gateway_msgs.msg as gateway_msgs
import gateway_msgs.srv as gateway_srvs
import rocon_launch
import rospy
import rocon_gateway_utils

##############################################################################
# Gazebo Robot Manager
##############################################################################


class GazeboRobotManager:
    '''
      This class contains all the robot-independent functionality to launch
      robots in gazebo, create concert clients for each robot, and flip
      necessary information to each concert client, so each robot can truly
      behave as a concert client.
    '''

    def __init__(self, robot_manager):
        """
        :param robot_manager RobotManager: Instantiation of abstract class
            RobotManager that contains all robot specific information.
        """
        self.robots = []
        self.robot_manager = robot_manager
        self.is_disabled = False
        self._processes = []
        self._temporary_files = []

        # Gateway
        gateway_namespace = rocon_gateway_utils.resolve_local_gateway()
        rospy.wait_for_service(gateway_namespace + '/flip')
        self._gateway_flip_service = rospy.ServiceProxy(gateway_namespace + '/flip', gateway_srvs.Remote)

        # Terminal type for spawning
        try:
            self._terminal = rocon_launch.create_terminal()
        except (rocon_launch.UnsupportedTerminal, rocon_python_comms.NotFoundException) as e:
            rospy.logwarn("Turtle Herder : cannot find a suitable terminal, falling back to spawning inside the current one [%s]" % str(e))
            self._terminal = rocon_launch.create_terminal(rocon_launch.terminals.active)

    def _spawn_simulated_robots(self, robots):
        """
        Names and locations of robots to be spawned, read from a parameter file.

        :param robots list of dicts[]: The parameter file.
        The parameter file should read into a list of dictionaries, where each
        dict contains a "name" string, and a "location" tuple. For example:
            [{'name': 'kobuki', 'location': [0.0, 0.0, 0.0]},
             {'name': 'guimul', 'location': [0.0, 2.0, 0.0]}]
        For a full definition of the location vector, see
        RobotManager.spawn_robot().
        """
        for robot in robots:
            try:
                self.robot_manager.spawn_robot(robot["name"], robot["location"])
                self.robots.append(robot["name"])
            # TODO add failure exception
            except rospy.ROSInterruptException:
                rospy.loginfo("GazeboRobotManager : shutdown while spawning robot")
                continue

    def _launch_robot_clients(self, robot_names):
        """
        Spawn concert clients for given named robot.

        :param robot_names str[]: Names of all robots.
        """
        # spawn the concert clients
        rocon_launch_text = self.robot_manager.prepare_rocon_launch_text(robot_names)
        rospy.loginfo("GazeboRobotManager : constructing robot client rocon launcher")
        #print("\n" + console.green + rocon_launch_text + console.reset)
        temp = tempfile.NamedTemporaryFile(mode='w+t', delete=False)
        temp.write(rocon_launch_text)
        temp.close()
        launch_configurations = rocon_launch.parse_rocon_launcher(temp.name, "--screen")
        try:
            os.unlink(temp.name)
        except OSError:
            rospy.logerr("GazeboRobotManager : failed to unlink the rocon launcher.")

        for launch_configuration in launch_configurations:
            rospy.loginfo("GazeboRobotManager : launching concert client %s on port %s" %
                      (launch_configuration.name, launch_configuration.port))
            #print("%s" % launch_configuration)
            process, meta_roslauncher = self._terminal.spawn_roslaunch_window(launch_configuration)
            self._processes.append(process)
            self._temporary_files.append(meta_roslauncher)

    def _establish_unique_names(self, robots):
        """
        Make sure robot names don't clash with currently spawned robots, or
        with other robots in the same list itself. If they do, postfix them
        with an incrementing counter.

        :param robots list of dicts[]: The parameter file defining robots and
            start locations. For a full description, see
            _spawn_simulated_robots().
        :return str[]: uniquified names for the concert clients.
        """
        unique_robots = []
        unique_robot_names = []
        for robot in robots:
            robot_name = robot["name"]
            name_extension = ''
            count = 0
            while (robot_name + name_extension in unique_robot_names or
                   robot_name + name_extension in self.robots):
                name_extension = str(count)
                count = count + 1
            unique_robot_names.append(robot_name + name_extension)
            robot_copy = copy.deepcopy(robot)
            robot_copy["name"] = robot_name + name_extension
            unique_robots.append(robot_copy)
        return unique_robots, unique_robot_names

    def _send_flip_rules(self, robot_names, cancel):
        """
        Flip rules from Gazebo to the robot's concert client.

        :param robot_names str[]: Names of robots to whom information needs to
            be flipped.
        :param cancel bool: Cancel existing flips. Used during shutdown.
        """
        for robot_name in robot_names:
            rules = self.robot_manager.get_flip_rule_list(robot_name)
            # send the request
            request = gateway_srvs.RemoteRequest()
            request.cancel = cancel
            remote_rule = gateway_msgs.RemoteRule()
            remote_rule.gateway = robot_name
            for rule in rules:
                remote_rule.rule = rule
                request.remotes.append(copy.deepcopy(remote_rule))
            try:
                self._gateway_flip_service(request)
            except rospy.ServiceException:  # communication failed
                rospy.logerr("GazeboRobotManager : failed to send flip rules")
                return
            except rospy.ROSInterruptException:
                rospy.loginfo("GazeboRobotManager : shutdown while contacting the gateway flip service")
                return

    def spawn_robots(self, robots):
        """
        Ensure all robots have existing names, spawn robots in gazebo, launch
        concert clients, and flip necessary information from gazebo to each
        concert client.

        :param robots list of dicts[]: The parameter file defining robots and
            start locations. For a full description, see
            _spawn_simulated_robots().
        """
        unique_robots, unique_robot_names = self._establish_unique_names(robots)
        self._spawn_simulated_robots(unique_robots)
        self._launch_robot_clients(unique_robot_names)
        self._send_flip_rules(unique_robot_names, cancel=False)

    def shutdown(self):
        """
          - Send unflip requests.
          - Cleanup robots in gazebo.
          - Shutdown spawned terminals.
        """
        for name in self.robots:
            try:
                self.robot_manager.delete_robot(name)
                #TODO quitely fail exception here
            except rospy.ROSInterruptException:
                break  # quietly fail

        self._terminal.shutdown_roslaunch_windows(processes=self._processes,
                                                  hold=False)
        for temporary_file in self._temporary_files:
            #print("Unlinking %s" % temporary_file.name)
            try:
                os.unlink(temporary_file.name)
            except OSError as e:
                rospy.logerr("GazeboRobotManager : failed to unlink temporary file [%s]" % str(e))
