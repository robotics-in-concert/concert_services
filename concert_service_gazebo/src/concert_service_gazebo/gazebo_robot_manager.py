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
import signal
import tempfile
import subprocess

import gateway_msgs.msg as gateway_msgs
import gateway_msgs.srv as gateway_srvs
import rocon_launch
import rospy
import rocon_gateway_utils
import rocon_python_utils.ros

##############################################################################
# Utilities
##############################################################################


class ProcessInfo(object):
    def __init__(self, process, temp_file):
        self.process = process
        self.temp_file = temp_file

##############################################################################
# Turtle Herder
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
        self._process_info = []
        self.is_disabled = False

        # Gateway
        gateway_namespace = rocon_gateway_utils.resolve_local_gateway()
        rospy.wait_for_service(gateway_namespace + '/flip')
        self._gateway_flip_service = rospy.ServiceProxy(gateway_namespace + '/flip', gateway_srvs.Remote)

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
        temp = tempfile.NamedTemporaryFile(mode='w+t', delete=False)
        rocon_launch_text = self.robot_manager.prepare_rocon_launch_text(robot_names)
        rospy.loginfo("GazeboRobotManager: constructing robot client rocon launcher")
        #print("\n" + console.green + rocon_launch_text + console.reset)
        temp.write(rocon_launch_text)
        temp.close()  # unlink it later
        rocon_launch_env = os.environ.copy()
        try:
            # ROS_NAMESPACE gets set since we are inside a node here
            # got to get rid of this otherwise it pushes things down
            del rocon_launch_env['ROS_NAMESPACE']
        except KeyError:
            pass
        rospy.loginfo("GazeboRobotManager : starting process %s" % ['rocon_launch', temp.name, '--screen'])
        if rocon_python_utils.ros.get_rosdistro() == 'hydro':
            process = subprocess.Popen(['rocon_launch', '--gnome', temp.name, '--screen'], env=rocon_launch_env)
        else:
            process = subprocess.Popen(['rocon_launch', temp.name, '--screen'], env=rocon_launch_env)
        self._process_info.append(ProcessInfo(process, temp))

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

        for process_info in self._process_info:
            print("Pid: %s" % process_info.process.pid)
            roslaunch_pids = rocon_launch.get_roslaunch_pids(process_info.process.pid)
            print("Roslaunch Pids: %s" % roslaunch_pids)
        # The os.kills aren't required if a concert is doing a full shutdown since
        # it disperses signals everywhere anyway. This is important if we implement the
        # disable from the service manager though.
        for pid in roslaunch_pids:
            try:
                os.kill(pid, signal.SIGINT)  # sighup or sigint? I can't remember - this is same as rocon_launch code
            except OSError:
                continue
        for pid in roslaunch_pids:
            print("Waiting on roslaunch pid %s" % pid)
            result = rocon_python_utils.system.wait_pid(pid)
            print("Pid %s exited with result %s" % (pid, result))
#         time.sleep(1)  # Do we need this?
        for process_info in self._process_info:
            print("Now killing konsoles %s" % process_info.process.pid)
            try:
                os.killpg(process_info.process.pid, signal.SIGTERM)
                #process_info.process.terminate()
            except OSError:
                print("process already died naturally")
                pass
        for process_info in self._process_info:
            print("Unlinking %s" % process_info.temp_file.name)
            try:
                os.unlink(process_info.temp_file.name)
            except OSError:
                pass

