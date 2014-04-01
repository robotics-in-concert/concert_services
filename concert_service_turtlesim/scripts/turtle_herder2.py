#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/concert_services/license/LICENSE
#
##############################################################################
# About
##############################################################################

# Simple script to manage spawning and killing of turtles across multimaster
# boundaries. Typically turtlesim clients would connect to the kill and
# spawn services directly to instantiate themselves, but since we can't
# flip service proxies, this is not possible. So this node is the inbetween
# go-to node and uses a rocon service pair instead.
#
# It supplements this relay role with a bit of herd management - sets up
# random start locations and feeds back aliased names when running with
# a concert.

##############################################################################
# Imports
##############################################################################

import copy
import math
import os
import random
import signal
import tempfile
import subprocess

import concert_service_utilities
import gateway_msgs.msg as gateway_msgs
import gateway_msgs.srv as gateway_srvs
import rocon_launch
import rospy
import rocon_console.console as console
import rocon_gateway_utils
import rocon_python_utils.ros
import std_msgs.msg as std_msgs
import turtlesim.srv as turtlesim_srvs

##############################################################################
# Utilities
##############################################################################


def prepare_rocon_launch_text(turtles):
    port = 11
    launch_text = '<concert>\n'
    for name in turtles:
        launch_text += '  <launch title="%s:114%s" package="concert_service_turtlesim" name="turtle2.launch" port="114%s">\n' % (name, str(port), str(port))
        launch_text += '    <arg name="turtle_name" value="%s"/>\n' % name
        launch_text += '    <arg name="turtle_concert_whitelist" value="Turtle Concert;Turtle Teleop Concert;Concert Tutorial"/>\n'
        launch_text += '    <arg name="turtle_rapp_whitelist" value="[rocon_apps, turtle_concert]"/>\n'
        launch_text += '  </launch>\n'
        port = port + 1
    launch_text += '</concert>\n'
    return launch_text


class ProcessInfo(object):
    def __init__(self, process, temp_file):
        self.process = process
        self.temp_file = temp_file

##############################################################################
# Turtle Herder
##############################################################################


class TurtleHerder:
    '''
      Shepherds the turtles!

      @todo get alised names from the concert client list if the topic is available

      @todo watchdog for killing turtles that are no longer connected.
    '''
    __slots__ = [
        'turtles',  # list of turtle name strings
        '_kill_turtle_service_client',
        '_spawn_turtle_service_client',
        '_gateway_flip_service',
        '_process_info',
        'is_disabled'  # flag set when service manager tells it to shut down.
    ]

    def __init__(self):
        self.turtles = []
        self._process_info = []
        self.is_disabled = False
        # herding backend
        rospy.wait_for_service('kill')  # could use timeouts here
        rospy.wait_for_service('spawn')
        self._kill_turtle_service_client = rospy.ServiceProxy('kill', turtlesim_srvs.Kill, persistent=True)
        self._spawn_turtle_service_client = rospy.ServiceProxy('spawn', turtlesim_srvs.Spawn, persistent=True)
        # kill the default turtle that turtlesim starts with
        try:
            unused_response = self._kill_turtle_service_client("turtle1")
        except rospy.ServiceException:
            rospy.logerr("Turtle Herder : failed to contact the internal kill turtle service")
        except rospy.ROSInterruptException:
            rospy.loginfo("Turtle Herder : shutdown while contacting the internal kill turtle service")
            return
        self._shutdown_subscriber = rospy.Subscriber('shutdown', std_msgs.Empty, self.shutdown)
        # gateway
        gateway_namespace = rocon_gateway_utils.resolve_local_gateway()
        rospy.wait_for_service(gateway_namespace + '/flip')
        self._gateway_flip_service = rospy.ServiceProxy(gateway_namespace + '/flip', gateway_srvs.Remote)

    def _spawn_simulated_turtles(self, turtles):
        """
        Very important to have checked that the turtle names are unique
        before calling this method.

        :param turtles str[]: names of the turtles to spawn.
        """
        for turtle in turtles:
            internal_service_request = turtlesim_srvs.SpawnRequest(
                                                random.uniform(3.5, 6.5),
                                                random.uniform(3.5, 6.5),
                                                random.uniform(0.0, 2.0 * math.pi),
                                                turtle)
            try:
                unused_internal_service_response = self._spawn_turtle_service_client(internal_service_request)
                self.turtles.append(turtle)
            except rospy.ServiceException:  # communication failed
                rospy.logerr("TurtleHerder : failed to contact the internal spawn turtle service")
                continue
            except rospy.ROSInterruptException:
                rospy.loginfo("TurtleHerder : shutdown while contacting the internal spawn turtle service")
                continue

    def _launch_turtle_clients(self, turtles):
        # spawn the turtle concert clients
        temp = tempfile.NamedTemporaryFile(mode='w+t', delete=False)
        rocon_launch_text = prepare_rocon_launch_text(turtles)
        rospy.loginfo("TurtleHerder: constructing turtle client rocon launcher")
        print("\n" + console.green + rocon_launch_text + console.reset)
        temp.write(rocon_launch_text)
        temp.close()  # unlink it later
        rocon_launch_env = os.environ.copy()
        try:
            # ROS_NAMESPACE gets set since we are inside a node here
            # got to get rid of this otherwise it pushes things down
            del rocon_launch_env['ROS_NAMESPACE']
        except KeyError:
            pass
        rospy.loginfo("TurtleHerder : starting process %s" % ['rocon_launch', temp.name, '--screen'])
        if rocon_python_utils.ros.get_rosdistro() == 'hydro':
            process = subprocess.Popen(['rocon_launch', '--gnome', temp.name, '--screen'], env=rocon_launch_env)
        else:
            process = subprocess.Popen(['rocon_launch', temp.name, '--screen'], env=rocon_launch_env)
        self._process_info.append(ProcessInfo(process, temp))

    def spawn_turtles(self, turtles):
        unique_turtle_names = self._establish_unique_names(turtles)
        self.turtles.extend(unique_turtle_names)

        self._spawn_simulated_turtles(turtles)
        self._launch_turtle_clients(unique_turtle_names)
        self._send_flip_rules(unique_turtle_names, cancel=False)

    def _establish_unique_names(self, turtles):
        """
        Make sure the turtle names don't clash with currently spawned turtles.
        If they do, postfix them with an incrementing counter.

        :param turtles str[]: list of new turtle names to uniquify.
        :return str[]: uniquified names for the turtles.
        """
        unique_turtle_names = []
        for turtle_name in turtles:
            name_extension = ''
            count = 0
            while turtle_name + name_extension in self.turtles:
                name_extension = '_' + str(count)
                count = count + 1
            unique_turtle_names.append(turtle_name + name_extension)
        return unique_turtle_names

    def _send_flip_rules(self, turtles, cancel):
        for turtle in turtles:
            rules = []
            rule = gateway_msgs.Rule()
            rule.node = ''
            rule.type = gateway_msgs.ConnectionType.SUBSCRIBER
            # could resolve this better by looking up the service info
            rule.name = "/services/turtlesim/%s/cmd_vel" % turtle
            rules.append(copy.deepcopy(rule))
            rule.type = gateway_msgs.ConnectionType.PUBLISHER
            rule.name = "/services/turtlesim/%s/pose" % turtle
            rules.append(copy.deepcopy(rule))
            # send the request
            request = gateway_srvs.RemoteRequest()
            request.cancel = cancel
            remote_rule = gateway_msgs.RemoteRule()
            remote_rule.gateway = turtle
            for rule in rules:
                remote_rule.rule = rule
                request.remotes.append(copy.deepcopy(remote_rule))
            try:
                self._gateway_flip_service(request)
            except rospy.ServiceException:  # communication failed
                rospy.logerr("TurtleHerder : failed to send flip rules")
                return
            except rospy.ROSInterruptException:
                rospy.loginfo("TurtleHerder : shutdown while contacting the gateway flip service")
                return

    def _ros_service_manager_disable_callback(self, msg):
        self.is_disabled = True

    def shutdown(self):
        """
          - Send unflip requests
          - Cleanup turtles on the turtlesim canvas.
          - Shutdown spawned terminals

        :todo: this should go in a service manager callable ros callback where we can
        call disable on this service and bring it down without having to SIGINT it.
        """
        # cleaning turtles is probably not really important since
        # we always shutdown turtlesim and turtle_herder together.
        # for name in self.turtles:
        #     try:
        #         unused_internal_service_response = self._kill_turtle_service_client(name)
        #     except rospy.ServiceException:  # communication failed
        #         break  # quietly fail
        #     except rospy.ROSInterruptException:
        #         break  # quietly fail

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

##############################################################################
# Launch point
##############################################################################

if __name__ == '__main__':

    rospy.init_node('turtle_herder')
    (service_name, unused_service_description, unused_service_id) = concert_service_utilities.get_service_info()
    turtles = rospy.get_param('/services/' + service_name + '/turtles', [])
    rospy.logwarn("TurtleHerder: spawning turtles: %s" % turtles)

    turtle_herder = TurtleHerder()
    turtle_herder.spawn_turtles(turtles)
    while not rospy.is_shutdown() and not turtle_herder.is_disabled:
        rospy.sleep(0.3)
    turtle_herder.shutdown()
