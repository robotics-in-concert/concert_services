#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_tutorials/license/LICENSE
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

import os
import signal
import subprocess
import tempfile
import math
import random
import copy
import time

import rocon_launch
import rospy
import rocon_gateway_utils
import rocon_python_utils.ros
import turtlesim.srv as turtlesim_srvs
import gateway_msgs.msg as gateway_msgs
import gateway_msgs.srv as gateway_srvs

##############################################################################
# Utilities
##############################################################################


def prepare_rocon_launch_text(turtles):
    port = 11
    launch_text = '<concert>\n'
    for name in turtles:
        launch_text += '  <launch title="%s:114%s" package="rocon_service_turtlesim" name="turtle.launch" port="114%s">\n' % (name, str(port), str(port))
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
        'turtles',  # Dictionary of string : concert_msgs.RemoconApp[]
        '_kill_turtle_service_client',
        '_spawn_turtle_service_client',
#        '_kill_turtle_service',
#        '_spawn_turtle_service',
        '_gateway_flip_service',
        '_process_info'
    ]

    def __init__(self):
        self.turtles = []
        self._process_info = []
        # herding backend
        rospy.wait_for_service('~internal/kill')  # could use timeouts here
        rospy.wait_for_service('~internal/spawn')
        self._kill_turtle_service_client = rospy.ServiceProxy('~internal/kill', turtlesim_srvs.Kill, persistent=True)
        self._spawn_turtle_service_client = rospy.ServiceProxy('~internal/spawn', turtlesim_srvs.Spawn, persistent=True)
        # kill the default turtle that turtlesim starts with
        try:
            unused_response = self._kill_turtle_service_client("turtle1")
        except rospy.ServiceException:
            rospy.logerr("Turtle Herder : failed to contact the internal kill turtle service")
        except rospy.ROSInterruptException:
            rospy.loginfo("Turtle Herder : shutdown while contacting the internal kill turtle service")
            return
        # herding frontend
#        self._kill_turtle_service = rospy.Service('~kill', rocon_service_srvs.KillTurtle, self._kill_turtle_service)
#        self._spawn_turtle_service = rospy.Service('~spawn', rocon_service_srvs.SpawnTurtle, self._spawn_turtle_service)
        # gateway
        gateway_namespace = rocon_gateway_utils.resolve_local_gateway()
        rospy.wait_for_service(gateway_namespace + '/flip')
        self._gateway_flip_service = rospy.ServiceProxy(gateway_namespace + '/flip', gateway_srvs.Remote)

#     def _kill_turtle_service(self, req):
#         '''
#           @param msg
#           @type rocon_service_srvs.KillTurtleRequest
#         '''
#         internal_service_request = turtlesim_srvs.KillRequest(msg.name)
#         try:
#             unused_internal_service_response = self._kill_turtle_service_client(internal_service_request)
#             self.turtles.remove(req.name)
#         except rospy.ServiceException:  # communication failed
#             rospy.logerr("Turtle Herder : failed to contact the internal kill turtle service")
#         except rospy.ROSInterruptException:
#             rospy.loginfo("Turtle Herder : shutdown while contacting the internal kill turtle service")
#             return
#         self._kill_turtle_service_pair_server.reply(rocon_service_srvs.KillTurtleResponse())
#         self._send_flip_rules_request(name=req.name, cancel=True)
# 
#     def _spawn_turtle_service(self, req):
#         '''
#           @param msg
#           @type rocon_service_srvs.SpawnTurtleRequest
#         '''
#         response = rocon_service_srvs.SpawnTurtleResponse()
#         response.name = ''
#         # Unique name
#         name = req.name
#         name_extension = ''
#         count = 0
#         while name + name_extension in self.turtles:
#             name_extension = '_' + str(count)
#             count = count + 1
#         name = name + name_extension
# 
#         internal_service_request = turtlesim_srvs.SpawnRequest(
#                                             random.uniform(3.5, 6.5),
#                                             random.uniform(3.5, 6.5),
#                                             random.uniform(0.0, 2.0 * math.pi),
#                                             name)
#         try:
#             unused_internal_service_response = self._spawn_turtle_service_client(internal_service_request)
#             self.turtles.append(name)
#         except rospy.ServiceException:  # communication failed
#             rospy.logerr("TurtleHerder : failed to contact the internal spawn turtle service")
#             return response
#         except rospy.ROSInterruptException:
#             rospy.loginfo("TurtleHerder : shutdown while contacting the internal spawn turtle service")
#             return response
#         self._send_flip_rules_request(name=name, cancel=False)
#         response.name = name
#         return response

    def spawn_turtles(self, turtles):
        # establish names
        for turtle_name in turtles:
            name_extension = ''
            count = 0
            while turtle_name + name_extension in self.turtles:
                name_extension = '_' + str(count)
                count = count + 1
            self.turtles.append(turtle_name + name_extension)

        # spawn the turtle concert clients
        temp = tempfile.NamedTemporaryFile(mode='w+t', delete=False)
        rocon_launch_text = prepare_rocon_launch_text(turtles)
        print("%s" % rocon_launch_text)
        temp.write(rocon_launch_text)
        temp.close()  # unlink it later
        rospy.loginfo("TurtleHerder : starting process %s" % ['rocon_launch', temp.name, '--screen'])
        if rocon_python_utils.ros.get_rosdistro() == 'hydro':
            process = subprocess.Popen(['rocon_launch', '--gnome', temp.name, '--screen'])
        else:
            process = subprocess.Popen(['rocon_launch', temp.name, '--screen'])
        self._process_info.append(ProcessInfo(process, temp))

        # create the turtles in turtlesim
        for name in self.turtles:
            internal_service_request = turtlesim_srvs.SpawnRequest(
                                                random.uniform(3.5, 6.5),
                                                random.uniform(3.5, 6.5),
                                                random.uniform(0.0, 2.0 * math.pi),
                                                name)
            try:
                unused_internal_service_response = self._spawn_turtle_service_client(internal_service_request)
                self.turtles.append(name)
            except rospy.ServiceException:  # communication failed
                rospy.logerr("TurtleHerder : failed to contact the internal spawn turtle service")
            except rospy.ROSInterruptException:
                rospy.loginfo("TurtleHerder : shutdown while contacting the internal spawn turtle service")

        # flip rules
        for name in self.turtles:
            self._send_flip_rules_request(name=name, cancel=False)

    def _send_flip_rules_request(self, name, cancel):
        rules = []
        rule = gateway_msgs.Rule()
        rule.node = ''
        rule.type = gateway_msgs.ConnectionType.SUBSCRIBER
        # could resolve this better by looking up the service info
        rule.name = "/services/turtlesim/%s/cmd_vel" % name
        rules.append(copy.deepcopy(rule))
        rule.type = gateway_msgs.ConnectionType.PUBLISHER
        rule.name = "/services/turtlesim/%s/pose" % name
        rules.append(copy.deepcopy(rule))
        # send the request
        request = gateway_srvs.RemoteRequest()
        request.cancel = cancel
        remote_rule = gateway_msgs.RemoteRule()
        remote_rule.gateway = name
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

    def shutdown(self):
        """
          - Send unflip requests  (this should go in a service manager callable ros callback)
          - Cleanup turtles on the turtlesim canvas.
          - Shutdown spawned terminals

          Cleaning turtles is probably not really important since
          we always shutdown turtlesim and turtle_herder together.
        """
        for name in self.turtles:
            try:
                unused_internal_service_response = self._kill_turtle_service_client(name)
            except rospy.ServiceException:  # communication failed
                break  # quietly fail
            except rospy.ROSInterruptException:
                break  # quietly fail

        for process_info in self._process_info:
            print("Pid: %s" % process_info.process.pid)
            roslaunch_pids = rocon_launch.get_roslaunch_pids(process_info.process.pid)
            print("Roslaunch Pids: %s" % roslaunch_pids)
        # don't need to tell the roslaunch processes to interrupt since we're
        # letting ros pass through that SIGINT everywhere. If we were to use a
        # signal_handler instead we probably would have to.
        # Nonetheless, still wait for the boys
        for pid in roslaunch_pids:
            print("Waiting on roslaunch pid %s" % pid)
            result = rocon_python_utils.system.wait_pid(process_info.process.pid)
            print("Pid %s exited with result %s" % (pid, result))
#         time.sleep(1)  # Do we need this?
        for process_info in self._process_info:
            print("Now killing konsoles")
            try:
                process_info.process.terminate()
            except OSError:
                # process had already died naturally
                pass
        for process_info in self._process_info:
            print("Unlinking %s" % process_info.temp_file.name)
            os.unlink(process_info.temp_file.name)

##############################################################################
# Launch point
##############################################################################

if __name__ == '__main__':

    rospy.init_node('turtle_herder')

    turtle_herder = TurtleHerder()
    turtle_herder.spawn_turtles(['kobuki', 'guimul'])
    rospy.spin()
    turtle_herder.shutdown()
