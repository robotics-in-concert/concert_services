#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/concert_services/license/LICENSE
#
##############################################################################
# About
##############################################################################

# Base class that defines the API of a robot-specific module needs to
# be defined. This robot specific module will be used by GazeboRobotManager to
# spawn and kill robots in gazebo.

import rospy
import rocon_python_utils
import gazebo_msgs.srv as gazebo_srvs
from gateway_msgs.msg import Rule, ConnectionType
from .utils import reformat_position_vector, generate_spawn_robot_launch_script, start_roslaunch_process 

class RobotManager(object):

    __slots__ = ['_robot_type', '_launch', '_world_namespace', '_processes', '_srv']
    
    def __init__(self, robot_type, world_namespace):
        self._robot_type = robot_type['name']
        self._launch = rocon_python_utils.ros.find_resource_from_string(robot_type['launch'])
        self._world_namespace = world_namespace
        self._processes = {}
        self._srv = {}

        self._setup_gazebo_api()

    def _setup_gazebo_api(self):
        delete_model_srv_name = self._world_namespace + '/delete_model'
        rospy.wait_for_service(delete_model_srv_name)
        
        self._srv['delete_model'] = rospy.ServiceProxy(delete_model_srv_name, gazebo_srvs.DeleteModel)

    def spawn_robot(self, name, position_vector, args=None):
        location = reformat_position_vector(position_vector)
        launch_script = generate_spawn_robot_launch_script(name, location, self._world_namespace, self._launch, args)

        self._processes[name] = start_roslaunch_process(launch_script)

    def delete_robot(self, name):
        delete_model_srv_req = gazebo_srvs.DeleteModelRequest(name)
        try:
            self.processes[name].terminate()
            self._srv['delete_model'](delete_model_srv_req)
        except rospy.ServiceExceptin: # Communication failed
            rospy.logerr('GazeboRobotManager : unable to delete model %s' % name)

    def get_flip_rule_list(self):
        return []
