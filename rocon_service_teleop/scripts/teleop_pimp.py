#!/usr/bin/env python
#       
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_services/license/LICENSE
#
##############################################################################
# About
##############################################################################

# Simple script to pimp out teleop operations for rocon interactions.. 
#
# - watch the app manager status and when it has a remote controller,
# - flip a spawn/kill pair across
# - call the spawn api
#  - the turtle herder will flip back some handles then.

##############################################################################
# Imports
##############################################################################

#import copy
import rospy
import rocon_python_comms
import rocon_tutorial_msgs.srv as rocon_tutorial_srvs
import rocon_std_msgs.msg as rocon_std_msgs

##############################################################################
# Classes
##############################################################################

class Pimp:
    '''
      Sits in the concert client and makes the connections to the turtlesim engine.
    '''
    __slots__ = [
        'name',
        'list_available_resources_client',
        'list_available_teleops_server',
    ]

    def __init__(self):
        rospy.wait_for_service('~/list_available_resources')
        self.list_available_resources_client = rospy.ServiceProxy('~/get_available_resources', rocon_tutorial_srvs.GetAvailableResources, persistent=True)
        self.list_available_teleops_server = rocon_python_comms.ServicePairServer('list_available_teleops', self.list_teleops_service, rocon_std_msgs.StringsPair, use_threads=True)

    def list_teleops_services(self, request_id, msg):
        '''
          @param request_id
          @type uuid_msgs/UniqueID
          @param msg
          @type StringsRequest
        '''
        response = rocon_std_msgs.StringsResponse()
        relayed_request = turtlesim_srvs.KillRequest(msg.name)
        try:
            internal_service_response = self._kill_turtle_service_client(internal_service_request)
            self.turtles.remove(msg.name)
        except rospy.ServiceException:  # communication failed
            rospy.logerr("Spawn Turtles : failed to contact the internal kill turtle service")
        except rospy.ROSInterruptException:
            rospy.loginfo("Spawn Turtles : shutdown while contacting the internal kill turtle service")
            return
        self._kill_turtle_service_pair_server.reply(request_id, response)
        self._send_flip_rules_request(name=msg.name, cancel=True)

    def _ros_subscriber_remote_controller(self, msg):
        '''
          Callback for the app manager's latched remote controller publisher that informs us of it's
          changes in state.
        '''
        # only using thread-safe list operations: http://effbot.org/pyfaq/what-kinds-of-global-value-mutation-are-thread-safe.htm
        self.remote_controller_updates.append(msg.data)
        self.event_remote_controller_changed.set()

    def spin(self):
        '''
          Loop around checking if there's work to be done registering our hatchling on the turtlesim engine.
        '''
        interacting_with_remote_controller = None
        while not rospy.is_shutdown():
            event_is_set = self.event_remote_controller_changed.wait(0.5)
            if event_is_set:  # clear it so that it blocks again at the next run.
                self.event_remote_controller_changed.clear()
            if interacting_with_remote_controller is None: # check if we have a remote controller state change and send flips
                # Just send off one set of flips at a time, have to make sure we process kill/spawn
                if len(self.remote_controller_updates) > 0:
                    try:
                        interacting_with_remote_controller = self.remote_controller_updates.pop(0)
                        if interacting_with_remote_controller == self.remote_controller:
                            interacting_with_remote_controller = None  # do nothing
                        else:
                            self._send_flip_rules(interacting_with_remote_controller)
                    except IndexError:
                        rospy.logerr("Hatchling: index error")
            else:  # see if we can register with turtlesim or not yet.
                if interacting_with_remote_controller == rocon_app_manager_msgs.Constants.NO_REMOTE_CONTROLLER:
                    response = self.kill_turtle(rocon_tutorial_msgs.KillTurtleRequest(self.name), timeout=rospy.Duration(4.0))
                else:
                    response = self.spawn_turtle(rocon_tutorial_msgs.SpawnTurtleRequest(self.name), timeout=rospy.Duration(4.0))
                if response:  # didn't time out, probably waiting for the flips to arrive.
                    self.remote_controller = interacting_with_remote_controller
                    interacting_with_remote_controller = None
            

##############################################################################
# Launch point
##############################################################################

if __name__ == '__main__':
    
    rospy.init_node('hatchling')
    hatchling = Hatchling()
    hatchling.spin()
