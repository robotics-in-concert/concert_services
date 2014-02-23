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
import scheduler_msgs.msg as scheduler_msgs
import geometry_msgs.msg as geometry_msgs
import rocon_service_msgs.msg as rocon_service_msgs

##############################################################################
# Classes
##############################################################################

class TeleopPimp:
    '''
      Listens for requests to gain a teleop'able robot.
    '''
    __slots__ = [
        'name',
        'scheduler_resources_subscriber',
        'list_available_teleops_server',
        'available_teleops_publisher',
        'allocation_timeout'
    ]

    def __init__(self):
        self.scheduler_resources_subscriber = rospy.Subscriber("scheduler_resources", scheduler_msgs.KnownResources, self.ros_scheduler_resources_callback)
        self.available_teleops_publisher = rospy.Publisher('available_teleops', geometry_msgs.Twist, latch=True)

        self.allocate_teleop_service_pair_server = rocon_python_comms.ServicePairServer('capture_teleop', self.ros_capture_teleop_callback, rocon_service_msgs.CaptureTeleopPair, use_threads=True)
        self.allocation_timeout = 5.0  # seconds

    def ros_scheduler_resources_callback(self, msg):
        """
         Process the subscriber to the scheduler's latched publisher of known resources.
        """
        # Todo : relay to interaction
        # teleops_msg = 
        # self.available_teleops_publisher.publish(teleops_msg)
        pass
    
    def ros_capture_teleop_callback(self, request_id, msg):
        '''
         Processes the service pair server 'capture_teleop'. This will run
         in a thread of its own.
        '''
        response = rocon_service_msgs.CaptureTeleopResponse()
        # Todo : request the scheduler for this resource, 
        # use self.allocation_timeout to fail gracefully
        response.result = False
        self.allocate_teleop_service_pair_server.reply(request_id, response)
            

##############################################################################
# Launch point
##############################################################################

if __name__ == '__main__':
    
    rospy.init_node('teleop_pimp')
    pimp = TeleopPimp()
    rospy.spin()
