#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_services/license/LICENSE
#
##############################################################################
# About
##############################################################################

# Simple script to pimp out teleop operations for rocon interactions.
#
# - watch the app manager status and when it has a remote controller,
# - flip a spawn/kill pair across
# - call the spawn api
#  - the turtle herder will flip back some handles then.

##############################################################################
# Imports
##############################################################################

import threading

import rospy
import rocon_python_comms
import concert_msgs.msg as concert_msgs
import rocon_app_manager_msgs.msg as rapp_manager_msgs
import rocon_scheduler_requests
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
        'allocation_timeout',
        'teleopable_robots',
        'requester',
        'lock'
    ]

    def __init__(self):
        # could use find_topic here, but would need to intelligently sort the non-unique list that comes back
        self.concert_clients_subscriber = rospy.Subscriber(concert_msgs.Strings.CONCERT_CLIENTS, concert_msgs.ConcertClients, self.ros_concert_clients_callback)
        self.scheduler_resources_subscriber = rospy.Subscriber("scheduler_resources", scheduler_msgs.KnownResources, self.ros_scheduler_resources_callback)
        self.available_teleops_publisher = rospy.Publisher('available_teleops', concert_msgs.ConcertClients, latch=True)
        self.teleopable_robots = []
        self.requester = self.setup_requester()
        self.lock = threading.Lock()

        self.allocate_teleop_service_pair_server = rocon_python_comms.ServicePairServer('capture_teleop', self.ros_capture_teleop_callback, rocon_service_msgs.CaptureTeleopPair, use_threads=True)
        self.allocation_timeout = 5.0  # seconds

    def setup_requester(self):
        uuid = None
        topic = rocon_scheduler_requests.common.SCHEDULER_TOPIC
        frequency = rocon_scheduler_requests.common.HEARTBEAT_HZ
        return rocon_scheduler_requests.Requester(self.requester_feedback, uuid, 0, topic, frequency)

    def requester_feedback(self, request_set):
        '''
          This returns requests processed by the scheduler with whatever necessary modifications
          that were made on the original requests.

          @param request_set : the modified requests
          @type dic { uuid.UUID : scheduler_msgs.ResourceRequest }
        '''
        pass

    def ros_concert_clients_callback(self, msg):
        '''
          This is the hack right now till we can use ros_scheduler_resources_callback to introspect what is available.
          This will get called periodically. For teleop interaction development, identify and store changes to the
          teleopable robots list.

          @TODO this could be smarter. It publishes available teleops periodically with frequency of
          concert clients publishing instead of on state changes - noisy.
        '''
        # find difference of incoming and stored lists based on unique concert names
        diff = lambda l1, l2: [x for x in l1 if x.name not in [l.name for l in l2]]
        # get all currently invited teleopable robots
        clients = [client for client in msg.clients if 'turtle_concert/teleop' in [app.name for app in client.apps]]
        self.lock.acquire()
        new_clients = diff(clients, self.teleopable_robots)
        lost_clients = diff(self.teleopable_robots, clients)
        for client in new_clients:
            self.teleopable_robots.append(client)
        for client in lost_clients:
            # rebuild list in place without lost client
            self.teleopable_robots[:] = [c for c in self.teleopable_robots if client.name != c.name]
        self.lock.release()
        self.publish_available_teleops()

    def publish_available_teleops(self):
        self.lock.acquire()
        msg = concert_msgs.ConcertClients()
        msg.clients = [c for c in self.teleopable_robots if c.status != rapp_manager_msgs.Constants.APP_RUNNING]
        self.available_teleops_publisher.publish(msg)
        self.lock.release()

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
