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
import time

import rospy
import rocon_python_comms
import concert_msgs.msg as concert_msgs
import concert_service_utilities
import rocon_app_manager_msgs.msg as rapp_manager_msgs
import rocon_scheduler_requests
import unique_id
import rocon_std_msgs.msg as rocon_std_msgs
import scheduler_msgs.msg as scheduler_msgs
import rocon_service_msgs.msg as rocon_service_msgs

##############################################################################
# Classes
##############################################################################


class TeleopPimp:
    '''
      Listens for requests to gain a teleop'able robot.
    '''
    __slots__ = [
        'service_name',
        'service_description',
        'service_id',
        'scheduler_resources_subscriber',
        'list_available_teleops_server',
        'available_teleops_publisher',
        'allocation_timeout',
        'teleopable_robots',
        'requester',
        'lock',
        'pending_requests'  # a list of request id's pending feedback from the scheduler
    ]

    def __init__(self):
        (self.service_name, self.service_description, self.service_id) = concert_service_utilities.get_service_info()

        # could use find_topic here, but would need to intelligently sort the non-unique list that comes back
        self.concert_clients_subscriber = rospy.Subscriber(concert_msgs.Strings.CONCERT_CLIENTS, concert_msgs.ConcertClients, self.ros_concert_clients_callback)
        self.scheduler_resources_subscriber = rospy.Subscriber("scheduler_resources", scheduler_msgs.KnownResources, self.ros_scheduler_resources_callback)
        self.available_teleops_publisher = rospy.Publisher('available_teleops', rocon_std_msgs.StringArray, latch=True)
        self.teleopable_robots = []
        self.requester = self.setup_requester(self.service_id)
        self.lock = threading.Lock()
        self.pending_requests = []

        self.allocate_teleop_service_pair_server = rocon_python_comms.ServicePairServer('capture_teleop', self.ros_capture_teleop_callback, rocon_service_msgs.CaptureTeleopPair, use_threads=True)
        self.allocation_timeout = 5.0  # seconds

    def setup_requester(self, uuid):
        topic = concert_msgs.Strings.SCHEDULER_REQUESTS
        frequency = rocon_scheduler_requests.common.HEARTBEAT_HZ
        return rocon_scheduler_requests.Requester(self.requester_feedback, uuid, 0, topic, frequency)

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
        msg = rocon_std_msgs.StringArray()
        msg.strings = [c.platform_info.uri for c in self.teleopable_robots if c.status != rapp_manager_msgs.Constants.APP_RUNNING]
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
         in a thread of its own for each request.
        '''
        response = rocon_service_msgs.CaptureTeleopResponse()
        response.result = False
        # Todo : request the scheduler for this resource,
        # use self.allocation_timeout to fail gracefully
        self.lock.acquire()
        if msg.rocon_uri not in [c.platform_info.uri for c in self.teleopable_robots]:
            response.result = False
            self.allocate_teleop_service_pair_server.reply(request_id, response)
        else:
            # send a request
            resource = scheduler_msgs.Resource()
            resource.id = unique_id.toMsg(unique_id.fromRandom())
            resource.rapp = 'turtle_concert/teleop'
            resource.uri = msg.rocon_uri
            resource_request_id = self.requester.new_request([resource])
            self.pending_requests.append(resource_request_id)
            self.requester.send_requests()
            timeout_time = time.time() + 5.0
            while not rospy.is_shutdown() and time.time() < timeout_time:
                if resource_request_id not in self.pending_requests:
                    response.result = True
                    break
                rospy.rostime.wallsleep(0.1)
            if response.result == False:
                self.requester.rset[resource_request_id].cancel()
            self.allocate_teleop_service_pair_server.reply(request_id, response)
        self.lock.release()

    def requester_feedback(self, request_set):
        '''
          Keep an eye on our pending requests and see if they get allocated here.
          Once they do, kick them out of the pending requests list so _ros_capture_teleop_callback
          can process and reply to the interaction.

          @param request_set : the modified requests
          @type dic { uuid.UUID : scheduler_msgs.ResourceRequest }
        '''
        for request_id, request in request_set.requests.iteritems():
            if request_id in self.pending_requests:
                if request.msg.status == scheduler_msgs.Request.GRANTED:
                    self.pending_requests.remove(request_id)

    def cancel_all_requests(self):
        '''
          Exactly as it says! Used typically when shutting down or when
          it's lost more allocated resources than the minimum required (in which case it
          cancels everything and starts reissuing new requests).
        '''
        #self.lock.acquire()
        self.requester.cancel_all()
        self.requester.send_requests()
        #self.lock.release()

##############################################################################
# Launch point
##############################################################################

if __name__ == '__main__':

    rospy.init_node('teleop_pimp')
    pimp = TeleopPimp()
    rospy.spin()
    if not rospy.is_shutdown():
        pimp.cancel_all_requests()
