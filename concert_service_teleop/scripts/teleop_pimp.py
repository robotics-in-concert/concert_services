#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/concert_services/license/LICENSE
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

import sys
import threading
import time

import rospy
import rocon_python_comms
import concert_service_utilities
import concert_scheduler_requests
import unique_id
import rocon_std_msgs.msg as rocon_std_msgs
import scheduler_msgs.msg as scheduler_msgs
import concert_service_msgs.msg as concert_service_msgs

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
        'service_priority',
        'service_id',
        'scheduler_resources_subscriber',
        'list_available_teleops_server',
        'available_teleops_publisher',
        'allocation_timeout',
        'teleopable_robots',
        'requester',
        'lock',
        'pending_requests',   # a list of request id's pending feedback from the scheduler
        'allocated_requests'  # dic of { rocon uri : request id } for captured teleop robots.
    ]

    def __init__(self):
        ####################
        # Discovery
        ####################
        (self.service_name, self.service_description, self.service_priority, self.service_id) = concert_service_utilities.get_service_info()
        try:
            known_resources_topic_name = rocon_python_comms.find_topic('scheduler_msgs/KnownResources', timeout=rospy.rostime.Duration(5.0), unique=True)
        except rocon_python_comms.NotFoundException as e:
            rospy.logerr("TeleopPimp : could not locate the scheduler's known resources topic [%s]" % str(e))
            sys.exit(1)

        ####################
        # Setup
        ####################
        self.lock = threading.Lock()
        self.concert_clients_subscriber = rospy.Subscriber(known_resources_topic_name, scheduler_msgs.KnownResources, self.ros_scheduler_known_resources_callback)
        self.available_teleops_publisher = rospy.Publisher('available_teleops', rocon_std_msgs.StringArray, latch=True, queue_size=5)
        self.teleopable_robots = []
        self.requester = self.setup_requester(self.service_id)
        self.pending_requests = []
        self.allocated_requests = {}

        self.allocate_teleop_service_pair_server = rocon_python_comms.ServicePairServer('capture_teleop', self.ros_capture_teleop_callback, concert_service_msgs.CaptureTeleopPair, use_threads=True)
        self.allocation_timeout = rospy.get_param('allocation_timeout', 15.0)  # seconds

    def setup_requester(self, uuid):
        try:
            scheduler_requests_topic_name = concert_service_utilities.find_scheduler_requests_topic()
            #rospy.loginfo("Service : found scheduler [%s][%s]" % (topic_name))
        except rocon_python_comms.NotFoundException as e:
            rospy.logerr("TeleopPimp : %s" % (str(e)))
            return  # raise an exception here?
        frequency = concert_scheduler_requests.common.HEARTBEAT_HZ
        return concert_scheduler_requests.Requester(self.requester_feedback, uuid, 0, scheduler_requests_topic_name, frequency)

    def ros_scheduler_known_resources_callback(self, msg):
        '''
          For teleop interaction development, identify and store changes to the
          teleopable robots list - we get this list via the resource_pool topic
          provided by the scheduler for introspection.

          :param msg: incoming message
          :type msg: scheduler_msgs.KnownResources
        '''
        # find difference of incoming and stored lists based on unique concert names
        diff = lambda l1, l2: [x for x in l1 if x.uri not in [l.uri for l in l2]]
        # get all currently invited teleopable robots
        available_resources = [r for r in msg.resources if 'rocon_apps/video_teleop' in r.rapps and r.status == scheduler_msgs.CurrentStatus.AVAILABLE]
        preemptible_resources = [r for r in msg.resources if 'rocon_apps/video_teleop' in r.rapps and r.status == scheduler_msgs.CurrentStatus.ALLOCATED and r.priority < self.service_priority]
        resources = available_resources + preemptible_resources
        self.lock.acquire()
        new_resources = diff(resources, self.teleopable_robots)
        lost_resources = diff(self.teleopable_robots, resources)
        for resource in new_resources:
            self.teleopable_robots.append(resource)
        for resource in lost_resources:
            # rebuild list in place without lost client
            self.teleopable_robots[:] = [r for r in self.teleopable_robots if resource.uri != r.uri]
        self.lock.release()
        self.publish_available_teleops()

    def publish_available_teleops(self):
        self.lock.acquire()
        #rospy.logwarn("Publishing: %s" % [r.status for r in self.teleopable_robots])
        msg = rocon_std_msgs.StringArray()
        msg.strings = [r.uri for r in self.teleopable_robots if r.status != scheduler_msgs.CurrentStatus.ALLOCATED]
        self.available_teleops_publisher.publish(msg)
        self.lock.release()

    def ros_capture_teleop_callback(self, request_id, msg):
        '''
         Processes the service pair server 'capture_teleop'. This will run
         in a thread of its own for each request. It has a significantly long lock
         though - this needs to get fixed.
        '''
        response = concert_service_msgs.CaptureTeleopResponse()
        response.result = False
        # Todo : request the scheduler for this resource,
        # use self.allocation_timeout to fail gracefully
        self.lock.acquire()
        if not msg.release:  # i.e. we're capturing:
            if msg.rocon_uri not in [r.uri for r in self.teleopable_robots]:
                rospy.logwarn("TeleopPimp : couldn't capture teleopable robot [not available][%s]" % msg.rocon_uri)
                response.result = False
                self.allocate_teleop_service_pair_server.reply(request_id, response)
            else:
                # send a request
                resource = scheduler_msgs.Resource()
                resource.id = unique_id.toMsg(unique_id.fromRandom())
                resource.rapp = 'rocon_apps/video_teleop'
                resource.uri = msg.rocon_uri
                resource_request_id = self.requester.new_request([resource], priority=self.service_priority)
                #rospy.logwarn("DJS : resource request id of new request [%s]" % resource_request_id)
                self.pending_requests.append(resource_request_id)
                self.requester.send_requests()
                timeout_time = time.time() + self.allocation_timeout
                while not rospy.is_shutdown() and time.time() < timeout_time:
                    if resource_request_id not in self.pending_requests:
                        self.allocated_requests[msg.rocon_uri] = resource_request_id
                        response.result = True
                        break
                    rospy.rostime.wallsleep(0.1)
                if response.result == False:
                    rospy.logwarn("TeleopPimp : couldn't capture teleopable robot [timed out][%s]" % msg.rocon_uri)
                    self.requester.rset[resource_request_id].cancel()
                else:
                    rospy.loginfo("TeleopPimp : captured teleopable robot [%s][%s]" % (msg.rocon_uri, self.allocated_requests[msg.rocon_uri]))
                self.allocate_teleop_service_pair_server.reply(request_id, response)
        else:  # we're releasing
            if msg.rocon_uri in self.allocated_requests.keys():
                rospy.loginfo("TeleopPimp : released teleopable robot [%s][%s]" % (msg.rocon_uri, self.allocated_requests[msg.rocon_uri].hex))
                self.requester.rset[self.allocated_requests[msg.rocon_uri]].cancel()
                self.requester.send_requests()
                del self.allocated_requests[msg.rocon_uri]
            response.result = True
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
            #rospy.logwarn("DJS : request %s has status [%s]" % (request_id, request.msg.status))
            if request.msg.status == scheduler_msgs.Request.GRANTED:
                if request_id in self.pending_requests:
                    self.pending_requests.remove(request_id)
            elif request.msg.status == scheduler_msgs.Request.CLOSED:
                self.pending_requests.remove(request_id)
                self.granted_requests.remove(request_id)

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
