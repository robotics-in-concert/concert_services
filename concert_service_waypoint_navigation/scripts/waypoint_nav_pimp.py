#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/concert_services/license/LICENSE
#
##############################################################################
# About
##############################################################################

# Simple script to pimp out make a map operations for rocon interactions.
#
# - watch the app manager status and when it has a remote controller,
# - flip a spawn/kill pair across
# - call the spawn api
#  - the turtle herder will flip back some handles then.

##############################################################################
# Imports
##############################################################################

import rospy
import rocon_uri
import concert_service_utilities
import unique_id
import rocon_std_msgs.msg as rocon_std_msgs
import scheduler_msgs.msg as scheduler_msgs
import concert_service_msgs.msg as concert_service_msgs

from concert_software_farmer import SoftwareFarmClient, FailedToStartSoftwareException

class WaypointNavPimp(concert_service_utilities.ResourcePimp):

    def setup_variables(self):
        '''
            Need to setup the following variables
            service_priority, service_id, resource_type, available_resource_publisher_name, capture_topic_name
        '''
        (service_name, service_description, service_priority, service_id) = concert_service_utilities.get_service_info()       
        self.service_priority = service_priority
        self.service_id = service_id
        self.resource_type = 'rocon_apps/waypoint_nav'
        self.available_resource_publisher_name = 'available_waypoint_nav'
        self.capture_topic_name = 'capture_waypoint_nav'

        # it should be rapp's public interface
        self._default_map_topic = 'map' 
        self._default_waypoints_topic = 'waypoints'

    def ros_capture_callback(self, request_id, msg):
        '''
         Processes the service pair server 'capture_waypiont_nav'. This will run
         in a thread of its own for each request. It has a significantly long lock
         though - this needs to get fixed.
        '''
        # Todo : request the scheduler for this resource,
        # use self.allocation_timeout to fail gracefully
        response = concert_service_msgs.CaptureResourceResponse()
        response.result = False
        if not msg.release:  # i.e. we're capturing:
            if msg.rocon_uri not in [r.uri for r in self.available_resources]:
                self.logwarn("couldn't capture resource [not available][%s]" % msg.rocon_uri)
                response.result = False
            else:
                resource = self._create_resource(msg.rocon_uri)
                request_result, resource_request_id = self.send_allocation_request(resource)
                response.result = request_result
                if request_result == False:
                    self.logwarn("couldn't capture resource [timed out][%s]" % msg.rocon_uri)
                else:
                    self.loginfo("captured resource [%s][%s]" % (msg.rocon_uri, resource_request_id))
                    response.remappings = resource.remappings
        else:  # we're releasing
            self.send_releasing_request(msg.rocon_uri)
            response.result = True
        return response

    def _create_resource(self, uri):
        # Create a resource to request
        resource = scheduler_msgs.Resource()
        resource.id = unique_id.toMsg(unique_id.fromRandom())
        resource.rapp = self.resource_type
        resource.uri = uri
        resource.remappings = []
        map_remapped, waypoints_remapped  = self._get_remapped_topic(rocon_uri.parse(resource.uri).name.string)
        resource.remappings = [rocon_std_msgs.Remapping(self._default_map_topic, map_remapped), rocon_std_msgs.Remapping(self._default_waypoints_topic, waypoints_remapped)]
        return resource

    def _get_remapped_topic(self, name):
        '''
          Sets up remapping rules for Rapp configuration
        '''
        map_remapped = rospy.resolve_name(rospy.get_param('map_topic'))
        waypoints_remapped = rospy.resolve_name(rospy.get_param('waypoints_topic'))

        return map_remapped, waypoints_remapped

    def loginfo(self, msg):
        rospy.loginfo("WaypointNavPimp : %s"%str(msg))

    def logwarn(self, msg):
        rospy.logwarn("WaypointNavPimp : %s"%str(msg))

    def logerr(self, msg):
        rospy.logerr("WaypointNavPimp : %s"%str(msg))

##############################################################################
# Launch point
##############################################################################

if __name__ == '__main__':
    rospy.init_node('waypoint_nav_pimp')

    pimp = WaypointNavPimp()
    rospy.on_shutdown(pimp.cancel_all_requests)
    pimp.loginfo("Initialised")
    rospy.spin()
    pimp.loginfo("Bye Bye")
