#!/usr/bin/env python
import rospy
from concert_software_farmer import SoftwareFarmClient, FailedToStartSoftwareException

import world_canvas_client

def publish_map(world, namespace):
    map_name = rospy.get_param('map_name')
    map_topic = rospy.get_param('map_topic')
    rospy.loginfo("Publish World: Map - %s"%map_name)

    # Get the 2D map for the given world
    map_ac = world_canvas_client.AnnotationCollection(world=world, types=['nav_msgs/OccupancyGrid'], srv_namespace=namespace, names=[map_name])
    map_ac.load_data()

    # Publish the map on server side; topic type is get from the annotation info
    map_ac.publish(map_topic, None, False, False)   # i.e. by_server=True, as_list=False
    
    return map_ac

def publish_location(world, namespace):
    waypoints_topic = rospy.get_param('waypoints_topic')
    waypoints_viz_topic = rospy.get_param('waypoints_viz_topic')
    rospy.loginfo("Publish World: Waypoints   - %s"%waypoints_topic)
    
    waypoints_ac = world_canvas_client.AnnotationCollection(world=world, types=['yocs_msgs/Waypoint'], srv_namespace=namespace)
    waypoints_ac.load_data()
    waypoints_ac.publish(waypoints_topic, 'yocs_msgs/WaypointList', by_server=False, as_list=True)
    waypoints_ac.publish_markers(waypoints_viz_topic)
    return waypoints_ac

if __name__ == '__main__':
    rospy.init_node('world_canvas_client')

    try:
        sfc = SoftwareFarmClient()
        success, namespace, _unused_parameters = sfc.allocate("concert_software_common/world_canvas_server")

        if not success:
            raise FailedToStartSoftwareException("Failed to allocate software")

        rospy.loginfo("Publish World : world canvas namespace : %s"%namespace)
        world = rospy.get_param('world')
        rospy.loginfo("Publish World: %s"%world)
        map_ac = publish_map(world, namespace)
        waypoint_ac = publish_location(world, namespace)
        rospy.loginfo("Done")
        rospy.spin()

    except (FailedToStartSoftwareException) as e:
        rospy.logerr("Publish World : %s"%str(e))
