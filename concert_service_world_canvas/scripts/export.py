#!/usr/bin/env python
'''
    Concert version of yaml_export.
    Uses rocon_python_comms.find_serice to get world_canvas handle 
'''

import rospy
import world_canvas_msgs.srv
import rocon_python_comms

if __name__ == '__main__':
    rospy.init_node('import')

    file = rospy.get_param('~file')

    try:
        known_yaml_export= rocon_python_comms.find_service('world_canvas_msgs/YAMLExport', timeout=rospy.rostime.Duration(5.0), unique=True)
    except rocon_python_comms.NotFoundException as e:
        self.logerr("could not locate the scheduler's known resources topic [%s]" % str(e))
        sys.exit(1)

    rospy.loginfo("Import annotations from file '%s'", file)
    export_srv = rospy.ServiceProxy(known_yaml_export, world_canvas_msgs.srv.YAMLExport)
    response = export_srv(file)

    if response.result == True:
        rospy.loginfo("Database successfully exported from file '%s'", file)
    else:
        rospy.logerr("Export database failed; %s", response.message)
