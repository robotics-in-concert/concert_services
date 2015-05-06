#!/usr/bin/env python
"""
    Command Navigator
"""
import argparse
import sys

import actionlib
import rospy
import yocs_msgs.msg as yocs_msgs

def parse_arguments():
    parser = argparse.ArgumentParser(description="Robot Commander.")
    parser.add_argument('-r', '--robot', type=str, help='Robot name')
    parser.add_argument('-t', '--target', type=str, help='Target Location')

    args = parser.parse_args(sys.argv[1:])
    return args.robot, args.target

class NavigateCommander(object):

    ACTION_NAVIGATOR = 'navigator'
    
    def __init__(self, robot_name):
        '''
            Initilise NavigatorCommander with given robot name
        '''
        self._robot_name = robot_name

        ac_name = '/%s/%s'%(robot_name, NavigateCommander.ACTION_NAVIGATOR)
        self._ac_navigator = actionlib.SimpleActionClient(ac_name, yocs_msgs.NavigateToAction)
        self.loginfo('Setting navigator of %s'%robot_name)
        rospy.on_shutdown(self.shutdown)

        if not self._ac_navigator.wait_for_server(rospy.Duration(3.0)):
            self.logerr('Navigator action server not found. Please check if you have captured %s'%robot_name)
            self._ready = False
        else:
            self._ready = True

    def command(self, target):
        '''
            Send navigate to action
        '''
        if not self._ready:
            self.logerr('Navigator is not ready exiting..')
            return

        goal = yocs_msgs.NavigateToGoal(location=target, approach_type=yocs_msgs.NavigateToGoal.APPROACH_ON, num_retry=1, timeout=300.0)
        self._done = False
        self._ac_navigator.send_goal(goal, feedback_cb=self._navigator_feedback)
        self._ac_navigator.wait_for_result()
        result = self._ac_navigator.get_result()
        self.loginfo("%s, Message : %s"%(result.success,result.message))

    def _navigator_feedback(self, feedback):
        navigator_feed = str("Distance : %s, Remain Time : %s, Message : %s"%(str(feedback.distance),str(feedback.remain_time),str(feedback.message)))
        self.loginfo(navigator_feed)

    def shutdown(self):
        self._ac_navigator.cancel_goal()

    def loginfo(self, msg):
        rospy.loginfo('%s : %s'%(rospy.get_name(), str(msg)))

    def logerr(self, msg):
        rospy.logerr('%s : %s'%(rospy.get_name(), str(msg)))


if __name__ == '__main__':
    rospy.init_node('navigate_commander')

    robot_name, target = parse_arguments() 

    nc = NavigateCommander(robot_name)
    nc.loginfo("initialised")
    nc.command(target)
    nc.loginfo("Bye Bye")
