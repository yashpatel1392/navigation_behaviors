#!/usr/bin/env python

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient

from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import *
from geometry_msgs.msg import *
from tf import transformations

import rospy


class MoveBaseActionState(EventState):
    """
    -- robot_name   string      robot namespace.

    ># waypoint     Pose2D      goal coordinates for the robot.

    <= success                  indicates successful completion of navigation.
    <= failed                   indicates unsuccessful completion of navigation.

    """

    def __init__(self, robot_name):
        super(MoveBaseActionState, self).__init__(outcomes=['success', 'failed'],
                                                  input_keys=['waypoint'])

        self._action_topic = robot_name + "/move_base"
        self._client = ProxyActionClient({self, MoveBaseAction})

    def execute(self, userdata):
        if self._client.has_result(self._action_topic):
          
            status = self._client.get_state(self._action_topic)
            
            if status == GoalStatus.SUCCEEDED:
                return 'success'
              
            elif status in [GoalStatus.PREEMPTED, GoalStatus.REJECTED, GoalStatus.ABORTED]:
                return 'failed'

    def on_enter(self, userdata):
        goal = MoveBaseGoal()

        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = userdata.waypoint.x
        goal.target_pose.pose.position.y = userdata.waypoint.y
        goal.target_pose.pose.orientation.w = userdata.waypoint.theta

        self._client.send_goal(self._action_topic, goal)

    # copied from armada_behaviors
    def cancel_active_goals(self):
        if self._client.is_available(self._action_topic):
            if self._client.is_active(self._action_topic):
                if not self._client.has_result(self._action_topic):
                    self._client.cancel(self._action_topic)
                    Logger.loginfo('Cancelled move_base active action goal.')

    # copied from armada_behaviors
    def on_exit(self, userdata):
        self.cancel_active_goals()
        
    # copied from armada_behaviors
    def on_stop(self):
        self.cancel_active_goals()
