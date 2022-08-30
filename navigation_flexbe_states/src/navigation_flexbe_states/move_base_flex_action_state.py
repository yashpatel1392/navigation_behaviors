#!/usr/bin/env python

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient, ProxySubscriberCached

from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import *
from nav_msgs.msg import Path, Odometry
from mbf_msgs.msg import ExePathAction
from mbf_msgs.msg import ExePathGoal
from mbf_msgs.msg import MoveBaseAction

import rospy

# ># waypoint     Pose2D      goal coordinates for the robot.

class MoveBaseFlexActionState(EventState):
    """
    -- robot_name   string      robot namespace.
    
    <= success                  indicates successful completion of navigation.
    <= failed                   indicates unsuccessful completion of navigation.

    """

    def __init__(self, robot_name):
        super(MoveBaseFlexActionState, self).__init__(outcomes=['success', 'failed'])
        
        self._robot_name = robot_name
        
        Logger.loginfo("%s" % str(self._robot_name))

        self._path_topic = self._robot_name + "/path"
        self._odom_topic = self._robot_name + "/odom"
        self._goal_pose_topic = self._robot_name + "/goal_pose"
        
        self._sub = ProxySubscriberCached({self._path_topic: Path, self._odom_topic: Odometry, self._goal_pose_topic: PoseStamped})
        
        self._action_topic = self._robot_name + "_move_base_flex/exe_path"
        Logger.loginfo("\n%s" % str(self._action_topic))
        self._client = ProxyActionClient({self._action_topic: ExePathAction})

    def execute(self, userdata):
        Logger.loginfo("Execute Begins")
        # checker 
        if self._sub.has_msg(self._odom_topic):
            self._odom_data = self._sub.get_last_msg(self._odom_topic)
            self._sub.remove_last_msg(self._odom_topic)
        
        if self._sub.has_msg(self._goal_pose_topic):
            self._goal_pose_data = self._sub.get_last_msg(self._goal_pose_topic)
            self._sub.remove_last_msg(self._goal_pose_topic)
        
        self._positions_match = False
        
        if all([abs(self._odom_data.pose.pose.position.x - self._goal_pose_data.pose.position.x) < 0.5,
            abs(self._odom_data.pose.pose.position.y - self._goal_pose_data.pose.position.y) < 0.5,
            abs(self._odom_data.pose.pose.orientation.w - self._goal_pose_data.pose.orientation.w) < 0.09]):
                Logger.loginfo("Positions matched, goal is reached!")
                self._positions_match = True
        
        if self._positions_match and self._client.has_result(self._action_topic):
            status = self._client.get_state(self._action_topic)
            if status == GoalStatus.SUCCEEDED:
                return 'success'
        
            elif status in [GoalStatus.PREEMPTED, GoalStatus.REJECTED, GoalStatus.ABORTED]:
                return 'failed'
        
        Logger.loginfo("Execute Ends")


    def on_enter(self, userdata):      
        Logger.loginfo("OnEnter Begins")
            
        while True:
            if self._sub.has_msg(self._path_topic):
                goal = ExePathGoal()
                goal.controller = "TrajectoryPlannerROS"
                goal.path = self._sub.get_last_msg(self._path_topic)
                self._client.send_goal(self._action_topic, goal)
                break
        
        Logger.loginfo("OnEnter Ends")


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
