#!/usr/bin/env python

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient, ProxySubscriberCached

from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import *
from nav_msgs.msg import Path, Odometry
from mbf_msgs.msg import ExePathAction
from mbf_msgs.msg import ExePathGoal
from mbf_msgs.msg import MoveBaseAction
from move_base_msgs.msg import *


import rospy


# ># waypoint     Pose2D      goal coordinates for the robot.

class MoveBaseMovers(EventState):
    """
    -- robot_names  string      robot namespace.
    
    <= success                  indicates successful completion of navigation.
    <= failed                   indicates unsuccessful completion of navigation.

    """

    def __init__(self, robot_names):
        super(MoveBaseMovers, self).__init__(outcomes=['success', 'failed'],
                                        input_keys=['robot_goals_IN'])
        
        self._robot_names_list = robot_names.split(", ")
        
        self._goal_pose_topics = []
        self._odom_topics = []
        self._action_topics = []
        
        for name in self._robot_names_list:
            self._goal_pose_topics.append(name + "/goal_pose")
            self._odom_topics.append(name + "/odom")
            self._action_topics.append(name + "_move_base")
        
        self._odom_dict = dict.fromkeys(self._odom_topics, Odometry)
        self._goal_pose_dict = dict.fromkeys(self._goal_pose_topics, PoseStamped)
        self._action_dict = dict.fromkeys(self._action_topics, ExePathAction)
        
        self._odom_sub = ProxySubscriberCached(self._odom_dict)
        self._goal_pose_subs = ProxySubscriberCached(self._goal_pose_dict)
        
        self._clients = ProxyActionClient(self._action_dict)
        
        

    def execute(self, userdata):
        Logger.loginfo("Execute Begins")
        
        for odom_topic, goal_pose_topic, action_topic in self._odom_topics, self._goal_pose_topics, self._action_topics:
        # checker 
            if self._sub.has_msg(odom_topic):
                self._odom_data = self._sub.get_last_msg(odom_topic)
                self._sub.remove_last_msg(odom_topic)
            
            if self._sub.has_msg(goal_pose_topic):
                self._goal_pose_data = self._sub.get_last_msg(goal_pose_topic)
                self._sub.remove_last_msg(goal_pose_topic)
        
            self._positions_match = False
            
            if all([abs(self._odom_data.pose.pose.position.x - self._goal_pose_data.pose.position.x) < 0.5,
                abs(self._odom_data.pose.pose.position.y - self._goal_pose_data.pose.position.y) < 0.5,
                abs(self._odom_data.pose.pose.orientation.w - self._goal_pose_data.pose.orientation.w) < 0.09]):
                    Logger.loginfo("Positions matched, goal is reached!")
                    self._positions_match = True
            
            if self._positions_match and self._clients.has_result(action_topic):
                status = self._clients.get_state(action_topic)
                if status == GoalStatus.SUCCEEDED:
                    self._map[action_topic] = True
            
                elif status in [GoalStatus.PREEMPTED, GoalStatus.REJECTED, GoalStatus.ABORTED]:
                    self._map[action_topic] = False
        
        counter = 0
        for i in self._map:
            if self._map[i] == True:
                counter += 1
        
        Logger.loginfo("Execute Ends")
        
        if counter == len(self._action_topics):
            return 'success'
        else:
            return 'failed'


    def on_enter(self, userdata):      
        Logger.loginfo("OnEnter Begins")
            
        # for i in range(len(self._action_topics)):
        #     goal = ExePathGoal()
        #     goal.controller = "TrajectoryPlannerROS"
        #     goal.path = userdata.robot_paths_IN[i]
        #     self._clients.send_goal(self._action_topics[i], goal)
            
        #         self._map = {}

        for i in range(len(self._action_topics)):
            goal_pose = userdata.robot_goals_IN[i]
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "map"
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose = goal_pose.pose
            # goal.target_pose.pose.position.x = goal_pose.pose.position.x
            # goal.target_pose.pose.position.y = goal_pose.pose.position.y
            # goal.target_pose.pose.orientation.w = goal_pose.pose.orientation.w
            self._clients(self._action_topics[i], goal)
            
        Logger.loginfo("OnEnter Ends")


    # copied from armada_behaviors
    def cancel_active_goals(self):
        for action_topic in self._action_topics:
            if self._clients.is_available(action_topic):
                if self._clients.is_active(action_topic):
                    if not self._clients.has_result(action_topic):
                        self._clients.cancel(action_topic)
                        Logger.loginfo('Cancelled move_base active action goal.')

    # copied from armada_behaviors
    def on_exit(self, userdata):
        self.cancel_active_goals()
        
    # copied from armada_behaviors
    def on_stop(self):
        self.cancel_active_goals()
