#!/usr/bin/env python

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient, ProxySubscriberCached

from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import *
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

import rospy


class MoveBaseMovers(EventState):
    """
    This state sets up move base for multiple robots together as well as sends the goal 
    for each of the robot as well. Goals are received as input keys.

    -- robot_names          string              list of robot names.

    ># robot_goals_IN       PoseStamped[]       list of the goals for each of the robot.

    <= success                                  indicates successful completion of navigation.
    <= failed                                   indicates unsuccessful completion of navigation.

    """

    def __init__(self, robot_names):
        # Declare outcomes, input_keys, and output_keys by calling the super constructor with the corresponding arguments.

        super(MoveBaseMovers, self).__init__(outcomes=['success', 'failed'],
                                                input_keys=['robot_goals_IN'])
        self._robot_names_list = robot_names.split(", ")
        self._goal_pose_topics = []
        self._odom_topics = []
        self._map_pose_topics = []
        self._action_topics = []
        
        for name in self._robot_names_list:
            self._goal_pose_topics.append(name + "/goal_pose")
            self._odom_topics.append(name + "/odom")
            self._map_pose_topics.append(name + "/map_pose")
            self._action_topics.append(name + "_move_base")
        
        self._odom_dict = dict.fromkeys(self._odom_topics, Odometry)
        self._map_pose_dict = dict.fromkeys(self._map_pose_topics, PoseStamped)
        self._goal_pose_dict = dict.fromkeys(self._goal_pose_topics, PoseStamped)
        self._action_dict = dict.fromkeys(self._action_topics, MoveBaseAction)        
        self._outcomes_list = dict.fromkeys(self._action_topics, False)

        
        self._odom_sub = ProxySubscriberCached(self._odom_dict)
        self._goal_pose_subs = ProxySubscriberCached(self._goal_pose_dict)
        self._map_pose_subs = ProxySubscriberCached(self._map_pose_dict)

        self._clients = ProxyActionClient(self._action_dict)
        

    # Checker
    def check_positions_matched(self, map_pose_topic, goal_pose):       
        if self._sub.has_msg(map_pose_topic):
            self._map_pose_data = self._sub.get_last_msg(map_pose_topic)
            self._sub.remove_last_msg(map_pose_topic)
        else:
            return False

        orient = self._map_pose_data.pose.orientation
        (_, _, current_yaw) = euler_from_quaternion([orient.x,orient.y,orient.z,orient.w])        
        g_orient = goal_pose.pose.orientation
        (_, _, goal_yaw) = euler_from_quaternion([g_orient.x,g_orient.y,g_orient.z,g_orient.w])
                
        if all([abs(self._map_pose_data.pose.position.x - goal_pose.pose.position.x) < 0.5,
            abs(self._map_pose_data.pose.position.y - goal_pose.pose.position.y) < 0.5,
            abs(current_yaw - goal_yaw) < 0.09]):
                Logger.loginfo("Positions matched, goal is reached!")
                return True

        return False


    def execute(self, userdata):
        # This method is called periodically while the state is active.
        # Main purpose is to check state conditions and trigger a corresponding outcome.
        # If no outcome is returned, the state will stay active.

        for map_pose_topic, goal_pose, action_topic in zip(self._map_pose_topics, userdata.robot_goals_IN, self._action_topics):    
            if self.check_positions_matched(map_pose_topic, goal_pose):
                if self._clients.has_result(action_topic):
                    status = self._clients.get_state(action_topic)
                    if status == GoalStatus.SUCCEEDED:
                        if self._outcomes_list[action_topic] != True:
                            self._outcomes_list[action_topic] = True 
                    elif status in [GoalStatus.PREEMPTED, GoalStatus.REJECTED, GoalStatus.ABORTED]:
                        return 'failed'
        
        print(self._outcomes_list.values())
        if all(self._outcomes_list.values()):
            return 'success'


    def on_enter(self, userdata):
        # This method is called when the state becomes active, i.e. a transition from another state to this one is taken.
        
        for i in range(len(self._action_topics)):
            goal_pose = userdata.robot_goals_IN[i]
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "map"
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose = goal_pose.pose
            self._clients.send_goal(self._action_topics[i], goal)
            

    # This function cancels active goals for each of the robots.
    def cancel_active_goals(self):
        for action_topic in self._action_topics:
            if self._clients.is_available(action_topic):
                if self._clients.is_active(action_topic):
                    if not self._clients.has_result(action_topic):
                        self._clients.cancel(action_topic)
                        Logger.loginfo('Cancelled move_base active action goal.')


    def on_exit(self, userdata):
        # This method is called when an outcome is returned and another state gets active.
        self.cancel_active_goals()
        

    def on_stop(self):
        # This method is called whenever the behavior stops execution, also if it is cancelled.
        self.cancel_active_goals()
