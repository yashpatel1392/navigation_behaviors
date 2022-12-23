#!/usr/bin/env python

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient, ProxySubscriberCached

from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import *
from geometry_msgs.msg import *
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion


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
        
        
    def check_positions_matched(self, map_pose_topic, goal_pose):
        print("--- --- --- map_pose_topic: ", map_pose_topic)
        
        if self._sub.has_msg(map_pose_topic):
            self._map_pose_data = self._sub.get_last_msg(map_pose_topic)
            self._sub.remove_last_msg(map_pose_topic)
        else:
            return False
        
            
        orient = self._map_pose_data.pose.orientation
        (_, _, current_yaw) = euler_from_quaternion([orient.x,orient.y,orient.z,orient.w])        
        
        g_orient = goal_pose.pose.orientation
        (_, _, goal_yaw) = euler_from_quaternion([g_orient.x,g_orient.y,g_orient.z,g_orient.w])
        
        print("Current (%f, %f, %f) Goal (%f, %f, %f)" % (self._map_pose_data.pose.position.x, self._map_pose_data.pose.position.y, current_yaw, goal_pose.pose.position.x, goal_pose.pose.position.y, goal_yaw))
        
        if all([abs(self._map_pose_data.pose.position.x - goal_pose.pose.position.x) < 0.5,
            abs(self._map_pose_data.pose.position.y - goal_pose.pose.position.y) < 0.5,
            abs(current_yaw - goal_yaw) < 0.09]):
                Logger.loginfo("Positions matched, goal is reached!")
                return True

        return False

    def execute(self, userdata):
        for map_pose_topic, goal_pose, action_topic in zip(self._map_pose_topics, userdata.robot_goals_IN, self._action_topics):    
            print("----------action topic: ", action_topic)
            if self.check_positions_matched(map_pose_topic, goal_pose):
                print("positions matched ---------------------------")
                if self._clients.has_result(action_topic):
                    print("results matched ---------------------------")
                    status = self._clients.get_state(action_topic)
                    if status == GoalStatus.SUCCEEDED:
                        print("status matched ---------------------------")
                        if self._outcomes_list[action_topic] != True:
                            self._outcomes_list[action_topic] = True 
                    elif status in [GoalStatus.PREEMPTED, GoalStatus.REJECTED, GoalStatus.ABORTED]:
                        return 'failed'
        
        print(self._outcomes_list.values())
        if all(self._outcomes_list.values()):
            return 'success'


    def on_enter(self, userdata):             
        Logger.loginfo("OnEnter Begins")
        print("---------------------------------------------------------------------")
        print("Num paths: ", len(userdata.robot_goals_IN))
        for j in range(len(userdata.robot_goals_IN)):
            print("==============")
            print("x: ", userdata.robot_goals_IN[j].pose.position.x)
            print("y: ", userdata.robot_goals_IN[j].pose.position.y)
            print("w: ", userdata.robot_goals_IN[j].pose.orientation.w)
            print("==============")
        print("Actions topics: ", self._action_topics)
        print("Robot Names: ", self._robot_names_list)
        print("Outcomes: ", self._outcomes_list)
        print("---------------------------------------------------------------------")

        for i in range(len(self._action_topics)):
            goal_pose = userdata.robot_goals_IN[i]
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "map"
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose = goal_pose.pose
            self._clients.send_goal(self._action_topics[i], goal)
            
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
