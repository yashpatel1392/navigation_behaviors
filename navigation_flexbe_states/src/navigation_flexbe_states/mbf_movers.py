#!/usr/bin/env python

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient, ProxySubscriberCached

from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import *
from nav_msgs.msg import Path, Odometry
from mbf_msgs.msg import ExePathAction
from mbf_msgs.msg import ExePathGoal
from mbf_msgs.msg import MoveBaseAction
from tf.transformations import euler_from_quaternion

import time
import rospy


# ># waypoint     Pose2D      goal coordinates for the robot.

class MBFMovers(EventState):
    """
    -- robot_names  string      robot namespace.

    <= success                  indicates successful completion of navigation.
    <= failed                   indicates unsuccessful completion of navigation.

    """
    
    def __init__(self, robot_names):
        super(MBFMovers, self).__init__(outcomes=['success', 'failed'],
                                        input_keys=['robot_paths_IN'])
        
        self._robot_names_list = robot_names.split(", ")
        
        self._goal_pose_topics = []
        self._odom_topics = []
        self._action_topics = []
        self._map_pose_topics = []
        self._map = {}
        
        for name in self._robot_names_list:
            self._goal_pose_topics.append(name + "/goal_pose")
            self._odom_topics.append(name + "/odom")
            self._action_topics.append(name + "_move_base_flex/exe_path")
            self._map_pose_topics.append(name + "/map_pose")
            self._map[name + "_move_base_flex/exe_path"] = False
            
        print('==============================================================')
        print(self._goal_pose_topics)
        print(self._odom_topics)
        print(self._action_topics)
        print(self._map_pose_topics)
        print('==============================================================')
        
        
        self._odom_dict = dict.fromkeys(self._odom_topics, Odometry)
        self._map_pose_dict = dict.fromkeys(self._map_pose_topics, PoseStamped)
        self._goal_pose_dict = dict.fromkeys(self._goal_pose_topics, PoseStamped)
        self._action_dict = dict.fromkeys(self._action_topics, ExePathAction)
        self._outcomes_list = dict.fromkeys(self._action_topics, False)
    
        
        self._odom_sub = ProxySubscriberCached(self._odom_dict)
        self._goal_pose_subs = ProxySubscriberCached(self._goal_pose_dict)
        self._map_pose_subs = ProxySubscriberCached(self._map_pose_dict)
        
        self._clients = ProxyActionClient(self._action_dict)
        self._goal_poses = []
        
    
    def execute(self, userdata):
        Logger.loginfo("Execute Begins")
        
        for map_pose_topic, goal_pose_topic, action_topic, goal_pose_data in zip(self._map_pose_topics, self._goal_pose_topics, self._action_topics, self._goal_poses):
        # checker
            if self._sub.has_msg(map_pose_topic):
                self._map_pose_data = self._sub.get_last_msg(map_pose_topic)
                self._sub.remove_last_msg(map_pose_topic)
            
            # if self._sub.has_msg(goal_pose_topic):
            #     self._goal_pose_data = self._sub.get_last_msg(goal_pose_topic)
            #     self._sub.remove_last_msg(goal_pose_topic)

        
            orient = self._map_pose_data.pose.orientation
            (_, _, current_yaw) = euler_from_quaternion([orient.x,orient.y,orient.z,orient.w])       
            
            g_orient = goal_pose_data.pose.orientation
            (_, _, goal_yaw) = euler_from_quaternion([g_orient.x,g_orient.y,g_orient.z,g_orient.w])
            
            self._positions_match = False
            
            print("------------------------------")
            print(map_pose_topic, goal_pose_topic)
            print("Current (%f, %f, %f) Goal (%f, %f, %f)" % (self._map_pose_data.pose.position.x, self._map_pose_data.pose.position.y, current_yaw, goal_pose_data.pose.position.x, goal_pose_data.pose.position.y, goal_yaw))
            print("------------------------------")
            
        
            if all([abs(self._map_pose_data.pose.position.x - goal_pose_data.pose.position.x) < 0.5,
                abs(self._map_pose_data.pose.position.y - goal_pose_data.pose.position.y) < 0.5,
                abs(current_yaw - goal_yaw) < 0.09]):
                    Logger.loginfo("Positions matched, goal is reached!")
                    self._positions_match = True
            
            if self._positions_match and self._clients.has_result(action_topic):
                status = self._clients.get_state(action_topic)
                if status == GoalStatus.SUCCEEDED:
                    self._map[action_topic] = True
            
                elif status in [GoalStatus.PREEMPTED, GoalStatus.REJECTED, GoalStatus.ABORTED]:
                    self._map[action_topic] = False
                    return 'failed'
        
        counter = 0
        for i in self._map:
            if self._map[i] == True:
                counter += 1
        
        Logger.loginfo("Execute Ends")
        
        if counter == len(self._action_topics):
            return 'success'
        
    
    
    def on_enter(self, userdata):     
        Logger.loginfo("OnEnter Begins")
        print("--------------------------- Number of paths: ", len(userdata.robot_paths_IN.values()))
        
        
        for action_topic in self._action_topics:
            print("3. ", self._action_topics)
            path_goal = userdata.robot_paths_IN[action_topic]
            
            goal = ExePathGoal()
            goal.controller = "TrajectoryPlannerROS"
            goal.path.poses = path_goal.poses
            x = path_goal.poses[len(path_goal.poses) - 1].pose.position.x
            y = path_goal.poses[len(path_goal.poses) - 1].pose.position.y
            w = path_goal.poses[len(path_goal.poses) - 1].pose.orientation.w
            
            xg = goal.path.poses[len(goal.path.poses) - 1].pose.position.x
            yg = goal.path.poses[len(goal.path.poses) - 1].pose.position.y
            wg = goal.path.poses[len(goal.path.poses) - 1].pose.orientation.w
            
            temp_goal_pose = PoseStamped()
            temp_goal_pose.pose.position.x = x
            temp_goal_pose.pose.position.y = y
            temp_goal_pose.pose.orientation.w = w
            
            self._goal_poses.append(temp_goal_pose)
            
            print("-------------------------------------------------------------------")
            print(action_topic)
            print(" LAST POINT ON THE PATH IS:  (%f, %f, %f)" % (x,y,w))
            print(" GOAL LAST POINT ON THE PATH IS:  (%f, %f, %f)" % (xg,yg,wg))
            print("-------------------------------------------------------------------")
            
            self._clients.send_goal(action_topic, goal)
            time.sleep(5)
            
        # for i in range(len(self._action_topics)):
        #     goal = ExePathGoal()
        #     goal.controller = "TrajectoryPlannerROS"
        #     goal.path.poses = userdata.robot_paths_IN[i].poses
        #     x = userdata.robot_paths_IN[i].poses[len(userdata.robot_paths_IN[i].poses) - 1].pose.position.x
        #     y = userdata.robot_paths_IN[i].poses[len(userdata.robot_paths_IN[i].poses) - 1].pose.position.y
        #     w = userdata.robot_paths_IN[i].poses[len(userdata.robot_paths_IN[i].poses) - 1].pose.orientation.w
            
        #     xg = goal.path.poses[len(goal.path.poses) - 1].pose.position.x
        #     yg = goal.path.poses[len(goal.path.poses) - 1].pose.position.y
        #     wg = goal.path.poses[len(goal.path.poses) - 1].pose.orientation.w
            
        #     temp_goal_pose = PoseStamped()
        #     temp_goal_pose.pose.position.x = x
        #     temp_goal_pose.pose.position.y = y
        #     temp_goal_pose.pose.orientation.w = w
            
        #     self._goal_poses.append(temp_goal_pose)
            
        #     print("-------------------------------------------------------------------")
        #     print(self._action_topics[i])
        #     print("_____ LAST POINT ON THE PATH IS:  (%f, %f, %f)" % (x,y,w))
        #     print("_____ GOAL LAST POINT ON THE PATH IS:  (%f, %f, %f)" % (xg,yg,wg))
        #     print("-------------------------------------------------------------------")
            
        #     self._clients.send_goal(self._action_topics[i], goal)
        #     time.sleep(5)
        
        # self._action_topics.sort(reverse=True)
        # print("1. ", self._action_topics)
        # self._goal_dict = {self._action_topics[i]: userdata.robot_paths_IN[i] for i in range(len(self._action_topics))}
        # print("2. ", self._action_topics)
        
        # for action_topic in self._action_topics:
        #     print("3. ", self._action_topics)
        #     path_goal = self._goal_dict[action_topic]
            
        #     goal = ExePathGoal()
        #     goal.controller = "TrajectoryPlannerROS"
        #     goal.path.poses = path_goal.poses
        #     x = path_goal.poses[len(path_goal.poses) - 1].pose.position.x
        #     y = path_goal.poses[len(path_goal.poses) - 1].pose.position.y
        #     w = path_goal.poses[len(path_goal.poses) - 1].pose.orientation.w
            
        #     xg = goal.path.poses[len(goal.path.poses) - 1].pose.position.x
        #     yg = goal.path.poses[len(goal.path.poses) - 1].pose.position.y
        #     wg = goal.path.poses[len(goal.path.poses) - 1].pose.orientation.w
            
        #     temp_goal_pose = PoseStamped()
        #     temp_goal_pose.pose.position.x = x
        #     temp_goal_pose.pose.position.y = y
        #     temp_goal_pose.pose.orientation.w = w
            
        #     self._goal_poses.append(temp_goal_pose)
            
        #     print("-------------------------------------------------------------------")
        #     print(action_topic)
        #     print(" LAST POINT ON THE PATH IS:  (%f, %f, %f)" % (x,y,w))
        #     print(" GOAL LAST POINT ON THE PATH IS:  (%f, %f, %f)" % (xg,yg,wg))
        #     print("-------------------------------------------------------------------")
            
        #     self._clients.send_goal(action_topic, goal)
        #     time.sleep(5)
        
        # for action_topic, path_goal in zip(self._action_topics, userdata.robot_paths_IN):
        #     goal = ExePathGoal()
        #     goal.controller = "TrajectoryPlannerROS"
        #     goal.path.poses = path_goal.poses
        #     x = path_goal.poses[len(path_goal.poses) - 1].pose.position.x
        #     y = path_goal.poses[len(path_goal.poses) - 1].pose.position.y
        #     w = path_goal.poses[len(path_goal.poses) - 1].pose.orientation.w
            
        #     xg = goal.path.poses[len(goal.path.poses) - 1].pose.position.x
        #     yg = goal.path.poses[len(goal.path.poses) - 1].pose.position.y
        #     wg = goal.path.poses[len(goal.path.poses) - 1].pose.orientation.w
            
        #     temp_goal_pose = PoseStamped()
        #     temp_goal_pose.pose.position.x = x
        #     temp_goal_pose.pose.position.y = y
        #     temp_goal_pose.pose.orientation.w = w
            
        #     self._goal_poses.append(temp_goal_pose)
            
        #     print("-------------------------------------------------------------------")
        #     print(action_topic)
        #     print(" LAST POINT ON THE PATH IS:  (%f, %f, %f)" % (x,y,w))
        #     print(" GOAL LAST POINT ON THE PATH IS:  (%f, %f, %f)" % (xg,yg,wg))
        #     print("-------------------------------------------------------------------")
            
        #     self._clients.send_goal(action_topic, goal)
        #     time.sleep(5)
        
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
    
    









