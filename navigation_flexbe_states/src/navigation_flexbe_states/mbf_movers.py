#!/usr/bin/env python

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient, ProxySubscriberCached

from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import *
from nav_msgs.msg import Path, Odometry
from mbf_msgs.msg import ExePathAction
from mbf_msgs.msg import ExePathGoal
from tf.transformations import euler_from_quaternion

import time


class MBFMovers(EventState):
    """
    This state uses move base flex exe_path action state for having multiple robots 
    navigate to its goal following the input paths.

    -- robot_names      string      list of the robot names.

    ># robot_paths_IN   Path[]      list of the paths for each robot.

    <= success                      indicates successful completion of navigation.
    <= failed                       indicates unsuccessful completion of navigation.

    """
    
    def __init__(self, robot_names):
        # Declare outcomes, input_keys, and output_keys by calling the super constructor with the corresponding arguments.

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
        # This method is called periodically while the state is active.
		# Main purpose is to check state conditions and trigger a corresponding outcome.
		# If no outcome is returned, the state will stay active.

        for map_pose_topic, goal_pose_topic, action_topic, goal_pose_data in zip(self._map_pose_topics, self._goal_pose_topics, self._action_topics, self._goal_poses):
            if self._sub.has_msg(map_pose_topic):
                self._map_pose_data = self._sub.get_last_msg(map_pose_topic)
                self._sub.remove_last_msg(map_pose_topic)
        
            orient = self._map_pose_data.pose.orientation
            (_, _, current_yaw) = euler_from_quaternion([orient.x,orient.y,orient.z,orient.w])       
            
            g_orient = goal_pose_data.pose.orientation
            (_, _, goal_yaw) = euler_from_quaternion([g_orient.x,g_orient.y,g_orient.z,g_orient.w])
            
            self._positions_match = False          
        
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
                
        if counter == len(self._action_topics):
            return 'success'

    
    def on_enter(self, userdata):   
        # This method is called when the state becomes active, i.e. a transition from another state to this one is taken.
  
        for action_topic in self._action_topics:
            path_goal = userdata.robot_paths_IN[action_topic]
            
            goal = ExePathGoal()
            goal.controller = "TrajectoryPlannerROS"
            goal.path.poses = path_goal.poses

            x = path_goal.poses[len(path_goal.poses) - 1].pose.position.x
            y = path_goal.poses[len(path_goal.poses) - 1].pose.position.y
            w = path_goal.poses[len(path_goal.poses) - 1].pose.orientation.w
            
            temp_goal_pose = PoseStamped()
            temp_goal_pose.pose.position.x = x
            temp_goal_pose.pose.position.y = y
            temp_goal_pose.pose.orientation.w = w

            self._goal_poses.append(temp_goal_pose)
            self._clients.send_goal(action_topic, goal)

            time.sleep(5) # can be removed/reduced after testing
    

    # This function cancels active goals for each of the robots.
    def cancel_active_goals(self):
        for action_topic in self._action_topics:
            if self._clients.is_available(action_topic):
                if self._clients.is_active(action_topic):
                    if not self._clients.has_result(action_topic):
                        self._clients.cancel(action_topic)
                        Logger.loginfo('Cancelled move_base_flex active action goal.')
    

    def on_exit(self, userdata):
        # This method is called when an outcome is returned and another state gets active.
        self.cancel_active_goals()
        

    def on_stop(self):
        # This method is called whenever the behavior stops execution, also if it is cancelled.
        self.cancel_active_goals()
    