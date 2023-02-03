#!/usr/bin/env python

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxySubscriberCached, ProxyActionClient
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from actionlib_msgs.msg import GoalStatus
from tf.transformations import euler_from_quaternion
from move_base_msgs.msg import *
import rospy, time, threading


class TuwMBOA(EventState):
    """
    This state uses move base for all the robots to navigate to their goals. Goal
    position is received from the path published by TUW.

    -- robot_names   string      robot namespaces.
    -- robot_goals   stirng      list of goal positions (x, y, orient)

    <= success                   indicates successful completion of navigation.
    <= failed                    indicates unsuccessful completion of navigation.

    """

    def __init__(self, robot_names, robot_goals):
        # Declare outcomes, input_keys, and output_keys by calling the super constructor with the corresponding arguments.

        super(TuwMBOA, self).__init__(outcomes=['success', 'failed'])
        self._robot_names_list = robot_names.split(", ")
        self._robot_goals_first_list = [float(i) for i in robot_goals.split(", ")]
        self._robot_goals_list = [self._robot_goals_first_list[i:i + 3] for i in
                                  range(0, len(self._robot_goals_first_list), 3)]        
        self._robot_path_topics = []
        self._map_pose_topics = []
        self._action_topics = []
                
        for name in self._robot_names_list:
            self._map_pose_topics.append(name + "/map_pose")
            self._action_topics.append(name + "_move_base")
            self._robot_path_topics.append(name + "/path")
        
        self._robot_subs_dict = dict.fromkeys(self._robot_path_topics, Path)
        self._map_pose_dict = dict.fromkeys(self._map_pose_topics, PoseStamped)
        self._action_dict = dict.fromkeys(self._action_topics, MoveBaseAction)        
        self._clients = ProxyActionClient(self._action_dict)        
        self._threads_list = []
        self._goals_dict = {}
        self._success_dict = dict.fromkeys(self._robot_names_list, False)
        self._sub = ProxySubscriberCached(self._robot_subs_dict)
        self._map_pose_subs = ProxySubscriberCached(self._map_pose_dict)
        self._poses_received = {}
        self._outcomes_list = dict.fromkeys(self._action_topics, False)
        self._enter_loop = False        
        self.counter = 0


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

    
    def send_goal(self, action_topic, goal_pose):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = goal_pose.pose
        self._clients.send_goal(action_topic, goal)


    def execute(self, userdata):
        # This method is called periodically while the state is active.
		# Main purpose is to check state conditions and trigger a corresponding outcome.
		# If no outcome is returned, the state will stay active.

        # for some reason this method wasn't ran periodically, therefore, infinite loop is used.
        while True:        
            if self._enter_loop:   
                for name, action_topic, map_pose_topic in zip(self._robot_names_list, self._action_topics, self._map_pose_topics):
                    if not self._poses_received[name] and self._goals_dict[name][0]:
                        self.send_goal(action_topic, self._goals_dict[name][1])
                        self._poses_received[name] = True
                    
                    if self._poses_received[name]:
                        if self.check_positions_matched(map_pose_topic, self._goals_dict[name][1]):
                            if self._clients.has_result(action_topic):
                                status = self._clients.get_state(action_topic)
                                if status == GoalStatus.SUCCEEDED:
                                    if self._outcomes_list[action_topic] != True:
                                        self._outcomes_list[action_topic] = True 
                                elif status in [GoalStatus.PREEMPTED, GoalStatus.REJECTED, GoalStatus.ABORTED]:
                                    return 'failed'

            print(self._success_dict.values())
            print(self._outcomes_list)

            if all(self._success_dict.values()) and all(self._outcomes_list.values()):
                for thread in self._threads_list:
                    thread.join()
                return 'success'


    # This function is used by the threads for subscribing to TUW to get paths
    def is_path_received(self, robot_name, robot_goal):
        while True:
            if self._sub.has_msg(robot_name + "/path"):   
                self._outcomes_list[robot_name + "_move_base"] = False  
                path = self._sub.get_last_msg(robot_name + "/path")
                                
                new_goal_pose = PoseStamped()
                new_goal_pose.pose = path.poses[len(path.poses) - 1].pose
                self._goals_dict[robot_name] = (True, new_goal_pose)
                self._poses_received[robot_name] = False
                
                self._sub.remove_last_msg(robot_name + "/path")
                
                checklist = [abs(new_goal_pose.pose.position.x - robot_goal[0]) < 0.5,
                             abs(new_goal_pose.pose.position.y - robot_goal[1]) < 0.5]
                
                self._enter_loop = True       

                if all(checklist):
                    self._success_dict[robot_name] = True
                    return


    def on_enter(self, userdata):
        # This method is called when the state becomes active, i.e. a transition from another state to this one is taken.
      
        self._threads_list = []      
        self._goals_dict = {} 

        for name in self._robot_names_list:
            self._goals_dict[name] = [False, PoseStamped()]

        self._enter_loop = False
        self._poses_received = {}
        self._outcomes_list = dict.fromkeys(self._action_topics, False)
        
        for i in range(len(self._robot_names_list)):
            thread = threading.Thread(target=self.is_path_received, args=(self._robot_names_list[i], self._robot_goals_list[i],))
            if not thread.is_alive():
                self._threads_list.append(thread)
                self._threads_list[i].start()


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
