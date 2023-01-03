#!/usr/bin/env python

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxySubscriberCached, ProxyActionClient
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from actionlib_msgs.msg import GoalStatus

import rospy, time
import threading
from tf.transformations import euler_from_quaternion
from move_base_msgs.msg import *


class TuwMBOA(EventState):
    """
    -- robot_names   string      robot namespaces.

    #> robot_paths   list of paths  this is the list of robot paths

    <= success                  indicates successful completion of navigation.
    <= failed                   indicates unsuccessful completion of navigation.

    """

    def __init__(self, robot_names, robot_goals):
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
        self._outcomes_list = dict.fromkeys(self._action_topics, False)


        self._threads_list = []
        
        self._goals_dict = {}
        self._success_dict = dict.fromkeys(self._robot_names_list, False)
                
        self._sub = ProxySubscriberCached(self._robot_subs_dict)
        self._map_pose_subs = ProxySubscriberCached(self._map_pose_dict)

        self._poses_received = {}

        self._enter_loop = False



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

    def send_goal(self, action_topic, goal_pose):
        print("called send_goal() with ", action_topic)
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = goal_pose.pose
        self._clients.send_goal(action_topic, goal)

    def execute(self, userdata):
        while True:
            print("-------------------- In execute()")
            print("--------------- entering for loop")
            print("bool value enter loop ", self._enter_loop)
            
            if self._enter_loop:
                print("bool value ", self._goals_dict["pioneer"][0])
                
                for name, action_topic, map_pose_topic in zip(self._robot_names_list, self._action_topics, self._map_pose_topics):
                    if not self._poses_received[name] and self._goals_dict[name][0]:
                        self.send_goal(action_topic, self._goals_dict[name][1])
                        # self._goals_dict[name][0] = False
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
            
            # print("waiting for 10 secs")
            # time.sleep(10)
            # print("waiting finished........")
            # self._enter_loop = True

            print(self._success_dict.values())
            print(self._outcomes_list)

            if all(self._success_dict.values()) and all(self._outcomes_list.values()):
                print("exiting..........")
                for thread in self._threads_list:
                    thread.join()
                return 'success'
            
            print("----- out of execute")
        

    
    def is_path_received(self, robot_name, robot_goal):
        while True:
            # goal_pose = PoseStamped()
            # self._goals_dict[robot_name] = [False, goal_pose]
            if self._sub.has_msg(robot_name + "/path"):      
                print("found a path for -------------- ", robot_name)          
                path = self._sub.get_last_msg(robot_name + "/path")
                                
                new_goal_pose = PoseStamped()
                new_goal_pose.pose = path.poses[len(path.poses) - 1].pose
                self._goals_dict[robot_name] = (True, new_goal_pose)
                self._poses_received[robot_name] = False
                
                self._sub.remove_last_msg(robot_name + "/path")
                
                checklist = [abs(new_goal_pose.pose.position.x - robot_goal[0]) < 0.5,
                             abs(new_goal_pose.pose.position.y - robot_goal[1]) < 0.5]
                            #  abs(new_goal_pose.pose.orientation.w - robot_goal[2] < 0.09)]
                
                print("done for -------------- ", robot_name)   
                print("boolean value: ", self._goals_dict[robot_name][0])
                self._enter_loop = True       

                if all(checklist):
                    print("success for -------------- ", robot_name)
                    self._success_dict[robot_name] = True
                    return


    def on_enter(self, userdata):
        print("-------------------- In on_enter()")
        
        if len(self._threads_list) != 0:
            self._threads_list.clear()        
        
        for i in range(len(self._robot_names_list)):
            print("Robot Name: ", self._robot_names_list[i])
            thread = threading.Thread(target=self.is_path_received, args=(self._robot_names_list[i], self._robot_goals_list[i],))
            if not thread.is_alive():
                self._threads_list.append(thread)
                self._threads_list[i].start()


    def on_exit(self, userdata):
        # print("Number of paths sent out are: %f" % (len(self._robot_paths)))
        # print("Number of paths sent out are: %f" % (len(userdata.robot_paths_OUT)))
        
        # for thread in self._threads_list:
        #     thread.exit()
        pass

    def on_stop(self):
        pass
