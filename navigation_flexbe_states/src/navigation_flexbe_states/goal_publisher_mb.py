#!/usr/bin/env python

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyPublisher 
from geometry_msgs.msg import PoseStamped

import rospy

class GoalPublisherMB(EventState):
    """
    -- robot_names   string      robot namespaces.
    -- robot_goals   string      robot goals
    
    <= success                  indicates successful completion of navigation.
    <= failed                   indicates unsuccessful completion of navigation.

    """
    
    def __init__(self, robot_names, robot_goals):
        super(GoalPublisherMB, self).__init__(outcomes=['success', 'failed'],
                                              output_keys=['goals'])
        
        self._robot_names_list = robot_names.split(", ")
        self._robot_goals_first_list = [float(i) for i in robot_goals.split(", ")]
        self._robot_goals_list = [self._robot_goals_first_list[i:i + 3] for i in range(0, len(self._robot_goals_first_list), 3)]
        self._goals = []
        self._pub = ProxyPublisher()
    
    def execute(self, userdata):
        userdata.goals = self._goals
        return 'success'
    
    def on_enter(self, userdata):
        for i in range(len(self._robot_names_list)):
            current_topic = self._robot_names_list[i] + "/goal_pose"
            print(current_topic)
            print(self._robot_goals_list[i])
            self._pub.createPublisher(current_topic, PoseStamped)
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = self._robot_names_list[i] + "/goal_pose"
            goal_pose.pose.position.x = self._robot_goals_list[i][0]
            goal_pose.pose.position.y = self._robot_goals_list[i][1]
            goal_pose.pose.orientation.w = self._robot_goals_list[i][2]
            self._goals.append(goal_pose)
            self._pub.publish(current_topic, goal_pose)

            

    def on_exit(self, userdata):
        pass
    
    def on_stop(self):
        pass
