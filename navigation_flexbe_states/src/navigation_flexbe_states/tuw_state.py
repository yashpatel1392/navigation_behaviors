#!/usr/bin/env python

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient, ProxySubscriberCached, ProxyPublisher
from tuw_multi_robot_msgs.msg import RobotGoalsArray, RobotGoals
from geometry_msgs.msg import Pose, PoseStamped


import rospy, time

class TuwState(EventState):

    """
        -- robot_names   string      robot namespaces.
        #> robot_goals   string      robot goals.

        <= success                  indicates successful completion of navigation.
        <= failed                   indicates unsuccessful completion of navigation.

    """

    def __init__(self, robot_names, robot_goals):
        super(TuwState, self).__init__(outcomes=['success', 'failed'])
        self._robot_names_list = robot_names.split(", ")
        self._robot_goals_first_list = [float(i) for i in robot_goals.split(", ")]
        self._robot_goals_list = [self._robot_goals_first_list[i:i + 3] for i in
                                  range(0, len(self._robot_goals_first_list), 3)]

        self._pub = ProxyPublisher({"/goals": RobotGoalsArray})
        self._sent_goals = RobotGoalsArray()
        

    def execute(self, userdata):
        goal_msg = RobotGoalsArray()
        goal_msg.header.frame_id = "map"
        goal_msg.header.stamp = rospy.Time.now()
        goal_msg.header.seq = 0
        
        for name, robot_goal in zip(self._robot_names_list, self._robot_goals_list):
            robot = RobotGoals()
            robot.robot_name = name
            
            goal_pose_msg = PoseStamped()
            goal_pose_msg.header = name + "/goal_pose"
            # print("-------------- (%f, %f, %f) ------------" % (robot_goal[0],robot_goal[1],robot_goal[2]))
            print("--------------")
            print(robot_goal[0])
            print(robot_goal[1])
            print(robot_goal[2])
            print("--------------")
            goal_pose = Pose()
            goal_pose.position.x = robot_goal[0]
            goal_pose.position.y = robot_goal[1]
            goal_pose.position.z = 0.0
            goal_pose.orientation.x = 0.0
            goal_pose.orientation.y = 0.0
            goal_pose.orientation.z = 0.0
            goal_pose.orientation.w = robot_goal[2]
            
            self._pub.createPublisher(name + "/goal_pose", PoseStamped)
            goal_pose_msg.pose = goal_pose
            self._pub.publish(name + "/goal_pose", goal_pose_msg)
            
            robot.destinations.append(goal_pose)
            
            goal_msg.robots.append(robot)
            
        print("--------Number of robots: %f" % (len(goal_msg.robots)))
        self._pub.publish("/goals", goal_msg)
        
        
        self._sent_goals = goal_msg
        return 'success'

        
        
    def on_enter(self, userdata):
        print("-=-=-=-=-= ONENTER")
        time.sleep(5)
        

    def on_exit(self, userdata):
        for i in range(len(self._robot_names_list)):
            print("robot name: ", self._sent_goals.robots[i].robot_name)
        

    def on_stop(self):
        pass

