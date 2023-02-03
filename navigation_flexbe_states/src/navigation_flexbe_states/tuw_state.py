#!/usr/bin/env python

from flexbe_core import EventState
from flexbe_core.proxy import ProxyPublisher
from tuw_multi_robot_msgs.msg import RobotGoalsArray, RobotGoals
from geometry_msgs.msg import Pose, PoseStamped

import rospy, time


class TuwState(EventState):
    """
    This state publishes the goal coordinates to TUW, which then publishes the path
    for each of the robot individually. This state should be run parallel to 
    tuw_goal_sender_mthread or tuw_oa, depending on config being used.

    -- robot_names   string      list of robot namespaces.
    -- robot_goals   string      list of robot goals.

    <= success                  indicates successful completion of navigation.
    <= failed                   indicates unsuccessful completion of navigation.

    """

    def __init__(self, robot_names, robot_goals):
        # Declare outcomes, input_keys, and output_keys by calling the super constructor with the corresponding arguments.

        super(TuwState, self).__init__(outcomes=['success', 'failed'])
        self._robot_names_list = robot_names.split(", ")
        self._robot_goals_first_list = [float(i) for i in robot_goals.split(", ")]
        self._robot_goals_list = [self._robot_goals_first_list[i:i + 3] for i in
                                    range(0, len(self._robot_goals_first_list), 3)]
        self._pub = ProxyPublisher({"/goals": RobotGoalsArray})
        self._sent_goals = RobotGoalsArray()
        

    def execute(self, userdata):
        # This method is called periodically while the state is active.
        # Main purpose is to check state conditions and trigger a corresponding outcome.
        # If no outcome is returned, the state will stay active.
        
        goal_msg = RobotGoalsArray()
        goal_msg.header.frame_id = "map"
        goal_msg.header.stamp = rospy.Time.now()
        goal_msg.header.seq = 0
        
        for name, robot_goal in zip(self._robot_names_list, self._robot_goals_list):
            robot = RobotGoals()
            robot.robot_name = name
            
            goal_pose_msg = PoseStamped()
            goal_pose_msg.header = name + "/goal_pose"
        
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
            
        self._pub.publish("/goals", goal_msg)        
        self._sent_goals = goal_msg
        return 'success'
        
        
    def on_enter(self, userdata):
        # This method is called when the state becomes active, i.e. a transition from another state to this one is taken.
        
        time.sleep(5) # can be removed/reduced after testing
        

    def on_exit(self, userdata):
        # This method is called when an outcome is returned and another state gets active.
        pass # Nothing to do here        


    def on_stop(self):
        # This method is called whenever the behavior stops execution, also if it is cancelled.
        pass # Nothing to do here
