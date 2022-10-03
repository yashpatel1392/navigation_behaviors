#!/usr/bin/env python

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient, ProxySubscriberCached, ProxyPublisher
from tuw_multi_robot_msgs import *

import rospy

class TuwState(EventState):

    """
        -- robot_names   string      robot namespaces.
        -- robot_goals   string      robot goals.

        <= success                  indicates successful completion of navigation.
        <= failed                   indicates unsuccessful completion of navigation.

    """

    def __init__(self, robot_names, robot_goals):
        super(TuwState, self).__init__(outcomes=['success', 'failed'])
        self._robot_names_list = robot_names.split(", ")
        self._robot_goals_first_list = [float(i) for i in robot_goals.split(", ")]
        self._robot_goals_list = [self._robot_goals_first_list[i:i + 3] for i in
                                  range(0, len(self._robot_goals_first_list), 3)]

        self._pub = ProxyPublisher()

    def execute(self, userdata):
        return 'success'

    def on_enter(self, userdata):
        self._pub.createPublisher("/robot_info", RobotGoalsArray)
        goal_msg = RobotGoalsArray()
        goal_msg.header.frame_id = "map"
        goal_msg.header.stamp = rospy.Time.now()
        goal_msg.header.seq = 0
        for i in range(len(self._robot_names_list)):
            goal_msg.robots[i].robot_name = self._robot_names_list[i]
            goal_msg.robots[i].destinations[0].position.x = self._robot_goals_list[i][0]
            goal_msg.robots[i].destinations[0].position.y = self._robot_goals_list[i][1]
            goal_msg.robots[i].destinations[0].position.z = 0.0
            goal_msg.robots[i].destinations[0].orientation.x = 0.0
            goal_msg.robots[i].destinations[0].orientation.y = 0.0
            goal_msg.robots[i].destinations[0].orientation.z = 0.0
            goal_msg.robots[i].destinations[0].orientation.w = self._robot_goals_list[i][2]
        self._pub.publish("/robot_info", goal_msg)

    def on_exit(self, userdata):
        pass

    def on_stop(self):
        pass

