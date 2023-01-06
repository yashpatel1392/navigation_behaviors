#!/usr/bin/env python

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient, ProxySubscriberCached, ProxyPublisher

from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import *
from nav_msgs.msg import Path, Odometry
from mbf_msgs.msg import ExePathAction
from mbf_msgs.msg import ExePathGoal
from mbf_msgs.msg import MoveBaseAction

from std_msgs.msg import String, Bool

import rospy, time

# ># waypoint     Pose2D      goal coordinates for the robot.

class Loggers(EventState):
    """
    -- topic_name   string      topic name.
    
    <= success                  indicates successful completion of navigation.
    <= failed                   indicates unsuccessful completion of navigation.

    """

    def __init__(self, robot_names, activate):
        super(Loggers, self).__init__(outcomes=['success', 'failed'])

        self._robot_names_list = robot_names.split(", ")
        self._activate = activate
        self._status_topics = []
        for name in self._robot_names_list:
            self._status_topics.append(name + "/test_status")
        self._status_pub_dict = dict.fromkeys(self._status_topics, Bool)
        self._status_pub = ProxyPublisher(self._status_pub_dict)

    def execute(self, userdata):
        # time.sleep(5)
        return 'success'

    def on_enter(self, userdata):
        for status_topic in self._status_topics:
            goal_msg = Bool()
            goal_msg.data = self._activate
            self._status_pub.publish(status_topic, goal_msg)
        

    def on_exit(self, userdata):
        pass

    def on_stop(self):
        pass