#!/usr/bin/env python

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient, ProxySubscriberCached, ProxyPublisher

from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import *
from nav_msgs.msg import Path, Odometry
from mbf_msgs.msg import ExePathAction
from mbf_msgs.msg import ExePathGoal
from mbf_msgs.msg import MoveBaseAction

from std_msgs.msg import String

import rospy, time

# ># waypoint     Pose2D      goal coordinates for the robot.

class PublisherState(EventState):
    """
    -- topic_name   string      topic name.
    
    <= success                  indicates successful completion of navigation.
    <= failed                   indicates unsuccessful completion of navigation.

    """

    def __init__(self, topic_name):
        super(PublisherState, self).__init__(outcomes=['success', 'failed'])
        self._topic_name = topic_name
        self._pub = ProxyPublisher()

    def execute(self, userdata):
        return 'success'

    def on_enter(self, userdata):
        time.sleep(10)
        self._pub.createPublisher(self._topic_name, String)
        goal_msg = String()
        goal_msg.data = "Publishing to " + self._topic_name
        self._pub.publish(self._topic_name, goal_msg)

    def on_exit(self, userdata):
        pass

    def on_stop(self):
        pass