#!/usr/bin/env python

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient, ProxySubscriberCached

from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import *
from geometry_msgs.msg import *
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

import rospy

# ># waypoint     Pose2D      goal coordinates for the robot.

class MoveBaseState(EventState):
    """
    -- robot_name   string      robot namespace.

    <= success                  indicates successful completion of navigation.
    <= failed                   indicates unsuccessful completion of navigation.

    """

    def __init__(self, robot_name):
        super(MoveBaseState, self).__init__(outcomes=['success', 'failed'],
                                            input_keys=['goal'])
        
        self._robot_name = robot_name
        # self._x = x
        # self._y = y
        # self._theta = theta
        
        Logger.loginfo("%s" % str(self._robot_name))

        self._odom_topic = self._robot_name + "/odom"
        self._sub = ProxySubscriberCached({self._odom_topic: Odometry})
        self._action_topic = self._robot_name + "_move_base"
        
        Logger.loginfo("\n%s" % str(self._action_topic))
        self._client = ProxyActionClient({self._action_topic: MoveBaseAction})
        
        self._goal_pose_data = PoseStamped()

    def execute(self, userdata):
        print ("in_execute")
        
        if self._sub.has_msg(self._odom_topic):
            self._odom_data = self._sub.get_last_msg(self._odom_topic)
            self._sub.remove_last_msg(self._odom_topic)
        
        self._positions_match = False
        
        print ("odom Y: %f" % (self._odom_data.pose.pose.position.y))
        print ("goal Y: %f" % (self._goal_pose_data.pose.position.y))

        orient = self._odom_data.pose.pose.orientation
        (_, _, yaw) = euler_from_quaternion([orient.x,orient.y,orient.z,orient.w])
        print("Yaw:",yaw)
        
        
        g_orient = self._goal_pose_data.pose.orientation
        (_, _, gyaw) = euler_from_quaternion([g_orient.x,g_orient.y,g_orient.z,g_orient.w])
        print("Goal Yaw:",gyaw)
        
        if all([abs(self._odom_data.pose.pose.position.x - self._goal_pose_data.pose.position.x) < 0.5,
            abs(self._odom_data.pose.pose.position.y - self._goal_pose_data.pose.position.y) < 0.5,
            abs(yaw - gyaw) < 0.09]):
            # abs(self._odom_data.pose.pose.orientation.w - self._goal_pose_data.pose.orientation.w) < 0.09]):
                Logger.loginfo("Positions matched, goal is reached!")
                self._positions_match = True
        
        if self._positions_match and self._client.has_result(self._action_topic):
            status = self._client.get_state(self._action_topic)
            if status == GoalStatus.SUCCEEDED:
                return 'success'
        
            elif status in [GoalStatus.PREEMPTED, GoalStatus.REJECTED, GoalStatus.ABORTED]:
                return 'failed'

    def on_enter(self, userdata):            
        self._goal_pose_data = userdata.goal[0]
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = userdata.goal[0].pose

        self._client.send_goal(self._action_topic, goal)

    # copied from armada_behaviors
    def cancel_active_goals(self):
        if self._client.is_available(self._action_topic):
            if self._client.is_active(self._action_topic):
                if not self._client.has_result(self._action_topic):
                    self._client.cancel(self._action_topic)
                    Logger.loginfo('Cancelled move_base active action goal.')

    # copied from armada_behaviors
    def on_exit(self, userdata):
        self.cancel_active_goals()
        
    # copied from armada_behaviors
    def on_stop(self):
        self.cancel_active_goals()
