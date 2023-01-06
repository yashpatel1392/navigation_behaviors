#!/usr/bin/env python

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient, ProxySubscriberCached, ProxyPublisher

from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import *
from geometry_msgs.msg import *
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Bool

import rospy

# ># waypoint     Pose2D      goal coordinates for the robot.

class PhyMoveBaseState(EventState):
    """
    -- robot_name   string      robot namespace.

    <= success                  indicates successful completion of navigation.
    <= failed                   indicates unsuccessful completion of navigation.

    """

    def __init__(self, robot_name):
        super(PhyMoveBaseState, self).__init__(outcomes=['success', 'failed'],
                                            input_keys=['goal'])
        
        self._robot_name = robot_name
        # self._x = x
        # self._y = y
        # self._theta = theta
        
        Logger.loginfo("%s" % str(self._robot_name))

        self._map_pose_topic = self._robot_name + "/map_pose"
        self._sub = ProxySubscriberCached({self._map_pose_topic: PoseStamped})
        self._action_topic = self._robot_name + "_move_base"
        
        Logger.loginfo("\n%s" % str(self._action_topic))
        self._client = ProxyActionClient({self._action_topic: MoveBaseAction})
        self._status_topic = self._robot_name + "/test_status"

        self._goal_pose_data = PoseStamped()

    def execute(self, userdata):
        print("topic: ", self._map_pose_topic)
        
        print ("in_execute")
        
        if self._sub.has_msg(self._map_pose_topic):
            print("topic (from if block): ", self._map_pose_topic)
            self._map_pose_data = self._sub.get_last_msg(self._map_pose_topic)
            self._sub.remove_last_msg(self._map_pose_topic)

        print("topic (out of if block): ", self._map_pose_topic)

        self._positions_match = False
        
        print ("odom Y: %f" % (self._map_pose_data.pose.position.y))
        print ("goal Y: %f" % (self._goal_pose_data.pose.position.y))

        orient = self._map_pose_data.pose.orientation
        (_, _, yaw) = euler_from_quaternion([orient.x,orient.y,orient.z,orient.w])
        print("Yaw:",yaw)
        
        
        g_orient = self._goal_pose_data.pose.orientation
        (_, _, gyaw) = euler_from_quaternion([g_orient.x,g_orient.y,g_orient.z,g_orient.w])
        print("Goal Yaw:",gyaw)
        
        if all([abs(self._map_pose_data.pose.position.x - self._goal_pose_data.pose.position.x) < 0.5,
            abs(self._map_pose_data.pose.position.y - self._goal_pose_data.pose.position.y) < 0.5,
            abs(yaw - gyaw) < 0.09]):
                Logger.loginfo("Positions matched, goal is reached!")
                self._positions_match = True
        
        if self._positions_match and self._client.has_result(self._action_topic):
            status = self._client.get_state(self._action_topic)
            if status == GoalStatus.SUCCEEDED:
                return 'success'
        
            elif status in [GoalStatus.PREEMPTED, GoalStatus.REJECTED, GoalStatus.ABORTED]:
                return 'failed'

        print ("out of execute")

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
