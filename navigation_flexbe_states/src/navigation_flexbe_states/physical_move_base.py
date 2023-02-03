#!/usr/bin/env python

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient, ProxySubscriberCached

from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import *
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion

import rospy


class PhyMoveBaseState(EventState):
    """
    This state can be used by a single robot to navigate to its goal using move base.
    It takes in goal position as a PoseStamped, which is received as an input key.
    This state is mainly used by physical robots as it compares the map_pose with its
    goal position to check whether the robot stopped closed to its goal position.

    -- robot_name   string          robot namespace.

    ># goal         PoseStamped     goal coordinates as geometry_msgs::PoseStamped

    <= success                      indicates successful completion of navigation.
    <= failed                       indicates unsuccessful completion of navigation.

    """

    def __init__(self, robot_name):
        # Declare outcomes, input_keys, and output_keys by calling the super constructor with the corresponding arguments.

        super(PhyMoveBaseState, self).__init__(outcomes=['success', 'failed'],
                                            input_keys=['goal'])
        self._robot_name = robot_name
        self._map_pose_topic = self._robot_name + "/map_pose"
        self._sub = ProxySubscriberCached({self._map_pose_topic: PoseStamped})
        self._action_topic = self._robot_name + "_move_base"
        self._client = ProxyActionClient({self._action_topic: MoveBaseAction})
        self._status_topic = self._robot_name + "/test_status"
        self._goal_pose_data = PoseStamped()


    def execute(self, userdata):
        # This method is called periodically while the state is active.
        # Main purpose is to check state conditions and trigger a corresponding outcome.
        # If no outcome is returned, the state will stay active.

        if self._sub.has_msg(self._map_pose_topic):
            self._map_pose_data = self._sub.get_last_msg(self._map_pose_topic)
            self._sub.remove_last_msg(self._map_pose_topic)

        self._positions_match = False

        orient = self._map_pose_data.pose.orientation
        (_, _, yaw) = euler_from_quaternion([orient.x,orient.y,orient.z,orient.w])       
        g_orient = self._goal_pose_data.pose.orientation
        (_, _, gyaw) = euler_from_quaternion([g_orient.x,g_orient.y,g_orient.z,g_orient.w])
        
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


    def on_enter(self, userdata):   
        # This method is called when the state becomes active, i.e. a transition from another state to this one is taken.
                        
        self._goal_pose_data = userdata.goal[0]
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = userdata.goal[0].pose

        self._client.send_goal(self._action_topic, goal)

    # This function cancels active goals for the robot.
    def cancel_active_goals(self):
        if self._client.is_available(self._action_topic):
            if self._client.is_active(self._action_topic):
                if not self._client.has_result(self._action_topic):
                    self._client.cancel(self._action_topic)
                    Logger.loginfo('Cancelled move_base active action goal.')


    def on_exit(self, userdata):
        # This method is called when an outcome is returned and another state gets active.
        self.cancel_active_goals()
        

    def on_stop(self):
        # This method is called whenever the behavior stops execution, also if it is cancelled.
        self.cancel_active_goals()
