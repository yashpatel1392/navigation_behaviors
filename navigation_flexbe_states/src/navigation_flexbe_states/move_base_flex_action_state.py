#!/usr/bin/env python

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient, ProxySubscriberCached

from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from mbf_msgs.msg import ExePathAction
from mbf_msgs.msg import ExePathGoal
from tf.transformations import euler_from_quaternion


class MoveBaseFlexActionState(EventState):
    """
    This state can be used by a single robot to navigate to its goal using move base flex.

    -- robot_name   string      robot namespace.

    <= success                  indicates successful completion of navigation.
    <= failed                   indicates unsuccessful completion of navigation.

    """

    def __init__(self, robot_name):
        # Declare outcomes, input_keys, and output_keys by calling the super constructor with the corresponding arguments.

        super(MoveBaseFlexActionState, self).__init__(outcomes=['success', 'failed'])
        self._robot_name = robot_name
        self._path_topic = self._robot_name + "/path"
        self._map_pose_topic = self._robot_name + "/map_pose"
        self._goal_pose_topic = self._robot_name + "/goal_pose"
        self._sub = ProxySubscriberCached({self._path_topic: Path, self._map_pose_topic: PoseStamped, self._goal_pose_topic: PoseStamped})
        self._action_topic = self._robot_name + "_move_base_flex/exe_path"
        self._client = ProxyActionClient({self._action_topic: ExePathAction})
        

    def execute(self, userdata):
        # This method is called periodically while the state is active.
        # Main purpose is to check state conditions and trigger a corresponding outcome.
        # If no outcome is returned, the state will stay active.

        # Checker 
        if self._sub.has_msg(self._map_pose_topic):
            self._map_pose_data = self._sub.get_last_msg(self._map_pose_topic)
            self._sub.remove_last_msg(self._map_pose_topic)        
            
        if self._sub.has_msg(self._goal_pose_topic):
            self._goal_pose_data = self._sub.get_last_msg(self._goal_pose_topic)
            self._sub.remove_last_msg(self._goal_pose_topic)

        orient = self._map_pose_data.pose.orientation
        (_, _, current_yaw) = euler_from_quaternion([orient.x,orient.y,orient.z,orient.w])       
        g_orient = self._goal_pose_data.pose.orientation
        (_, _, goal_yaw) = euler_from_quaternion([g_orient.x,g_orient.y,g_orient.z,g_orient.w])

        self._positions_match = False
                
        if all([abs(self._map_pose_data.pose.position.x - self._goal_pose_data.pose.position.x) < 0.5,
            abs(self._map_pose_data.pose.position.y - self._goal_pose_data.pose.position.y) < 0.5,
            abs(current_yaw - goal_yaw) < 0.09]):
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

        while True:                
            if self._sub.has_msg(self._path_topic):
                goal = ExePathGoal()
                goal.controller = "TrajectoryPlannerROS"
                goal.path = self._sub.get_last_msg(self._path_topic)
                self._client.send_goal(self._action_topic, goal)
                break
        

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
