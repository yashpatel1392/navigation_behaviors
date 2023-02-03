#!/usr/bin/env python

from flexbe_core import EventState
from flexbe_core.proxy import ProxyPublisher 
from geometry_msgs.msg import PoseStamped


class GoalPublisherState(EventState):
    """
    This states takes in a list of robot names and robot goals, where the goals are published to
    a particular topic (<robot_name>/goal_pose) with the message type as geometry_msgs::PoseStamped.

    -- robot_names      string      robot namespaces.
    -- robot_goals      string      robot goals

    <= success                      indicates successful completion of navigation.
    <= failed                       indicates unsuccessful completion of navigation.

    """

    def __init__(self, robot_names, robot_goals):
        # Declare outcomes, input_keys, and output_keys by calling the super constructor with the corresponding arguments.

        super(GoalPublisherState, self).__init__(outcomes=['success', 'failed'])
        self._robot_names_list = robot_names.split(", ")
        self._robot_goals_first_list = [float(i) for i in robot_goals.split(", ")]
        self._robot_goals_list = [self._robot_goals_first_list[i:i + 3] for i in range(0, len(self._robot_goals_first_list), 3)]
        self._pub = ProxyPublisher()


    def execute(self, userdata):
        # This method is called periodically while the state is active.
        # Main purpose is to check state conditions and trigger a corresponding outcome.
        # If no outcome is returned, the state will stay active.

        # This state will always return success
        return 'success'


    def on_enter(self, userdata):
        # This method is called when the state becomes active, i.e. a transition from another state to this one is taken.

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
            self._pub.publish(current_topic, goal_pose)
            

    def on_exit(self, userdata):
        # This method is called when an outcome is returned and another state gets active.
        pass # Nothing to do here


    def on_stop(self):
        # This method is called whenever the behavior stops execution, also if it is cancelled.
        pass # Nothing to do here
