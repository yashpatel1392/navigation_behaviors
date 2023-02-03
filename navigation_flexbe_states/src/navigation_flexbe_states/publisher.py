#!/usr/bin/env python

from flexbe_core import EventState
from flexbe_core.proxy import ProxyPublisher
from std_msgs.msg import Bool


class PublisherState(EventState):
    """
    This state publishes a std_msgs::Bool message (True) to the topic which is passed 
    in as a parameter.

    -- topic_name   string      topic name.

    <= success                  indicates successful completion of navigation.
    <= failed                   indicates unsuccessful completion of navigation.

    """

    def __init__(self, topic_name):
        # Declare outcomes, input_keys, and output_keys by calling the super constructor with the corresponding arguments.

        super(PublisherState, self).__init__(outcomes=['success', 'failed'])
        self._topic_name = topic_name
        self._pub = ProxyPublisher({self._topic_name: Bool})


    def execute(self, userdata):
        # This method is called periodically while the state is active.
        # Main purpose is to check state conditions and trigger a corresponding outcome.
        # If no outcome is returned, the state will stay active.

        return 'success'


    def on_enter(self, userdata):
        # This method is called when the state becomes active, i.e. a transition from another state to this one is taken.

        goal_msg = Bool()
        goal_msg.data = True
        self._pub.publish(self._topic_name, goal_msg)


    def on_exit(self, userdata):
        # This method is called when an outcome is returned and another state gets active.
        pass # Nothing to do here


    def on_stop(self):
        # This method is called whenever the behavior stops execution, also if it is cancelled.
        pass # Nothing to do here
