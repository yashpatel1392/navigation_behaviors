#!/usr/bin/env python

from flexbe_core import EventState
from flexbe_core.proxy import ProxySubscriberCached, ProxyPublisher
from std_msgs.msg import String


class PauseState(EventState):
    """
    This state continues running until a "continue" message is published to the 
    topic, whose name is passed as an input parameter. 

    -- topic        string      name to topic, which has to be subscibed.

    <= success                  indicates successful completion of navigation.
    <= failed                   indicates unsuccessful completion of navigation.

    """

    def __init__(self, topic):
        # Declare outcomes, input_keys, and output_keys by calling the super constructor with the corresponding arguments.

        super(PauseState, self).__init__(outcomes=['success'])
        self._topic = topic
        self._sub = ProxySubscriberCached({self._topic: String})
        self._pub = ProxyPublisher({self._topic: String})


    def execute(self, userdata):
        # This method is called periodically while the state is active.
        # Main purpose is to check state conditions and trigger a corresponding outcome.
        # If no outcome is returned, the state will stay active.

        if self._topic:
            if self._sub.has_msg(self._topic):
                self._data = self._sub.get_last_msg(self._topic)
                if self._data.data == "continue":
                    return 'success'
        

    def on_enter(self, userdata):
        # This method is called when the state becomes active, i.e. a transition from another state to this one is taken.
        pass # Nothing to do here


    def on_exit(self, userdata):
        # This method is called when an outcome is returned and another state gets active.

        pause_msg = String()
        pause_msg.data = "pause"
        self._pub.publish(self._topic, pause_msg)
        pass


    def on_stop(self):
        # This method is called whenever the behavior stops execution, also if it is cancelled.

        pause_msg = String()
        pause_msg.data = "pause"
        self._pub.publish(self._topic, pause_msg)
        pass
