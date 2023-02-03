#!/usr/bin/env python

from flexbe_core import EventState
from flexbe_core.proxy import ProxyPublisher
from std_msgs.msg import Bool


class Loggers(EventState):
    """
    This state is used for starting/stopping the loggers. If the activate parameter is
    True, the loggers are started. If the activate parameter is False, the loggers are
    stopped. Loggers are started/stopped by publishing to <robot_name>/test_status topic.

    -- robot_names  string      list of robot names.
    -- activate     bool        boolean for activating/deactivating loggers.

    <= success                  indicates successful completion of navigation.
    <= failed                   indicates unsuccessful completion of navigation.

    """


    def __init__(self, robot_names, activate):
        # Declare outcomes, input_keys, and output_keys by calling the super constructor with the corresponding arguments.

        super(Loggers, self).__init__(outcomes=['success', 'failed'])
        self._robot_names_list = robot_names.split(", ")
        self._activate = activate
        self._status_topics = []
        for name in self._robot_names_list:
            self._status_topics.append(name + "/test_status")
        self._status_pub_dict = dict.fromkeys(self._status_topics, Bool)
        self._status_pub = ProxyPublisher(self._status_pub_dict)


    def execute(self, userdata):
        # This method is called periodically while the state is active.
        # Main purpose is to check state conditions and trigger a corresponding outcome.
        # If no outcome is returned, the state will stay active.
        
        # This state always returns success
        return 'success'


    def on_enter(self, userdata):
        # This method is called when the state becomes active, i.e. a transition from another state to this one is taken.

        for status_topic in self._status_topics:
            goal_msg = Bool()
            goal_msg.data = self._activate
            self._status_pub.publish(status_topic, goal_msg)
        

    def on_exit(self, userdata):
        # This method is called when an outcome is returned and another state gets active.
        pass # Nothing to do here


    def on_stop(self):
        # This method is called whenever the behavior stops execution, also if it is cancelled.
        pass # Nothing to do here
