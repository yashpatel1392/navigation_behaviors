#!/usr/bin/env python

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxySubscriberCached, ProxyPublisher
from std_msgs.msg import String

import rospy

class PauseState(EventState):
    """
    ># num_reps             int      robot namespaces.

    #> num_reps_remaining   int      this is the list of robot paths

    <= success                  indicates successful completion of navigation.
    <= failed                   indicates unsuccessful completion of navigation.

    """

    def __init__(self, topic):
        super(PauseState, self).__init__(outcomes=['success'])
        self._topic = topic
        self._sub = ProxySubscriberCached({self._topic: String})
        self._pub = ProxyPublisher({self._topic: String})
    
    def execute(self, userdata):
        if self._topic:
            if self._sub.has_msg(self._topic):
                self._data = self._sub.get_last_msg(self._topic)
                if self._data.data == "continue":
                    return 'success'
       
    
    def on_enter(self, userdata):
        pass


    def on_exit(self, userdata):
        pause_msg = String()
        pause_msg.data = "pause"
        self._pub.publish(self._topic, pause_msg)
        pass

    def on_stop(self):
        pause_msg = String()
        pause_msg.data = "pause"
        self._pub.publish(self._topic, pause_msg)
        pass
