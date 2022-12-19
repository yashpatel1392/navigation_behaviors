#!/usr/bin/env python

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxySubscriberCached
from nav_msgs.msg import Path

import rospy
import threading

class CounterState(EventState):
    """
    ># num_reps             int      robot namespaces.

    #> num_reps_remaining   int      this is the list of robot paths

    <= success                  indicates successful completion of navigation.
    <= failed                   indicates unsuccessful completion of navigation.

    """

    def __init__(self, decrement):
        super(CounterState, self).__init__(outcomes=['success', 'failed', 'end'],
                                           input_keys=['num_reps'],
                                           output_keys=['num_reps_remaining'])
        self._reps_remaining = 0
        self._last_rep = False
        self._dec = decrement
    
    def execute(self, userdata):
        userdata.num_reps_remaining = self._reps_remaining
        if self._last_rep == False:
            return 'success'
        else:
            return 'end'
    
    def on_enter(self, userdata):
        if self._dec == True:
            self._reps_remaining = userdata.num_reps - 1
            
            if self._reps_remaining == 0:
                self._last_rep = True
        else:
            self._reps_remaining = userdata.num_reps 


    def on_exit(self, userdata):
        print("Number of repetitions: %f" % (userdata.num_reps))
        print("Number of repetitions remaining: %f" % (userdata.num_reps_remaining))
        pass

    def on_stop(self):
        pass
