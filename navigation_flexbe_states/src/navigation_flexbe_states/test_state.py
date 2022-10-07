#!/usr/bin/env python

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxySubscriberCached
from nav_msgs.msg import Path

import rospy


class TestState(EventState):
    """
    -- robot_names   string      robot namespaces.

    #> robot_paths   list of paths  this is the list of robot paths

    <= success                  indicates successful completion of navigation.
    <= failed                   indicates unsuccessful completion of navigation.

    """

    def __init__(self):
        super(TestState, self).__init__(outcomes=['success', 'failed'],
                                                 input_keys=['robot_paths_IN'])
        self._robot_path_list = []

    def execute(self, userdata):
        return 'success'

    def on_enter(self, userdata):
        self._robot_path_list = userdata.robot_paths_IN
        print("Length of the paths list is: %f" % (len(self._robot_path_list)))
        print("Length of the paths list is: %f" % (len(userdata.robot_paths_IN)))
        
        
    def on_exit(self, userdata):
        pass

    def on_stop(self):
        pass
