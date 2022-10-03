#!/usr/bin/env python

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxySubscriberCached
from nav_msgs.msg import Path

import rospy


class GoalPublisherState(EventState):
    """
    -- robot_names   string      robot namespaces.

    #> robot_paths   list of paths  this is the list of robot paths

    <= success                  indicates successful completion of navigation.
    <= failed                   indicates unsuccessful completion of navigation.

    """

    def __init__(self, robot_names):
        super(GoalPublisherState, self).__init__(outcomes=['success', 'failed'],
                                                 output_keys=['robot_paths'])
        self._robot_names_list = robot_names.split(", ")
        self._robot_paths = []
        self._sub = ProxySubscriberCached()
        self._map = {}

    def check_if_all_true(self, dict, robot_names_list):
        counter = 0
        i = 0
        for i in range(len(robot_names_list)):
            if dict[robot_names_list[i]] == True:
                counter += 1
        return True if counter == i else False

    def execute(self, userdata):
        while not (check_if_all_true(self._map, self._robot_names_list)):
            for i in range(len(self._robot_names_list)):
                if self._sub.has_msg(self._robot_names_list[i] + "/path"):
                    self._map[self._robot_names_list[i]] = True
                    self._robot_paths.append(self._sub.get_last_msg(self._robot_names_list[i] + "/path"))

        userdata.robot_paths = self._robot_paths

        return 'success'

    def on_enter(self, userdata):
        for i in range(len(self._robot_names_list)):
            self._sub(self._robot_names_list[i] + "/path", Path)
            self._map[self._robot_names_list[i]] = False

    def on_exit(self, userdata):
        pass

    def on_stop(self):
        pass
