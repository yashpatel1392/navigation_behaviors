#!/usr/bin/env python

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxySubscriberCached
from nav_msgs.msg import Path

import rospy


class TuwGoalPublisherState(EventState):
    """
    -- robot_names   string      robot namespaces.

    #> robot_paths   list of paths  this is the list of robot paths

    <= success                  indicates successful completion of navigation.
    <= failed                   indicates unsuccessful completion of navigation.

    """

    def __init__(self, robot_names):
        super(TuwGoalPublisherState, self).__init__(outcomes=['success', 'failed'],
                                                 output_keys=['robot_paths_OUT'])
        self._robot_names_list = robot_names.split(", ")
        self._robot_paths = []
        self._robot_path_topics = []
        for i in range(len(self._robot_names_list)):
            self._robot_path_topics.append(self._robot_names_list[i] + "/path")
        
        self._robot_subs_dict = dict.fromkeys(self._robot_path_topics, Path)
            
        self._sub = ProxySubscriberCached(self._robot_subs_dict)
        self._map = {}

    def check_if_all_true(self, dict, robot_names_list):
        print("-------------------- In check_if_all_true()")

        counter = 0
        i = 0
        for i in range(len(robot_names_list)):
            if dict[robot_names_list[i]] == True:
                counter += 1
        return True if counter == i else False

    def execute(self, userdata):
        print("-------------------- In on_enter()")
        while not (self.check_if_all_true(self._map, self._robot_names_list)):
            for i in range(len(self._robot_names_list)):
                if self._sub.has_msg(self._robot_names_list[i] + "/path"):
                    self._map[self._robot_names_list[i]] = True
                    self._robot_paths.append(self._sub.get_last_msg(self._robot_names_list[i] + "/path"))

        userdata.robot_paths_OUT = self._robot_paths

        return 'success'

    def on_enter(self, userdata):
        print("-------------------- In on_enter()")
        for i in range(len(self._robot_names_list)):
            # self._sub(self._robot_names_list[i] + "/path", Path)
            self._map[self._robot_names_list[i]] = False

    def on_exit(self, userdata):
        print("Number of paths sent out are: %f" % (len(self._robot_paths)))

    def on_stop(self):
        pass
