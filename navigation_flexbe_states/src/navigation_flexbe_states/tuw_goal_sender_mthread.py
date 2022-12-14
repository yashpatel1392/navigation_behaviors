#!/usr/bin/env python

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxySubscriberCached
from nav_msgs.msg import Path

import rospy
import threading

class TuwGoalPublisherStateMThread(EventState):
    """
    -- robot_names   string      robot namespaces.

    #> robot_paths   list of paths  this is the list of robot paths

    <= success                  indicates successful completion of navigation.
    <= failed                   indicates unsuccessful completion of navigation.

    """

    def __init__(self, robot_names):
        super(TuwGoalPublisherStateMThread, self).__init__(outcomes=['success', 'failed'],
                                                 output_keys=['robot_paths_OUT'])
        self._robot_names_list = robot_names.split(", ")
        self._robot_paths = []
        self._robot_path_topics = []
                
        for i in range(len(self._robot_names_list)):
            self._robot_path_topics.append(self._robot_names_list[i] + "/path")
        
        self._robot_subs_dict = dict.fromkeys(self._robot_path_topics, Path)
        self._threads_list = []
        
        self._paths_dict = {}
        
        self._sub = ProxySubscriberCached(self._robot_subs_dict)

    def execute(self, userdata):
        print("-------------------- In execute()")
        print("Number of threads: %f" % (len(self._threads_list)))
        
        for thread in self._threads_list:
            thread.join()
        
        print("------------------ out of execute()")
        
        # userdata.robot_paths_OUT = self._robot_paths
        userdata.robot_paths_OUT = self._paths_dict

        
        return 'success'
    
    def is_path_received(self, robot_name):
        printed = False
        while True:
            if not printed: 
                print("=-_________=-________________", robot_name)
                printed = True
                
            if self._sub.has_msg(robot_name + "/path"):
                path = self._sub.get_last_msg(robot_name + "/path")
                self._robot_paths.append(path)
                
                temp_str = robot_name + "_move_base_flex/exe_path"                
                self._paths_dict[temp_str] = path
                self._sub.remove_last_msg(robot_name + "/path")
                return

    def on_enter(self, userdata):
        print("-------------------- In on_enter()")
        
        if len(self._threads_list) != 0:
            # self._threads_list.clear() 
            self._threads_list = []       
        
        for i in range(len(self._robot_names_list)):
            print("Robot Name: ", self._robot_names_list[i])
            thread = threading.Thread(target=self.is_path_received, args=(self._robot_names_list[i],))
            if not thread.is_alive():
                self._threads_list.append(thread)
                self._threads_list[i].start()


    def on_exit(self, userdata):
        print("Number of paths sent out are: %f" % (len(self._robot_paths)))
        print("Number of paths sent out are: %f" % (len(userdata.robot_paths_OUT)))
        
        # for thread in self._threads_list:
        #     thread.exit()

    def on_stop(self):
        pass
