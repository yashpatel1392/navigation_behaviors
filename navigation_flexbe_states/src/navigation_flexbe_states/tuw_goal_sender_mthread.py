#!/usr/bin/env python

from flexbe_core import EventState
from flexbe_core.proxy import ProxySubscriberCached
from nav_msgs.msg import Path
import threading


class TuwGoalPublisherStateMThread(EventState):
    """
    This state subscribes to TUW and saves paths for each of the robots published by TUW, 
    which is then sent as output key. This state should be run parallel to tuw_state.

    -- robot_names          string          list of robot namespaces.

    #> robot_paths_OUT      list of paths   list of robot paths published by TUW.

    <= success                              indicates successful completion of navigation.
    <= failed                               indicates unsuccessful completion of navigation.

    """

    def __init__(self, robot_names):
        # Declare outcomes, input_keys, and output_keys by calling the super constructor with the corresponding arguments.

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
        # This method is called periodically while the state is active.
		# Main purpose is to check state conditions and trigger a corresponding outcome.
		# If no outcome is returned, the state will stay active.
             
        for thread in self._threads_list:
            thread.join()
                
        userdata.robot_paths_OUT = self._paths_dict        
        return 'success'

    # This function is used by the threads for subscribing to TUW to get paths
    def is_path_received(self, robot_name):
        printed = False
        while True:
            if not printed: 
                printed = True
    
            if self._sub.has_msg(robot_name + "/path"):
                path = self._sub.get_last_msg(robot_name + "/path")
                self._robot_paths.append(path)
                temp_str = robot_name + "_move_base_flex/exe_path"                
                self._paths_dict[temp_str] = path
                self._sub.remove_last_msg(robot_name + "/path")
                return


    def on_enter(self, userdata): 
        # This method is called when the state becomes active, i.e. a transition from another state to this one is taken.
     
        self._threads_list = []       
        for i in range(len(self._robot_names_list)):
            thread = threading.Thread(target=self.is_path_received, args=(self._robot_names_list[i],))
            if not thread.is_alive():
                self._threads_list.append(thread)
                self._threads_list[i].start()


    def on_exit(self, userdata):
        # This method is called when an outcome is returned and another state gets active.
        pass # Nothing to do here


    def on_stop(self):
        # This method is called whenever the behavior stops execution, also if it is cancelled.
        pass # Nothing to do here
