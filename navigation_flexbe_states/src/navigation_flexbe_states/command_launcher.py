#!/usr/bin/env python

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxySubscriberCached
from nav_msgs.msg import Path
from flexbe_msgs.msg import BEStatus
from gazebo_msgs.msg import ModelStates

from std_msgs.msg import String


import rospy
import rostopic, roslib
import time, os

class CommandLauncher(EventState):
    """
    ># command_to_run             string      command_to_run.
    ># topic_name                 string      topic name.

    <= success                  indicates successful completion of navigation.
    <= failed                   indicates unsuccessful completion of navigation.

    """

    def __init__(self, command_to_run, topic_name):
        super(CommandLauncher, self).__init__(outcomes=['success', 'failed'])
        self._command = command_to_run + " &"
        self._topic_name = topic_name

    
    def execute(self, userdata):
        if self._topic_name:
            sub_created = False
            while True:
                if sub_created:
                    if self._sub.has_msg(self._topic_name):
                        break        

                topic_type = rostopic.get_topic_type(self._topic_name, blocking=False)
                
                if not sub_created and topic_type[0] != None:            
                    data_class = roslib.message.get_message_class(topic_type[0])
                    self._sub = ProxySubscriberCached({self._topic_name: data_class})
                    sub_created = True
        else:
            time.sleep(5)
        
        return 'success'
        
    def on_enter(self, userdata):
        os.system(self._command)
        pass

    def on_exit(self, userdata):
        pass

    def on_stop(self):
        pass
