#!/usr/bin/env python

from flexbe_core import EventState
from flexbe_core.proxy import ProxySubscriberCached
import rostopic, roslib
import time, os

# following imports are needed because the input topic could be of any of the following types
from nav_msgs.msg import Path
from flexbe_msgs.msg import BEStatus
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import String


class CommandLauncher(EventState):
    """
    This state is used for launching command lines. If the topic_name parameter is left
    empty, then this state would exit 5 seconds after lauching the command. If the topic
    name is specified then this state would wait for a message to be published to that 
    topic, after the command is launched.

    -- command_to_run       string      command_to_run.
    -- topic_name           string      topic name.

    <= success                          indicates successful completion of navigation.
    <= failed                           indicates unsuccessful completion of navigation.

    """

    def __init__(self, command_to_run, topic_name):
        # Declare outcomes, input_keys, and output_keys by calling the super constructor with the corresponding arguments.

        super(CommandLauncher, self).__init__(outcomes=['success', 'failed'])
        self._command = command_to_run + " &"
        self._topic_name = topic_name


    def execute(self, userdata):
        # This method is called periodically while the state is active.
        # Main purpose is to check state conditions and trigger a corresponding outcome.
        # If no outcome is returned, the state will stay active.

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
            time.sleep(5) # can be removed/reduced after testing
        return 'success'

        
    def on_enter(self, userdata):
        # This method is called when the state becomes active, i.e. a transition from another state to this one is taken.
        os.system(self._command)


    def on_exit(self, userdata):
        # This method is called when an outcome is returned and another state gets active.
        pass # Nothing to do here


    def on_stop(self):
        # This method is called whenever the behavior stops execution, also if it is cancelled.
        pass # Nothing to do here
