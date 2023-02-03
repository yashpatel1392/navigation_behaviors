#!/usr/bin/env python

from flexbe_core import EventState


class CounterState(EventState):
    """
    This state is used for automating multiple repetitions of the navigation test. 
    Decrement boolean paramter is set to False when a navigation test repetition
    is successful, and therefore, reducing the number of repetitions remaining by 1.

    ># num_reps             int      robot namespaces.

    #> num_reps_remaining   int      this is the list of robot paths

    <= success                       indicates successful completion of navigation.
    <= failed                        indicates unsuccessful completion of navigation.

    """

    def __init__(self, decrement):
        # Declare outcomes, input_keys, and output_keys by calling the super constructor with the corresponding arguments.

        super(CounterState, self).__init__(outcomes=['success', 'failed', 'end'],
                                            input_keys=['num_reps'],
                                            output_keys=['num_reps_remaining'])
        self._reps_remaining = 0
        self._last_rep = False
        self._dec = decrement


    def execute(self, userdata):
        # This method is called periodically while the state is active.
        # Main purpose is to check state conditions and trigger a corresponding outcome.
        # If no outcome is returned, the state will stay active.

        userdata.num_reps_remaining = self._reps_remaining

        if self._last_rep == False:
            return 'success'
        else:
            return 'end'


    def on_enter(self, userdata):
        # This method is called when the state becomes active, i.e. a transition from another state to this one is taken.

        if self._dec == True:
            self._reps_remaining = userdata.num_reps - 1
            if self._reps_remaining == 0:
                self._last_rep = True
        else:
            self._reps_remaining = userdata.num_reps 


    def on_exit(self, userdata):
        # This method is called when an outcome is returned and another state gets active.
        pass # Nothing to do here


    def on_stop(self):
        # This method is called whenever the behavior stops execution, also if it is cancelled.
        pass # Nothing to do here
