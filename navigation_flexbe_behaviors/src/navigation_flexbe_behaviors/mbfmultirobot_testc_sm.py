#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from navigation_flexbe_states.counter import CounterState
from navigation_flexbe_states.mbf_movers import MBFMovers
from navigation_flexbe_states.move_base_flex_action_state import MoveBaseFlexActionState
from navigation_flexbe_states.pause import PauseState
from navigation_flexbe_states.tuw_goal_sender_mthread import TuwGoalPublisherStateMThread
from navigation_flexbe_states.tuw_state import TuwState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Wed Dec 21 2022
@author: yashp
'''
class MBFMultiRobot_TestCSM(Behavior):
	'''
	MBFMultiRobot_TestC
	'''


	def __init__(self):
		super(MBFMultiRobot_TestCSM, self).__init__()
		self.name = 'MBFMultiRobot_TestC'

		# parameters of this behavior
		self.add_parameter('robot_names', 'pioneer, pioneer_bot')
		self.add_parameter('robot_goals', '5.0, 0.0, 1.0, 5.0, 8.0, 1.0')
		self.add_parameter('topic', 'continue')
		self.add_parameter('true', True)
		self.add_parameter('false', False)
		self.add_parameter('num_reps', 5)
		self.add_parameter('robot_start', '0.0, 0.0, 1.0, 0.0, 8.0, 1.0')
		self.add_parameter('r1', 'pioneer')
		self.add_parameter('r2', 'pioneer_bot')

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:89 y:559, x:537 y:227
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.counter2_OUT = self.num_reps

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]

		# x:338 y:299, x:554 y:313, x:266 y:430, x:91 y:333, x:430 y:365
		_sm_tuwreset_0 = ConcurrencyContainer(outcomes=['finished', 'failed'], output_keys=['robot_reset_paths'], conditions=[
										('finished', [('GoalPublisher', 'success'), ('GoalSender', 'success')]),
										('failed', [('GoalSender', 'failed')]),
										('failed', [('GoalPublisher', 'failed')])
										])

		with _sm_tuwreset_0:
			# x:69 y:129
			OperatableStateMachine.add('GoalPublisher',
										TuwGoalPublisherStateMThread(robot_names=self.robot_names),
										transitions={'success': 'finished', 'failed': 'failed'},
										autonomy={'success': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'robot_paths_OUT': 'robot_reset_paths'})

			# x:484 y:115
			OperatableStateMachine.add('GoalSender',
										TuwState(robot_names=self.robot_names, robot_goals=self.robot_start),
										transitions={'success': 'finished', 'failed': 'failed'},
										autonomy={'success': Autonomy.Off, 'failed': Autonomy.Off})


		# x:465 y:298, x:731 y:373, x:230 y:365, x:48 y:279, x:430 y:365
		_sm_container_1 = ConcurrencyContainer(outcomes=['finished', 'failed'], conditions=[
										('finished', [('Mover1', 'success'), ('Mover2', 'success')]),
										('failed', [('Mover2', 'failed')]),
										('failed', [('Mover1', 'failed')])
										])

		with _sm_container_1:
			# x:90 y:109
			OperatableStateMachine.add('Mover1',
										MoveBaseFlexActionState(robot_name=self.r1),
										transitions={'success': 'finished', 'failed': 'failed'},
										autonomy={'success': Autonomy.Off, 'failed': Autonomy.Off})

			# x:699 y:89
			OperatableStateMachine.add('Mover2',
										MoveBaseFlexActionState(robot_name=self.r2),
										transitions={'success': 'finished', 'failed': 'failed'},
										autonomy={'success': Autonomy.Off, 'failed': Autonomy.Off})



		with _state_machine:
			# x:83 y:214
			OperatableStateMachine.add('Counter',
										CounterState(decrement=self.false),
										transitions={'success': 'Container', 'failed': 'failed', 'end': 'finished'},
										autonomy={'success': Autonomy.Off, 'failed': Autonomy.Off, 'end': Autonomy.Off},
										remapping={'num_reps': 'counter2_OUT', 'num_reps_remaining': 'counter1_OUT'})

			# x:415 y:421
			OperatableStateMachine.add('DecrementReps',
										CounterState(decrement=self.true),
										transitions={'success': 'Counter', 'failed': 'failed', 'end': 'finished'},
										autonomy={'success': Autonomy.Off, 'failed': Autonomy.Off, 'end': Autonomy.Off},
										remapping={'num_reps': 'counter1_OUT', 'num_reps_remaining': 'counter2_OUT'})

			# x:945 y:117
			OperatableStateMachine.add('Pause',
										PauseState(topic=self.topic),
										transitions={'success': 'TUWReset'},
										autonomy={'success': Autonomy.Off})

			# x:702 y:396
			OperatableStateMachine.add('ResetMBFMovers',
										MBFMovers(robot_names=self.robot_names),
										transitions={'success': 'DecrementReps', 'failed': 'failed'},
										autonomy={'success': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'robot_paths_IN': 'robot_reset_paths'})

			# x:964 y:352
			OperatableStateMachine.add('TUWReset',
										_sm_tuwreset_0,
										transitions={'finished': 'ResetMBFMovers', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'robot_reset_paths': 'robot_reset_paths'})

			# x:579 y:69
			OperatableStateMachine.add('Container',
										_sm_container_1,
										transitions={'finished': 'Pause', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
