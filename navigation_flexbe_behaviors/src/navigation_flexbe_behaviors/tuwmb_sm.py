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
from navigation_flexbe_states.pause import PauseState
from navigation_flexbe_states.tuw_oa import TuwMBOA
from navigation_flexbe_states.tuw_state import TuwState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on @author: yashp
@author: yashp
'''
class TUWMBSM(Behavior):
	'''
	TUWMB
	'''


	def __init__(self):
		super(TUWMBSM, self).__init__()
		self.name = 'TUWMB'

		# parameters of this behavior
		self.add_parameter('robot_names', 'pioneer, pioneer_bot')
		self.add_parameter('robot_goals', '5.0, 0.0, 1.0, 5.0, 8.0, 1.0')
		self.add_parameter('topic', 'continue')
		self.add_parameter('true', True)
		self.add_parameter('false', False)
		self.add_parameter('num_reps', 5)
		self.add_parameter('robot_start', '0.0, 0.0, 1.0, 0.0, 8.0, 1.0')

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

		# x:541 y:343, x:318 y:297, x:230 y:365, x:132 y:299, x:430 y:365
		_sm_reset_0 = ConcurrencyContainer(outcomes=['finished', 'failed'], conditions=[
										('failed', [('ResetGoalSender', 'failed'), ('ResetMovers', 'failed')]),
										('finished', [('ResetGoalSender', 'success')]),
										('finished', [('ResetMovers', 'success')])
										])

		with _sm_reset_0:
			# x:68 y:112
			OperatableStateMachine.add('ResetMovers',
										TuwMBOA(robot_names=self.robot_names, robot_goals=self.robot_start),
										transitions={'success': 'finished', 'failed': 'failed'},
										autonomy={'success': Autonomy.Off, 'failed': Autonomy.Off})

			# x:470 y:126
			OperatableStateMachine.add('ResetGoalSender',
										TuwState(robot_names=self.robot_names, robot_goals=self.robot_start),
										transitions={'success': 'finished', 'failed': 'failed'},
										autonomy={'success': Autonomy.Off, 'failed': Autonomy.Off})


		# x:68 y:274, x:228 y:228, x:486 y:164, x:330 y:365, x:430 y:365
		_sm_navigation_1 = ConcurrencyContainer(outcomes=['finished', 'failed'], conditions=[
										('finished', [('NavGoalSender', 'success')]),
										('failed', [('NavGoalSender', 'failed'), ('NavGoalMovers', 'failed')]),
										('finished', [('NavGoalMovers', 'success')])
										])

		with _sm_navigation_1:
			# x:65 y:97
			OperatableStateMachine.add('NavGoalSender',
										TuwState(robot_names=self.robot_names, robot_goals=self.robot_goals),
										transitions={'success': 'finished', 'failed': 'failed'},
										autonomy={'success': Autonomy.Off, 'failed': Autonomy.Off})

			# x:343 y:51
			OperatableStateMachine.add('NavGoalMovers',
										TuwMBOA(robot_names=self.robot_names, robot_goals=self.robot_goals),
										transitions={'success': 'finished', 'failed': 'failed'},
										autonomy={'success': Autonomy.Off, 'failed': Autonomy.Off})



		with _state_machine:
			# x:83 y:214
			OperatableStateMachine.add('Counter',
										CounterState(decrement=self.false),
										transitions={'success': 'Navigation', 'failed': 'failed', 'end': 'finished'},
										autonomy={'success': Autonomy.Off, 'failed': Autonomy.Off, 'end': Autonomy.Off},
										remapping={'num_reps': 'counter2_OUT', 'num_reps_remaining': 'counter1_OUT'})

			# x:267 y:356
			OperatableStateMachine.add('DecrementReps',
										CounterState(decrement=self.true),
										transitions={'success': 'Counter', 'failed': 'failed', 'end': 'finished'},
										autonomy={'success': Autonomy.Off, 'failed': Autonomy.Off, 'end': Autonomy.Off},
										remapping={'num_reps': 'counter1_OUT', 'num_reps_remaining': 'counter2_OUT'})

			# x:355 y:44
			OperatableStateMachine.add('Navigation',
										_sm_navigation_1,
										transitions={'finished': 'Pause', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:686 y:85
			OperatableStateMachine.add('Pause',
										PauseState(topic=self.topic),
										transitions={'success': 'Reset'},
										autonomy={'success': Autonomy.Off})

			# x:479 y:471
			OperatableStateMachine.add('Pause2',
										PauseState(topic=self.topic),
										transitions={'success': 'DecrementReps'},
										autonomy={'success': Autonomy.Off})

			# x:716 y:278
			OperatableStateMachine.add('Reset',
										_sm_reset_0,
										transitions={'finished': 'Pause2', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
