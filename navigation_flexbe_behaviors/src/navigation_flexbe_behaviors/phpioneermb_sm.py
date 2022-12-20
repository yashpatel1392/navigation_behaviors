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
from navigation_flexbe_states.goal_publisher_mb import GoalPublisherMB
from navigation_flexbe_states.move_base_single import MoveBaseState
from navigation_flexbe_states.pause import PauseState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Tue Dec 20 2022
@author: yashp
'''
class PhPioneerMBSM(Behavior):
	'''
	PhPioneerMB
	'''


	def __init__(self):
		super(PhPioneerMBSM, self).__init__()
		self.name = 'PhPioneerMB'

		# parameters of this behavior
		self.add_parameter('name', 'pioneer')
		self.add_parameter('goal', '0.0, 8.0, 1.0')
		self.add_parameter('topic', 'continue')
		self.add_parameter('bool_true', True)
		self.add_parameter('bool_false', False)
		self.add_parameter('num_reps', 2)
		self.add_parameter('start', '0.0, 0.0, 1.0')

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:30 y:546, x:778 y:378
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.counter2_OUT = self.num_reps

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:90 y:294
			OperatableStateMachine.add('Counter1',
										CounterState(decrement=self.bool_false),
										transitions={'success': 'GoalPub', 'failed': 'failed', 'end': 'finished'},
										autonomy={'success': Autonomy.Off, 'failed': Autonomy.Off, 'end': Autonomy.Off},
										remapping={'num_reps': 'counter2_OUT', 'num_reps_remaining': 'counter1_OUT'})

			# x:441 y:652
			OperatableStateMachine.add('Counter2',
										CounterState(decrement=self.bool_true),
										transitions={'success': 'Counter1', 'failed': 'failed', 'end': 'finished'},
										autonomy={'success': Autonomy.Off, 'failed': Autonomy.Off, 'end': Autonomy.Off},
										remapping={'num_reps': 'counter1_OUT', 'num_reps_remaining': 'counter2_OUT'})

			# x:274 y:97
			OperatableStateMachine.add('GoalPub',
										GoalPublisherMB(robot_names=self.name, robot_goals=self.goal),
										transitions={'success': 'MoveBase', 'failed': 'failed'},
										autonomy={'success': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'goals': 'goals_OUT'})

			# x:598 y:67
			OperatableStateMachine.add('MoveBase',
										MoveBaseState(robot_name=self.name),
										transitions={'success': 'Pause', 'failed': 'failed'},
										autonomy={'success': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'goal': 'goals_OUT'})

			# x:932 y:117
			OperatableStateMachine.add('Pause',
										PauseState(topic=self.topic),
										transitions={'success': 'ResetGoalPub'},
										autonomy={'success': Autonomy.Off})

			# x:846 y:672
			OperatableStateMachine.add('Pause2',
										PauseState(topic=self.topic),
										transitions={'success': 'Counter2'},
										autonomy={'success': Autonomy.Off})

			# x:1442 y:155
			OperatableStateMachine.add('ResetGoalPub',
										GoalPublisherMB(robot_names=self.name, robot_goals=self.start),
										transitions={'success': 'ResetMoveBase', 'failed': 'failed'},
										autonomy={'success': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'goals': 'reset_goals'})

			# x:1201 y:584
			OperatableStateMachine.add('ResetMoveBase',
										MoveBaseState(robot_name=self.name),
										transitions={'success': 'Pause2', 'failed': 'failed'},
										autonomy={'success': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'goal': 'reset_goals'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
