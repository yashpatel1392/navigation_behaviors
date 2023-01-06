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
from navigation_flexbe_states.physical_move_base import PhyMoveBaseState
from navigation_flexbe_states.start_loggers import Loggers
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Tue Dec 20 2022
@author: yashp
'''
class SinglePioneerwithoutPauseSM(Behavior):
	'''
	PhysPioneerMB
	'''


	def __init__(self):
		super(SinglePioneerwithoutPauseSM, self).__init__()
		self.name = 'SinglePioneer (without Pause)'

		# parameters of this behavior
		self.add_parameter('robot_name', 'pioneer_bot')
		self.add_parameter('robot_start', '0.14, -2.75, 1.0')
		self.add_parameter('robot_goal', '0.2, -5.96, 1.0')
		self.add_parameter('true', True)
		self.add_parameter('false', False)
		self.add_parameter('num_reps', 5)

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:42 y:886, x:673 y:372
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.counter2_OUT = self.num_reps

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:16 y:307
			OperatableStateMachine.add('Counter',
										CounterState(decrement=self.false),
										transitions={'success': 'ActivateLogger', 'failed': 'failed', 'end': 'finished'},
										autonomy={'success': Autonomy.Off, 'failed': Autonomy.Off, 'end': Autonomy.Off},
										remapping={'num_reps': 'counter2_OUT', 'num_reps_remaining': 'counter1_OUT'})

			# x:922 y:69
			OperatableStateMachine.add('DeactivateLogger',
										Loggers(robot_names=self.robot_name, activate=self.false),
										transitions={'success': 'ResetActivateLogger', 'failed': 'failed'},
										autonomy={'success': Autonomy.Off, 'failed': Autonomy.Off})

			# x:241 y:486
			OperatableStateMachine.add('DecrementReps',
										CounterState(decrement=self.true),
										transitions={'success': 'Counter', 'failed': 'failed', 'end': 'finished'},
										autonomy={'success': Autonomy.Off, 'failed': Autonomy.Off, 'end': Autonomy.Off},
										remapping={'num_reps': 'counter1_OUT', 'num_reps_remaining': 'counter2_OUT'})

			# x:412 y:21
			OperatableStateMachine.add('GoalPub',
										GoalPublisherMB(robot_names=self.robot_name, robot_goals=self.robot_goal),
										transitions={'success': 'MoveBase', 'failed': 'failed'},
										autonomy={'success': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'goals': 'nav_goal'})

			# x:630 y:68
			OperatableStateMachine.add('MoveBase',
										PhyMoveBaseState(robot_name=self.robot_name),
										transitions={'success': 'DeactivateLogger', 'failed': 'failed'},
										autonomy={'success': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'goal': 'nav_goal'})

			# x:1143 y:288
			OperatableStateMachine.add('ResetActivateLogger',
										Loggers(robot_names=self.robot_name, activate=self.true),
										transitions={'success': 'ResetGoalPub', 'failed': 'failed'},
										autonomy={'success': Autonomy.Off, 'failed': Autonomy.Off})

			# x:637 y:583
			OperatableStateMachine.add('ResetDeactivateLogger',
										Loggers(robot_names=self.robot_name, activate=self.false),
										transitions={'success': 'DecrementReps', 'failed': 'failed'},
										autonomy={'success': Autonomy.Off, 'failed': Autonomy.Off})

			# x:1153 y:474
			OperatableStateMachine.add('ResetGoalPub',
										GoalPublisherMB(robot_names=self.robot_name, robot_goals=self.robot_start),
										transitions={'success': 'ResetMoveBase', 'failed': 'failed'},
										autonomy={'success': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'goals': 'reset_goal'})

			# x:869 y:603
			OperatableStateMachine.add('ResetMoveBase',
										PhyMoveBaseState(robot_name=self.robot_name),
										transitions={'success': 'ResetDeactivateLogger', 'failed': 'failed'},
										autonomy={'success': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'goal': 'reset_goal'})

			# x:195 y:69
			OperatableStateMachine.add('ActivateLogger',
										Loggers(robot_names=self.robot_name, activate=self.true),
										transitions={'success': 'GoalPub', 'failed': 'failed'},
										autonomy={'success': Autonomy.Off, 'failed': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
