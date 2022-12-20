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
from navigation_flexbe_states.pause import PauseState
from navigation_flexbe_states.physical_move_base import PhyMoveBaseState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Tue Dec 20 2022
@author: yashp
'''
class PhysPioneerMBSM(Behavior):
	'''
	PhysPioneerMB
	'''


	def __init__(self):
		super(PhysPioneerMBSM, self).__init__()
		self.name = 'PhysPioneerMB'

		# parameters of this behavior
		self.add_parameter('robot_name', 'pioneer_bot')
		self.add_parameter('robot_start', '0.2, -0.083, 1.0')
		self.add_parameter('robot_goal', '2.2, -0.083, 1.0')
		self.add_parameter('pause_topic', 'continue')
		self.add_parameter('true', True)
		self.add_parameter('false', False)
		self.add_parameter('num_reps', 0)

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
			# x:37 y:317
			OperatableStateMachine.add('Counter',
										CounterState(decrement=self.false),
										transitions={'success': 'GoalPub', 'failed': 'failed', 'end': 'finished'},
										autonomy={'success': Autonomy.Off, 'failed': Autonomy.Off, 'end': Autonomy.Off},
										remapping={'num_reps': 'counter2_OUT', 'num_reps_remaining': 'counter1_OUT'})

			# x:288 y:590
			OperatableStateMachine.add('DecrementReps',
										CounterState(decrement=self.true),
										transitions={'success': 'Counter', 'failed': 'failed', 'end': 'finished'},
										autonomy={'success': Autonomy.Off, 'failed': Autonomy.Off, 'end': Autonomy.Off},
										remapping={'num_reps': 'counter1_OUT', 'num_reps_remaining': 'counter2_OUT'})

			# x:285 y:159
			OperatableStateMachine.add('GoalPub',
										GoalPublisherMB(robot_names=self.robot_name, robot_goals=self.robot_goal),
										transitions={'success': 'MoveBase', 'failed': 'failed'},
										autonomy={'success': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'goals': 'nav_goal'})

			# x:630 y:68
			OperatableStateMachine.add('MoveBase',
										PhyMoveBaseState(robot_name=self.robot_name),
										transitions={'success': 'Pause1', 'failed': 'failed'},
										autonomy={'success': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'goal': 'nav_goal'})

			# x:967 y:143
			OperatableStateMachine.add('Pause1',
										PauseState(topic=self.pause_topic),
										transitions={'success': 'ResetGoalPub'},
										autonomy={'success': Autonomy.Off})

			# x:631 y:729
			OperatableStateMachine.add('Pause2',
										PauseState(topic=self.pause_topic),
										transitions={'success': 'DecrementReps'},
										autonomy={'success': Autonomy.Off})

			# x:1157 y:330
			OperatableStateMachine.add('ResetGoalPub',
										GoalPublisherMB(robot_names=self.robot_name, robot_goals=self.robot_start),
										transitions={'success': 'ResetMoveBase', 'failed': 'failed'},
										autonomy={'success': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'goals': 'reset_goal'})

			# x:958 y:572
			OperatableStateMachine.add('ResetMoveBase',
										PhyMoveBaseState(robot_name=self.robot_name),
										transitions={'success': 'Pause2', 'failed': 'failed'},
										autonomy={'success': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'goal': 'reset_goal'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
