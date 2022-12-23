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
from navigation_flexbe_states.move_base_movers import MoveBaseMovers
from navigation_flexbe_states.pause import PauseState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Wed Dec 21 2022
@author: yashp
'''
class MultiRobotSingleMoverMBSM(Behavior):
	'''
	MultiRobotSingleMoverMB
	'''


	def __init__(self):
		super(MultiRobotSingleMoverMBSM, self).__init__()
		self.name = 'MultiRobotSingleMoverMB'

		# parameters of this behavior
		self.add_parameter('robot_names', 'stretch, pioneer_bot')
		self.add_parameter('robot_goals', '2.0, -0.083, 1.0, 2.0, -2.7, 1.0')
		self.add_parameter('topic', 'continue')
		self.add_parameter('true', True)
		self.add_parameter('false', False)
		self.add_parameter('num_reps', 5)
		self.add_parameter('robot_start', '0.2, -0.083, 1.0, 0.15, -2.7, 1.0')

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:66 y:468, x:500 y:217
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.counter2_OUT = self.num_reps

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:83 y:214
			OperatableStateMachine.add('Counter',
										CounterState(decrement=self.false),
										transitions={'success': 'GoalPub', 'failed': 'failed', 'end': 'finished'},
										autonomy={'success': Autonomy.Off, 'failed': Autonomy.Off, 'end': Autonomy.Off},
										remapping={'num_reps': 'counter2_OUT', 'num_reps_remaining': 'counter1_OUT'})

			# x:255 y:365
			OperatableStateMachine.add('DecrementReps',
										CounterState(decrement=self.true),
										transitions={'success': 'Counter', 'failed': 'failed', 'end': 'finished'},
										autonomy={'success': Autonomy.Off, 'failed': Autonomy.Off, 'end': Autonomy.Off},
										remapping={'num_reps': 'counter1_OUT', 'num_reps_remaining': 'counter2_OUT'})

			# x:286 y:31
			OperatableStateMachine.add('GoalPub',
										GoalPublisherMB(robot_names=self.robot_names, robot_goals=self.robot_goals),
										transitions={'success': 'MoveBaseMovers', 'failed': 'failed'},
										autonomy={'success': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'goals': 'robot_goals'})

			# x:588 y:9
			OperatableStateMachine.add('MoveBaseMovers',
										MoveBaseMovers(robot_names=self.robot_names),
										transitions={'success': 'Pause1', 'failed': 'failed'},
										autonomy={'success': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'robot_goals_IN': 'robot_goals'})

			# x:772 y:85
			OperatableStateMachine.add('Pause1',
										PauseState(topic=self.topic),
										transitions={'success': 'ResetGoalPub'},
										autonomy={'success': Autonomy.Off})

			# x:498 y:423
			OperatableStateMachine.add('Pause2',
										PauseState(topic=self.topic),
										transitions={'success': 'DecrementReps'},
										autonomy={'success': Autonomy.Off})

			# x:937 y:206
			OperatableStateMachine.add('ResetGoalPub',
										GoalPublisherMB(robot_names=self.robot_names, robot_goals=self.robot_start),
										transitions={'success': 'ResetMoveBaseMovers', 'failed': 'failed'},
										autonomy={'success': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'goals': 'reset_goals'})

			# x:741 y:359
			OperatableStateMachine.add('ResetMoveBaseMovers',
										MoveBaseMovers(robot_names=self.robot_names),
										transitions={'success': 'Pause2', 'failed': 'failed'},
										autonomy={'success': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'robot_goals_IN': 'reset_goals'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
