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
class MultiRobotSingleMoverSM(Behavior):
	'''
	MultiRobotSingleMover
	'''


	def __init__(self):
		super(MultiRobotSingleMoverSM, self).__init__()
		self.name = 'MultiRobotSingleMover'

		# parameters of this behavior
		self.add_parameter('robot_names', 'pioneer, pioneer_bot')
		self.add_parameter('robot_goals', '5.0, 0.0, 1.0, 5.0, 8.0, 1.0')
		self.add_parameter('topic', 'continue')
		self.add_parameter('true', True)
		self.add_parameter('false', False)
		self.add_parameter('num_reps', 5)

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:89 y:559, x:500 y:217
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.counter2_OUT = self.num_reps

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]

		# x:30 y:365, x:130 y:365, x:230 y:365, x:330 y:365
		_sm_container_0 = ConcurrencyContainer(outcomes=['finished', 'failed'], output_keys=['robot_goals_OUT'], conditions=[
										('finished', [('GoalPub', 'success')]),
										('failed', [('GoalPub', 'failed')])
										])

		with _sm_container_0:
			# x:177 y:135
			OperatableStateMachine.add('GoalPub',
										GoalPublisherMB(robot_names=self.robot_names, robot_goals=self.robot_goals),
										transitions={'success': 'finished', 'failed': 'failed'},
										autonomy={'success': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'goals': 'robot_goals_OUT'})



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

			# x:675 y:86
			OperatableStateMachine.add('MoveBaseMovers',
										MoveBaseMovers(robot_names=self.robot_names),
										transitions={'success': 'Pause', 'failed': 'failed'},
										autonomy={'success': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'robot_goals_IN': 'robot_goals_OUT'})

			# x:711 y:324
			OperatableStateMachine.add('Pause',
										PauseState(topic=self.topic),
										transitions={'success': 'DecrementReps'},
										autonomy={'success': Autonomy.Off})

			# x:366 y:64
			OperatableStateMachine.add('Container',
										_sm_container_0,
										transitions={'finished': 'MoveBaseMovers', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'robot_goals_OUT': 'robot_goals_OUT'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
