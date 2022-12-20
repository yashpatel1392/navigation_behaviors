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
class SimMultiRobotSM(Behavior):
	'''
	SimMultiRobot
	'''


	def __init__(self):
		super(SimMultiRobotSM, self).__init__()
		self.name = 'SimMultiRobot'

		# parameters of this behavior
		self.add_parameter('robot1_name', 'pioneer')
		self.add_parameter('robot2_name', 'pioneer_bot')
		self.add_parameter('robot1_start', '0.0, 0.0, 1.0')
		self.add_parameter('robot2_start', '0.0, 8.0, 1.0')
		self.add_parameter('robot1_goal', '5.0, 0.0, 1.0')
		self.add_parameter('robot2_goal', '0.0, 20.0, 1.0')
		self.add_parameter('true', True)
		self.add_parameter('false', False)
		self.add_parameter('num_reps', 0)
		self.add_parameter('topic', 'continue')

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:32 y:743, x:566 y:294
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.counter2_OUT = self.num_reps

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]

		# x:837 y:345, x:130 y:451
		_sm_container_2_0 = OperatableStateMachine(outcomes=['finished', 'failed'])

		with _sm_container_2_0:
			# x:95 y:196
			OperatableStateMachine.add('GoalPub_R2',
										GoalPublisherMB(robot_names=self.robot2_name, robot_goals=self.robot2_goal),
										transitions={'success': 'MoveBase_R2', 'failed': 'failed'},
										autonomy={'success': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'goals': 'r2_goal'})

			# x:409 y:196
			OperatableStateMachine.add('MoveBase_R2',
										MoveBaseState(robot_name=self.robot2_name),
										transitions={'success': 'finished', 'failed': 'failed'},
										autonomy={'success': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'goal': 'r2_goal'})


		# x:798 y:427, x:306 y:449
		_sm_container_1 = OperatableStateMachine(outcomes=['finished', 'failed'])

		with _sm_container_1:
			# x:101 y:152
			OperatableStateMachine.add('GoalPub_R1',
										GoalPublisherMB(robot_names=self.robot1_name, robot_goals=self.robot1_goal),
										transitions={'success': 'MoveBase_R1', 'failed': 'failed'},
										autonomy={'success': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'goals': 'r1_goal'})

			# x:491 y:132
			OperatableStateMachine.add('MoveBase_R1',
										MoveBaseState(robot_name=self.robot1_name),
										transitions={'success': 'finished', 'failed': 'failed'},
										autonomy={'success': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'goal': 'r1_goal'})


		# x:472 y:341, x:1064 y:103, x:1071 y:392, x:465 y:69, x:451 y:596, x:584 y:646
		_sm_container_2 = ConcurrencyContainer(outcomes=['finished', 'failed'], conditions=[
										('finished', [('Container', 'finished')]),
										('finished', [('Container_2', 'finished')]),
										('failed', [('Container', 'failed')]),
										('failed', [('Container_2', 'failed')])
										])

		with _sm_container_2:
			# x:170 y:165
			OperatableStateMachine.add('Container',
										_sm_container_1,
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:711 y:170
			OperatableStateMachine.add('Container_2',
										_sm_container_2_0,
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})



		with _state_machine:
			# x:64 y:303
			OperatableStateMachine.add('Counter',
										CounterState(decrement=self.false),
										transitions={'success': 'Container', 'failed': 'failed', 'end': 'finished'},
										autonomy={'success': Autonomy.Off, 'failed': Autonomy.Off, 'end': Autonomy.Off},
										remapping={'num_reps': 'counter2_OUT', 'num_reps_remaining': 'counter1_OUT'})

			# x:689 y:650
			OperatableStateMachine.add('DecrementReps',
										CounterState(decrement=self.true),
										transitions={'success': 'Counter', 'failed': 'failed', 'end': 'finished'},
										autonomy={'success': Autonomy.Off, 'failed': Autonomy.Off, 'end': Autonomy.Off},
										remapping={'num_reps': 'counter1_OUT', 'num_reps_remaining': 'counter2_OUT'})

			# x:817 y:297
			OperatableStateMachine.add('Pause1',
										PauseState(topic=self.topic),
										transitions={'success': 'DecrementReps'},
										autonomy={'success': Autonomy.Off})

			# x:378 y:116
			OperatableStateMachine.add('Container',
										_sm_container_2,
										transitions={'finished': 'Pause1', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
