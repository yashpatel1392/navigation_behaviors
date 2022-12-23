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
from navigation_flexbe_states.goal_publisher import GoalPublisherState
from navigation_flexbe_states.move_base_flex_action_state import MoveBaseFlexActionState
from navigation_flexbe_states.pause import PauseState
from navigation_flexbe_states.tuw_state import TuwState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Wed Dec 21 2022
@author: yashp
'''
class MBFMultiRobot_TestDSM(Behavior):
	'''
	MBFMultiRobot_TestD
	'''


	def __init__(self):
		super(MBFMultiRobot_TestDSM, self).__init__()
		self.name = 'MBFMultiRobot_TestD'

		# parameters of this behavior
		self.add_parameter('topic', 'continue')
		self.add_parameter('true', True)
		self.add_parameter('false', False)
		self.add_parameter('num_reps', 5)
		self.add_parameter('robot1_name', 'pioneer')
		self.add_parameter('robot1_start', '0.0, 0.0, 1.0')
		self.add_parameter('robot1_goal', '5.0, 0.0, 1.0')
		self.add_parameter('robot2_name', 'pioneer_bot')
		self.add_parameter('robot2_start', '0.0, 8.0, 1.0')
		self.add_parameter('robot2_goal', '5.0, 8.0, 1.0')

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:89 y:559, x:568 y:221
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.counter2_OUT = self.num_reps

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]

		# x:338 y:299, x:738 y:42, x:56 y:277, x:117 y:608, x:379 y:611, x:894 y:435, x:630 y:479
		_sm_tuwreset_0 = ConcurrencyContainer(outcomes=['finished', 'failed'], output_keys=['robot_reset_paths'], conditions=[
										('finished', [('R1GoalSender', 'success'), ('R1Mover', 'success'), ('R2GoalSender', 'success'), ('R2Mover', 'success')]),
										('failed', [('R1GoalSender', 'failed')]),
										('failed', [('R1Mover', 'failed')]),
										('failed', [('R2Mover', 'failed')]),
										('failed', [('R2GoalSender', 'failed')])
										])

		with _sm_tuwreset_0:
			# x:150 y:132
			OperatableStateMachine.add('R1Mover',
										MoveBaseFlexActionState(robot_name=self.robot1_name),
										transitions={'success': 'finished', 'failed': 'failed'},
										autonomy={'success': Autonomy.Off, 'failed': Autonomy.Off})

			# x:795 y:179
			OperatableStateMachine.add('R2GoalSender',
										TuwState(robot_names=self.robot2_name, robot_goals=self.robot2_start),
										transitions={'success': 'finished', 'failed': 'failed'},
										autonomy={'success': Autonomy.Off, 'failed': Autonomy.Off})

			# x:592 y:399
			OperatableStateMachine.add('R2Mover',
										MoveBaseFlexActionState(robot_name=self.robot2_name),
										transitions={'success': 'finished', 'failed': 'failed'},
										autonomy={'success': Autonomy.Off, 'failed': Autonomy.Off})

			# x:484 y:115
			OperatableStateMachine.add('R1GoalSender',
										TuwState(robot_names=self.robot1_name, robot_goals=self.robot1_start),
										transitions={'success': 'finished', 'failed': 'failed'},
										autonomy={'success': Autonomy.Off, 'failed': Autonomy.Off})


		# x:454 y:184, x:15 y:156, x:911 y:100, x:196 y:326, x:916 y:304, x:852 y:467, x:1037 y:668
		_sm_tuwnav_1 = ConcurrencyContainer(outcomes=['finished', 'failed'], output_keys=['robot_paths'], conditions=[
										('finished', [('GoalSender', 'success'), ('R1Mover', 'success'), ('R2GoalSender', 'success'), ('R2Mover', 'success'), ('R1GoalPub', 'success'), ('R2GoalPub', 'success')]),
										('failed', [('GoalSender', 'failed')]),
										('failed', [('R1GoalPub', 'failed'), ('R2GoalSender', 'failed')]),
										('failed', [('R1Mover', 'failed')]),
										('failed', [('R2Mover', 'failed'), ('R2GoalPub', 'failed')])
										])

		with _sm_tuwnav_1:
			# x:180 y:39
			OperatableStateMachine.add('GoalSender',
										TuwState(robot_names=self.robot1_name, robot_goals=self.robot1_goal),
										transitions={'success': 'finished', 'failed': 'failed'},
										autonomy={'success': Autonomy.Off, 'failed': Autonomy.Off})

			# x:187 y:497
			OperatableStateMachine.add('R1GoalPub',
										GoalPublisherState(robot_names=self.robot1_name, robot_goals=self.robot1_goal),
										transitions={'success': 'finished', 'failed': 'failed'},
										autonomy={'success': Autonomy.Off, 'failed': Autonomy.Off})

			# x:613 y:46
			OperatableStateMachine.add('R1Mover',
										MoveBaseFlexActionState(robot_name=self.robot1_name),
										transitions={'success': 'finished', 'failed': 'failed'},
										autonomy={'success': Autonomy.Off, 'failed': Autonomy.Off})

			# x:623 y:474
			OperatableStateMachine.add('R2GoalPub',
										GoalPublisherState(robot_names=self.robot2_name, robot_goals=self.robot2_goal),
										transitions={'success': 'finished', 'failed': 'failed'},
										autonomy={'success': Autonomy.Off, 'failed': Autonomy.Off})

			# x:151 y:164
			OperatableStateMachine.add('R2GoalSender',
										TuwState(robot_names=self.robot2_name, robot_goals=self.robot2_goal),
										transitions={'success': 'finished', 'failed': 'failed'},
										autonomy={'success': Autonomy.Off, 'failed': Autonomy.Off})

			# x:628 y:178
			OperatableStateMachine.add('R2Mover',
										MoveBaseFlexActionState(robot_name=self.robot2_name),
										transitions={'success': 'finished', 'failed': 'failed'},
										autonomy={'success': Autonomy.Off, 'failed': Autonomy.Off})



		with _state_machine:
			# x:83 y:214
			OperatableStateMachine.add('Counter',
										CounterState(decrement=self.false),
										transitions={'success': 'TUWNav', 'failed': 'failed', 'end': 'finished'},
										autonomy={'success': Autonomy.Off, 'failed': Autonomy.Off, 'end': Autonomy.Off},
										remapping={'num_reps': 'counter2_OUT', 'num_reps_remaining': 'counter1_OUT'})

			# x:415 y:421
			OperatableStateMachine.add('DecrementReps',
										CounterState(decrement=self.true),
										transitions={'success': 'Counter', 'failed': 'failed', 'end': 'finished'},
										autonomy={'success': Autonomy.Off, 'failed': Autonomy.Off, 'end': Autonomy.Off},
										remapping={'num_reps': 'counter1_OUT', 'num_reps_remaining': 'counter2_OUT'})

			# x:728 y:39
			OperatableStateMachine.add('Pause1',
										PauseState(topic=self.topic),
										transitions={'success': 'TUWReset'},
										autonomy={'success': Autonomy.Off})

			# x:746 y:354
			OperatableStateMachine.add('Pause2',
										PauseState(topic=self.topic),
										transitions={'success': 'DecrementReps'},
										autonomy={'success': Autonomy.Off})

			# x:439 y:30
			OperatableStateMachine.add('TUWNav',
										_sm_tuwnav_1,
										transitions={'finished': 'Pause1', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'robot_paths': 'robot_paths'})

			# x:976 y:182
			OperatableStateMachine.add('TUWReset',
										_sm_tuwreset_0,
										transitions={'finished': 'Pause2', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'robot_reset_paths': 'robot_reset_paths'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
