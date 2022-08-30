#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from navigation_flexbe_states.move_base_action_state import MoveBaseActionState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Thu Aug 18 2022
@author: yashp
'''
class navigationSM(Behavior):
	'''
	navigation
	'''


	def __init__(self):
		super(navigationSM, self).__init__()
		self.name = 'navigation'

		# parameters of this behavior
		self.add_parameter('xpos', 0)
		self.add_parameter('ypos', 0)
		self.add_parameter('wpos', 0)
		self.add_parameter('robot', '')

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:388 y:323, x:388 y:56
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:89 y:144
			OperatableStateMachine.add('mover',
										MoveBaseActionState(robot_name=self.robot, x=self.xpos, y=self.ypos, theta=self.wpos),
										transitions={'success': 'finished', 'failed': 'failed'},
										autonomy={'success': Autonomy.Off, 'failed': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
