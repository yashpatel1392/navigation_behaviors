#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from navigation_flexbe_states.move_base_flex_action_state import MoveBaseFlexActionState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Thu Aug 18 2022
@author: yashp
'''
class mbfnavSM(Behavior):
	'''
	mbfnav
	'''


	def __init__(self):
		super(mbfnavSM, self).__init__()
		self.name = 'mbfnav'

		# parameters of this behavior
		self.add_parameter('robot', '')
		self.add_parameter('anotherobot', '')

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:489 y:55, x:479 y:320
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]

		# x:30 y:365, x:130 y:365, x:230 y:365, x:330 y:365, x:430 y:365, x:530 y:365
		_sm_container_0 = ConcurrencyContainer(outcomes=['finished', 'failed'], conditions=[
										('finished', [('mover', 'success')]),
										('failed', [('mover', 'failed')]),
										('finished', [('anothermbfmover', 'success')]),
										('failed', [('anothermbfmover', 'failed')])
										])

		with _sm_container_0:
			# x:78 y:176
			OperatableStateMachine.add('mover',
										MoveBaseFlexActionState(robot_name=self.robot),
										transitions={'success': 'finished', 'failed': 'failed'},
										autonomy={'success': Autonomy.Off, 'failed': Autonomy.Off})

			# x:304 y:112
			OperatableStateMachine.add('anothermbfmover',
										MoveBaseFlexActionState(robot_name=self.anotherobot),
										transitions={'success': 'finished', 'failed': 'failed'},
										autonomy={'success': Autonomy.Off, 'failed': Autonomy.Off})



		with _state_machine:
			# x:132 y:187
			OperatableStateMachine.add('Container',
										_sm_container_0,
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
