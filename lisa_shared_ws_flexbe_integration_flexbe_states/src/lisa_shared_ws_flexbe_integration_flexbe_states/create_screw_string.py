#!/usr/bin/env python
import rospy

from flexbe_core import EventState#, Logger


LOOKUP_GRIPPER = {0: 'Part A', 1: 'Part B', 2: 'Part C', 3: 'Part D' }

class LisaMotekConcatenateScrewPartString(EventState):
	'''
	Compose a string with a proper screw gripper identifier.

	-- text_format 	string 	The format string to utter, it must contains the position for the screw id
	-- number_part_screws 	int	The number of screws into one gripper (default is 4)
	#> screw_id  	    int 		supposed to be an int, in any case a str conversion is performed
	#< text_to_utter    string 		the formatted text to utter

	<= done 			The message is formatted

	'''

	def __init__(self, text_format="Screw ID is {}", number_part_screws=4):
		super(LisaMotekConcatenateScrewPartString, self).__init__(
					outcomes=['done'],
					input_keys=['screw_id'],
					output_keys=['text_to_utter'])
		self._text = text_format
		self._number_part_screws = number_part_screws

	def execute(self, userdata):
		return 'done'

	def on_enter(self, userdata):
		# int gripper = userdata.screw_id/m_number_part_screws;
		gripper_id = int(userdata.screw_id/self._number_part_screws)
		gripper_name = LOOKUP_GRIPPER[gripper_id] if gripper_id in LOOKUP_GRIPPER else gripper_id
		text = "Screw {} in Gripper {}".format(userdata.screw_id, gripper_name)
		userdata.text_to_utter = self._text.format(text)

	def on_exit(self, userdata):
		pass

	def on_start(self):
		pass

	def on_stop(self):
		pass 
		
