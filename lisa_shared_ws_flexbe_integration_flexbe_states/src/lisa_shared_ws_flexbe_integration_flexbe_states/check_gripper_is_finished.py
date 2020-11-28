#!/usr/bin/env python
from flexbe_core import EventState, Logger

# import required service
from shared_workspace_msgs.srv import SetScrew
import rospy

# enum from https://ids-git.fzi.de/manipulation/ros_shared_workspace_demo/-/blob/master/shared_workspace/src/GripperManager.cpp
SCREW_STATUS = {0: 'empty', 1: 'inserted', 2: 'screwed', 3: 'wrong_torque', 4: 'gone' }
SCREW_EMPTY = SCREW_STATUS[0] 	 	# the screw is not available
SCREW_INSERTED = SCREW_STATUS[1]  	# the screw is available in the gripper but not not yet screwed
SCREW_SCREWED = SCREW_STATUS[2]		# the screw is in the final position and done (screwed)
SCREW_WRONG_TORQUE = SCREW_STATUS[3]	# a wrong torque situation (the screw is not correctly in place)
SCREW_GONE = SCREW_STATUS[4]		# this condition raise when the screw is inserted but not found. At the moment it seems there is a bug in insertmotek (all the screws are signed as gone because screw_found is always false)



# I wanna return, complete, missing screw, check_screws
class CheckGripperIsFinished(EventState):
	'''
	From a a list of states and an input gripper verify the whole screws in the gripper are 

	># screw_id	int 	Screw to update

	<= succeeded 		All good.
	<= aborted		    The service returns an error.

	'''

   	def __init__(self, number_part_screws=4):
		# See example_state.py for basic explanations.	
		super(CheckGripperIsFinished, self).__init__(
			outcomes=['done', 'screws_available', 'operator_check' ,'error'],
			input_keys=['screw_status_list'],)
		self._number_part_screws = number_part_screws
		self._screw_list = []

	def execute(self, userdata):
		if len(self._screw_list) != self._number_part_screws:
			Logger.logerr("Screw list length is different ({}) than expected ({})".format(self._screw_list,  self._number_part_screws))
			return 'error'
		if  len(self._screw_list) > 0:
			_done = [] # Screw recognized as done
			_todo = [] # Screw recognized as not yet done
			_with_issue = [] # An error, gone or wrong torque
			_missing = []  # screw is recognized as missing
			for ss in enumerate(self._screw_list):
				n = ss[0]
				s = ss[1]
				Logger.loginfo("Relative id {} state is {}".format(n,s))
				if s == SCREW_SCREWED:
					_done.append(n)
				elif s == SCREW_EMPTY:
					_missing.append(n)
				elif s == SCREW_INSERTED:
					_todo.append(n)
				elif s == SCREW_WRONG_TORQUE:
					_with_issue.append(n)
				elif s == SCREW_GONE:
					_with_issue.append(n)
				else:
					Logger.logerr("Exit, screw ("+str(n)+"), the status "+str(s)+" is not recognized")
					return 'error'
			Logger.loginfo("Found the following\nDone {}\nTodo {}\nWith probelms={}\n missing={}".format(
					_done, _todo, _with_issue, _missing ))
			if len(_done) == self._number_part_screws:
				Logger.loginfo("Exit done")
				return 'done'
			elif len(_todo) > 0:
				Logger.loginfo("Exit screws_available")
				return 'screws_available'
			else:
				Logger.loginfo("Exit operator_check")
				return 'operator_check'

	def on_enter(self, userdata):
		self._screw_list = userdata.screw_status_list
		Logger.loginfo("received screws list:\n{}".format(self._screw_list))

