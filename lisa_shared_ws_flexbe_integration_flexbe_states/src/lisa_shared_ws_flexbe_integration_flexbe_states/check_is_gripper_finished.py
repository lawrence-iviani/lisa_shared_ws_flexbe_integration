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
	From a a list of states and an input gripper verify the whole screws in the gripper are properly  screwer
	Can return done (all the screws are finished)

	># gripper_id	int 		The gripper that has to be executed
	># screw_status_list	list	List of available screws in the gripper (if all screws in all gripper is passed an error is raised) 

	<= done 		All screws are properly screwed
	<= screws_available	Some screws are available (status inserted)
	<= operator_check	No more scews are availabvle but not all the screws are properly done
	'''

	def __init__(self, number_part_screws=4):
		super(CheckGripperIsFinished, self).__init__(
			outcomes=['done', 'screws_available', 'operator_check' ,'error'],
			input_keys=['gripper_id','screw_status_list'],)
		self._number_part_screws = number_part_screws
		self._screw_list = []

	def execute(self, userdata):
		if len(self._screw_list) != self._number_part_screws:
			Logger.logerr("CheckGripperIsFinished: Screw list length is different ({}) than expected ({})".format(self._screw_list,  self._number_part_screws))
			return 'error'
		if  len(self._screw_list) > 0:
			_done = [] # Screw recognized as done
			_todo = [] # Screw recognized as not yet done
			_with_issue = [] # An error, gone or wrong torque
			_missing = []  # screw is recognized as missing
			_first_screw_id = self._selected_grippper * self._number_part_screws
			for ss in enumerate(self._screw_list):
				n = ss[0]+_first_screw_id
				s = ss[1]
				Logger.logdebug("CheckGripperIsFinished:  id {} state is {}".format(n,s))
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
					Logger.logerr("CheckGripperIsFinished: Exit, screw ("+str(n)+"), the status "+str(s)+" is not recognized")
					return 'error'
			Logger.loginfo("CheckGripperIsFinished: Found the following\nDone {}\nTodo {}\nWith probelms={}\n missing={}".format(
					_done, _todo, _with_issue, _missing ))
			if len(_done) == self._number_part_screws:
				Logger.loginfo("CheckGripperIsFinished: Exit done {}".format(_done))
				return 'done'
			elif len(_todo) > 0:
				Logger.loginfo("CheckGripperIsFinished: Exit screws_available {}".format(_todo))
				return 'screws_available'
			else:
				Logger.loginfo("CheckGripperIsFinished: Exit operator_check {}".format(_with_issue))
				return 'operator_check'

	def on_enter(self, userdata):
		self._screw_list = userdata.screw_status_list
		self._selected_grippper = userdata.gripper_id
		Logger.loginfo("CheckGripperIsFinished:on_enter received gripper_id={} and screws list:{}".format(self._selected_grippper, self._screw_list))

