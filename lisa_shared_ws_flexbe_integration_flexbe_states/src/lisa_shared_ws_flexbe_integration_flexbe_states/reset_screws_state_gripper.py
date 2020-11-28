#!/usr/bin/env python
from flexbe_core import EventState, Logger

# import required service
from shared_workspace_msgs.srv import SetScrew
import rospy


# from enum from https://ids-git.fzi.de/manipulation/ros_shared_workspace_demo/-/blob/master/shared_workspace
# and as used in CheckGripperIsFinished
# so, could be 2 (enumearaion value or the label screwed).
# all the other has to be considered as incomplete
COMPLETED_SCREW = [2, 'screwed']


class ResetScrewStatesInGripper(EventState):
	'''
	Call to the service that update the state of the input screw as screwed

	># screw_id	int 	Screw to update

	<= succeeded 		All good.
	<= aborted		    The service returns an error.

	'''

   	def __init__(self, number_part_screws=4):
		# See example_state.py for basic explanations.
		super(ResetScrewStatesInGripper, self).__init__(
		outcomes=['succeeded', 'aborted', 'nothing_to_do'],
		input_keys=['screw_id', 'screw_status_list'],
		output_keys=['result', 'reason'],)
		self._number_part_screws = number_part_screws

	def execute(self, userdata):
		try:
			Logger.loginfo('Waiting for service /set_screw	')
			set_screw = rospy.ServiceProxy('/set_screw', SetScrew)
			gripper_id = int(self._screw_id/self._number_part_screws) 
			first_screw_id = gripper_id * self._number_part_screws
			Logger.loginfo('going to reset gripper {}'.format(gripper_id))
			resp = []
			for ss in enumerate(self._screw_list):
				n = ss[0]
				s = ss[1]
				_abs_id = first_screw_id + n
				Logger.loginfo("Relative id {} state is {}".format(n,s))
				if s not in COMPLETED_SCREW:
					resp.append(set_screw(_abs_id, False, False))

			if len(resp)==0:
				userdata.result = True
				userdata.reason = 'all screws are screwed'
				return 'nothing_to_do'
			elif all(resp):
				userdata.result = True
				userdata.reason = 'success'
			else:	
				userdata.result = False
				userdata.reason = 'a problem in resetting the screw\'s gripper'
			Logger.loginfo('EXECUTE done succeeded')
			return 'succeeded'
		except rospy.ServiceException, e:
			Logger.loginfo("Service call failed: %s" % e)
			userdata.reason = str(e)
			Logger.loginfo('EXECUTE done aborted')
			return 'aborted'

	def on_enter(self, userdata):
		self._screw_list = userdata.screw_status_list
		self._screw_id = userdata.screw_id


