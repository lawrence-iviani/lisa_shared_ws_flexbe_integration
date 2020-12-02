#!/usr/bin/env python	
from flexbe_core import EventState, Logger

# import required service
from shared_workspace_msgs.srv import ResetScrew #SetScrew
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
			Logger.loginfo('Waiting for service /reset_screw	')
			reset_screw = rospy.ServiceProxy('/reset_screw', ResetScrew)
			gripper_id = int(self._screw_id/self._number_part_screws) 
			first_screw_id = gripper_id * self._number_part_screws
			Logger.logdebug('ResetScrewStatesInGripper: going to reset gripper {}'.format(gripper_id))
			resp = []
			for ss in enumerate(self._screw_list):
				n = ss[0]
				s = ss[1]
				_abs_id = first_screw_id + n
				Logger.loginfo("ResetScrewStatesInGripper: Relative id {} state is {}".format(n+first_screw_id,s))
				if s not in COMPLETED_SCREW:
					Logger.logdebug("ResetScrewStatesInGripper:  NOT SCREWED!! id {} reset it".format(n+first_screw_id,s))
					resp.append( (reset_screw(_abs_id),n+first_screw_id,s) )
				else:
					Logger.logdebug("ResetScrewStatesInGripper:  ALREADY SCREWED!! id {}: let it untouched".format(n+first_screw_id,s))
			if len(resp)==0:
				userdata.result = True
				userdata.reason = 'all screws are screwed'
				Logger.loginfo('ResetScrewStatesInGripper: EXECUTE done nothing_to_do')
				return 'nothing_to_do'
			elif all(resp):
				userdata.result = True
				userdata.reason = 'success'
			else:	
				userdata.result = False
				userdata.reason = 'a problem in resetting the screw\'s gripper'
			Logger.loginfo('ResetScrewStatesInGripper: EXECUTE done succeeded, {}'.format(resp))
			return 'succeeded'
		except rospy.ServiceException, e:
			userdata.reason = str(e)
			Logger.logwarn('ResetScrewStatesInGripper: EXECUTE done aborted, reason is: '+str(e))
			return 'aborted'

	def on_enter(self, userdata):
		self._screw_list = userdata.screw_status_list
		self._screw_id = userdata.screw_id


