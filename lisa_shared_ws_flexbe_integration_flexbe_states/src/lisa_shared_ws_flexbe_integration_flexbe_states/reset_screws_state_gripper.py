#!/usr/bin/env python
from flexbe_core import EventState, Logger

# import required service
from shared_workspace_msgs.srv import SetScrew
import rospy


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
            outcomes=['succeeded', 'aborted'],
            input_keys=['screw_id'],)
         #   output_keys=['result'])

        #self._service_namespace = service_namespace
        # rospy.wait_for_service(self._service_namespace)
        self._number_part_screws = number_part_screws#
	Logger.loginfo('INIT')

	def execute(self, userdata):
		return 'succeeded'
		try:
			Logger.loginfo('Watining for service /set_screw	')
			set_screw = rospy.ServiceProxy('/set_screw', SetScrew)
			gripper_id = int(self._screw_id/self._number_part_screws)
			first_screw_id = gripper_id * self._number_part_screws
			Logger.loginfo('going to reset gripper {}'.format(gripper_id))
			for s in range(self._number_part_screws):
				_abs_id = first_screw_id + s
				resp = set_screw(_abs_id, False, False)
				Logger.loginfo('Screw state updated absolute_id={}, screw({}) gripper(){}'.format(_abs_id, s, gripper_id))
			userdata.result = resp.result
			return 'succeeded'
		except rospy.ServiceException, e:
			Logger.loginfo("Service call failed: %s" % e)
			return 'aborted'

	def on_enter(self, userdata):
		Logger.loginfo('ENTER')
		self._screw_id = userdata.screw_id
