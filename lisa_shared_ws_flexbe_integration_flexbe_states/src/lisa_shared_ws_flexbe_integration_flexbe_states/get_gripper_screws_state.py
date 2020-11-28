#!/usr/bin/env python
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyServiceCaller

# import required service
from shared_workspace_msgs.msg import GripperArray
import rospy

# A mnemonic list of names
LOOKUP_GRIPPER = {0: 'Part A', 1: 'Part B', 2: 'Part C', 3: 'Part D' }

# enum from https://ids-git.fzi.de/manipulation/ros_shared_workspace_demo/-/blob/master/shared_workspace/src/GripperManager.cpp
SCREW_STATES = {0: 'empty', 1: 'inserted', 2: 'screwed', 3: 'wrong_torque', 4: 'gone' }
GRIPPER_STATES = {0: 'visible', 1: 'not visible', 2: 'missing', 3: 'position only'};


class GripperStateListener(EventState):
    '''
   Listen for a topic grippers_state (containing the state of the screws of a gripper) and produce an output kezt

    -- input_id_is_screw 	bool	Tell if the input key gripper_id is a screw_id instead
    -- time_out 		int	time out in second to wait the topib to be published
    -- number_part_screws	String 	How many screws per gripper

    <# gripper_id 		int  	the gripper id that has to be invetigated or the screw_id of the gripper (or the screw id if the param input_id_is_screw is set to True

    ># gripper_status		int 	the status of the gripper
    ># list_gripper_screws_status	int[] 	List of the available screws in the gripper with their state
    ># gripper_id		int 	The gripper id detected, this is the same of the input if input_id_is_screw=False otherwise is the deterimined gripper_if from a screw_id
    ># gripper_name		string 	A convenient name lookup for the given id, for uttering mostly
    ># gripper_name		string 	An error message, if any

    <= succeeded 		All good.
    <= aborted		    	An issue in listening to the topic, error_reason is populated

    '''

    def __init__(self, input_id_is_screw=False , time_out = 1.0, number_part_screws = 4):
        # See example_state.py for basic explanations.
        super(GripperStateListener, self).__init__(
            outcomes=['succeeded', 'aborted'],
            input_keys=['gripper_id'],
            output_keys=['gripper_status', 'list_gripper_screws_status', 'gripper_id', 'gripper_name', 'error_reason' ])

        self.topic_name = '/grippers_state'
        self.wait = time_out
        self.error_reason = None
	self.list_gripper_screws_status = None  # if set to None means it was never called
					 # if no screws are avaliable in the gripper a empty list should be returned
	self.input_id_is_screw = input_id_is_screw
	self.number_part_screws = number_part_screws
	# with this flas as True means gripper_id is a screw_id instead, this means I have to calculate the 
	# actual gripper_id from the screw_id. 
		
    def execute(self, userdata):
	elapsed = rospy.get_rostime() - self.start_time
	if self.error_reason is not None:
		Logger.logerr("Exit for an error: " + str(self.error_reason))
        	return 'aborted'
	if self.list_gripper_screws_status is None and elapsed.to_sec() > self.wait:
		Logger.logerr("Exit for time out")
        	return 'aborted'
	# ok, no error, no timeout, i have succeded then
        if self.list_gripper_screws_status is not None:
            return 'succeeded'
        
    def _topic_rcvd(self, data):
	if self.list_gripper_screws_status is None:
	    Logger.logdebug('Subscribed topic rcvd: {}'.format(data))
	    try:
	    	self.list_gripper_screws_status = [SCREW_STATES[ord(d)] for d in data.grippers[self.gripper_id].screws_status]
		self.gripper_status = GRIPPER_STATES[data.grippers[self.gripper_id].status]
		Logger.loginfo('Gripper status is {} - Screws status are {}'.format(self.gripper_status, self.list_gripper_screws_status))
	    except Exception as e:
		self.error_reason = "Error receiving the topic >>>" + str(e) + "<<<"
	else:
	    pass #Logger.logwarn("Topic " + str() + " already called. Unregister ")
	    #self.sub_gripper_state.unregister()

    def on_enter(self, userdata):
	if self.input_id_is_screw:	
		self.gripper_id = userdata.gripper_id // self.number_part_screws
	else:
		self.gripper_id = userdata.gripper_id
	Logger.loginfo('Identified gripper id {}'.format(self.gripper_id))
	try:
   	    Logger.loginfo('Subscribing topic: {}'.format(self.topic_name))
	    self.sub_gripper_state = rospy.Subscriber(self.topic_name, GripperArray, self._topic_rcvd)		
	except Exception as e:
	    self.error_reason = "Error subscribing topic >>>" + str(e) + "<<<"
	self.start_time = rospy.get_rostime()

    def on_exit(self, userdata):
	# saving the output
	try:
            userdata.error_reason = self.error_reason if self.error_reason is not None else ''
	    userdata.list_gripper_screws_status = self.list_gripper_screws_status if self.list_gripper_screws_status is not None else []
	    userdata.gripper_id = self.gripper_id
	    userdata.gripper_name = LOOKUP_GRIPPER[self.gripper_id] # self.gripper_id #
	except Exception as e:
	    Logger.logwarn("Bad, error during exit: " + str(e))
	self.sub_gripper_state.unregister()

