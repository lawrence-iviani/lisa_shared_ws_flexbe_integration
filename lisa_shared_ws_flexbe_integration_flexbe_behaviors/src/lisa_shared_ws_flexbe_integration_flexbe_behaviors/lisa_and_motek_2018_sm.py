#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from flexbe_states.check_condition_state import CheckConditionState
from flexbe_states.log_key_state import LogKeyState
from flexbe_states.wait_state import WaitState
from horse_flexbe_states.part_available_pose_service_state import PartAvailablePoseService
from horse_flexbe_states.set_screw_service_state import SetScrewServiceState
from horse_flexbe_states.shared_ws_action_state import SharedWsActionState
from horse_flexbe_states.shared_ws_new_traj_action_state import SharedWsNewTrajActionState
from lisa_flexbe_states_flexbe_states.lisa_utter_state import LisaUtterState
from lisa_shared_ws_flexbe_integration_flexbe_behaviors.gripper_complete_sm import GripperCompleteSM
from lisa_shared_ws_flexbe_integration_flexbe_states.create_screw_string import LisaMotekConcatenateScrewPartString
from shared_workspace_behaviors.motekinsertscrew_sm import MotekInsertScrewSM
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Nov 20 2020
@author: Lawrence Iviani
'''
class LisaandMotek2018SM(Behavior):
    '''
    Added acoustic communication channel, based on Motek 2018 with localization of parts and online trajectory computation
    '''


    def __init__(self):
        super(LisaandMotek2018SM, self).__init__()
        self.name = 'Lisa and Motek 2018'

        # parameters of this behavior

        # references to used behaviors
        self.add_behavior(GripperCompleteSM, 'Grip Ended Check/Gripper Complete')
        self.add_behavior(MotekInsertScrewSM, 'MotekInsertScrew')

        # semantic properties of this behavior
        self._semantic_properties = {}

        # Additional initialization code can be added inside the following tags
        # [MANUAL_INIT]

        # [/MANUAL_INIT]

        # Behavior comments:



    def create(self):
        exec_speed = 0.80 #was scary at 0.35
        exec_acceleration = 1.00 # was scary at 0.40
        coll_threshold = 10 # was 14 with CVL at first, then 12 before switch to D435
        coll_repeats = 0
        # x:59 y:997, x:159 y:150
        _state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
        _state_machine.userdata.eSELECT_AVAILABLE_TRAJ = 12
        _state_machine.userdata.eEXECUTE_FW = 1
        _state_machine.userdata.eREAD_FROM_DISK = 6
        _state_machine.userdata.eEXECUTE_BW = 2
        _state_machine.userdata.ZERO = 0
        _state_machine.userdata.endeffector_frame = 'tool_screwdriver_tip_link'
        _state_machine.userdata.reference_frame = 'base_link'
        _state_machine.userdata.tf_frame = 'screw_'
        _state_machine.userdata.velocity = 1.0
        _state_machine.userdata.target_offset = []
        _state_machine.userdata.offset_position_to_screw = [0.0 , 0.0, 0.003]
        _state_machine.userdata.offset_position_from_screw = [0.0 , 0.0, -0.022]
        _state_machine.userdata.offset_orientation_jiggle = [0.0, 0.0, 0.2, 1.0]
        _state_machine.userdata.no_offset_position = [0.0, 0.0, 0,0]
        _state_machine.userdata.no_offset_orientation = [0.0, 0.0, 0.0, 1.0]
        _state_machine.userdata.eDRIVE_TO_START = 11
        _state_machine.userdata.start_step = 0
        _state_machine.userdata.eSELECT_AVAILABLE_REPLAN_TRAJ = 14
        _state_machine.userdata.eEXECUTE_REPLAN_TRAJ = 15
        _state_machine.userdata.eEXECUTE_BW_FROM_STEP = 17
        _state_machine.userdata.eEXECUTE_FW_FROM_STEP = 16
        _state_machine.userdata.eCREATE_NEW_TRAJ = 19
        _state_machine.userdata.eEXECUTE_NEW_TRAJ = 20
        _state_machine.userdata.next_trajectory = 0

        # Additional creation code can be added inside the following tags
        # [MANUAL_CREATE]

        # [/MANUAL_CREATE]

        # x:715 y:376, x:1319 y:338, x:811 y:160
        _sm_grip_ended_check_0 = OperatableStateMachine(outcomes=['aborted', 'succeeded', 'fail'], input_keys=['next_trajectory', 'insertion_successful', 'screw_found'], output_keys=['list_available_screw_poses'])

        with _sm_grip_ended_check_0:
            # x:91 y:23
            OperatableStateMachine.add('log_screw_id',
                                        LogKeyState(text="MAIN: insterd screw id {}", severity=Logger.REPORT_HINT),
                                        transitions={'done': 'log_last_insertion'},
                                        autonomy={'done': Autonomy.Off},
                                        remapping={'data': 'next_trajectory'})

            # x:1121 y:514
            OperatableStateMachine.add('Gripper Complete',
                                        self.use_behavior(GripperCompleteSM, 'Grip Ended Check/Gripper Complete'),
                                        transitions={'finished': 'log_answer', 'failed': 'fail', 'max_retry': 'fail'},
                                        autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit, 'max_retry': Autonomy.Inherit},
                                        remapping={'answer': 'answer'})

            # x:1100 y:21
            OperatableStateMachine.add('Update Screw State',
                                        SetScrewServiceState(),
                                        transitions={'succeeded': 'Available Screw In current Gripper', 'aborted': 'fail'},
                                        autonomy={'succeeded': Autonomy.Off, 'aborted': Autonomy.Off},
                                        remapping={'screw_id': 'next_trajectory', 'finished_successful': 'insertion_successful', 'screw_found': 'screw_found', 'result': 'result'})

            # x:393 y:251
            OperatableStateMachine.add('UtterNotInserted',
                                        LisaUtterState(context_id="GripCheck", wait_time=15),
                                        transitions={'done': 'log_screw_found', 'preempt': 'log_screw_found', 'timeouted': 'log_screw_found', 'error': 'fail'},
                                        autonomy={'done': Autonomy.Off, 'preempt': Autonomy.Off, 'timeouted': Autonomy.Off, 'error': Autonomy.Off},
                                        remapping={'text_to_utter': 'text_to_utter', 'error_reason': 'error_reason'})

            # x:432 y:17
            OperatableStateMachine.add('check_insertion',
                                        CheckConditionState(predicate=lambda x: x == True),
                                        transitions={'true': 'log_screw_found', 'false': 'utter_not_inserted_text'},
                                        autonomy={'true': Autonomy.Off, 'false': Autonomy.Off},
                                        remapping={'input_value': 'insertion_successful'})

            # x:907 y:435
            OperatableStateMachine.add('log_answer',
                                        LogKeyState(text="User reply {}", severity=Logger.REPORT_HINT),
                                        transitions={'done': 'aborted'},
                                        autonomy={'done': Autonomy.Off},
                                        remapping={'data': 'answer'})

            # x:273 y:19
            OperatableStateMachine.add('log_last_insertion',
                                        LogKeyState(text="MAIN: last insertion was succesfu? {}", severity=Logger.REPORT_HINT),
                                        transitions={'done': 'check_insertion'},
                                        autonomy={'done': Autonomy.Off},
                                        remapping={'data': 'insertion_successful'})

            # x:674 y:29
            OperatableStateMachine.add('log_screw_found',
                                        LogKeyState(text="MAIN: screw was found ? {}", severity=Logger.REPORT_HINT),
                                        transitions={'done': 'wwww'},
                                        autonomy={'done': Autonomy.Off},
                                        remapping={'data': 'screw_found'})

            # x:124 y:141
            OperatableStateMachine.add('utter_not_inserted_text',
                                        LisaMotekConcatenateScrewPartString(text_format="I couldn't properly insert {}. skipped", number_part_screws=4),
                                        transitions={'done': 'UtterNotInserted'},
                                        autonomy={'done': Autonomy.Off},
                                        remapping={'screw_id': 'next_trajectory', 'text_to_utter': 'text_to_utter'})

            # x:910 y:8
            OperatableStateMachine.add('wwww',
                                        WaitState(wait_time=1),
                                        transitions={'done': 'Update Screw State'},
                                        autonomy={'done': Autonomy.Off})

            # x:1078 y:162
            OperatableStateMachine.add('Available Screw In current Gripper',
                                        PartAvailablePoseService(check_only_current_gripper=True),
                                        transitions={'succeeded': 'succeeded', 'aborted': 'Gripper Complete'},
                                        autonomy={'succeeded': Autonomy.Off, 'aborted': Autonomy.Off},
                                        remapping={'last_trajectory_id': 'next_trajectory', 'list_available_parts': 'list_available_screw_poses'})



        with _state_machine:
            # x:112 y:30
            OperatableStateMachine.add('Load Trajectory',
                                        SharedWsActionState(exec_speed=exec_speed, exec_acceleration=exec_acceleration, coll_threshold=coll_threshold, coll_repeats=coll_repeats),
                                        transitions={'succeeded': 'Move To Home', 'preempted': 'failed', 'aborted': 'failed'},
                                        autonomy={'succeeded': Autonomy.Off, 'preempted': Autonomy.Off, 'aborted': Autonomy.Off},
                                        remapping={'action_id': 'eREAD_FROM_DISK', 'trajectory_id': 'ZERO', 'results': 'results'})

            # x:1060 y:458
            OperatableStateMachine.add('Execute New Trajectory',
                                        SharedWsActionState(exec_speed=exec_speed, exec_acceleration=exec_acceleration, coll_threshold=coll_threshold, coll_repeats=coll_repeats),
                                        transitions={'succeeded': 'MotekInsertScrew', 'preempted': 'Execute New Trajectory', 'aborted': 'Sleep Until Retry'},
                                        autonomy={'succeeded': Autonomy.Off, 'preempted': Autonomy.Off, 'aborted': Autonomy.Off},
                                        remapping={'action_id': 'eEXECUTE_NEW_TRAJ', 'trajectory_id': 'next_trajectory', 'results': 'results'})

            # x:747 y:236
            OperatableStateMachine.add('Find Collision Free Trajectory',
                                        SharedWsNewTrajActionState(exec_speed=exec_speed, exec_acceleration=exec_acceleration, coll_threshold=coll_threshold, coll_repeats=coll_repeats),
                                        transitions={'succeeded': 'Log_next_traj_id', 'preempted': 'failed', 'aborted': 'WaitForScrews'},
                                        autonomy={'succeeded': Autonomy.Off, 'preempted': Autonomy.Off, 'aborted': Autonomy.Off},
                                        remapping={'action_id': 'eCREATE_NEW_TRAJ', 'trajectory_id': 'next_trajectory', 'list_available_screw_poses': 'list_available_screw_poses', 'results': 'results', 'next_trajectory': 'next_trajectory'})

            # x:332 y:317
            OperatableStateMachine.add('Grip Ended Check',
                                        _sm_grip_ended_check_0,
                                        transitions={'aborted': 'Move To Home', 'succeeded': 'log_succeded', 'fail': 'failed'},
                                        autonomy={'aborted': Autonomy.Inherit, 'succeeded': Autonomy.Inherit, 'fail': Autonomy.Inherit},
                                        remapping={'next_trajectory': 'next_trajectory', 'insertion_successful': 'insertion_successful', 'screw_found': 'screw_found', 'list_available_screw_poses': 'list_available_screw_poses'})

            # x:1003 y:300
            OperatableStateMachine.add('Log_next_traj_id',
                                        LogKeyState(text="MAIN: Next Free Tjact. ID is for screw : {}", severity=Logger.REPORT_HINT),
                                        transitions={'done': 'Execute New Trajectory'},
                                        autonomy={'done': Autonomy.Off},
                                        remapping={'data': 'next_trajectory'})

            # x:1461 y:673
            OperatableStateMachine.add('MotekInsertScrew',
                                        self.use_behavior(MotekInsertScrewSM, 'MotekInsertScrew'),
                                        transitions={'finished': 'Grip Ended Check', 'failed': 'failed'},
                                        autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
                                        remapping={'velocity': 'velocity', 'insertion_successful': 'insertion_successful', 'screw_found': 'screw_found'})

            # x:334 y:30
            OperatableStateMachine.add('Move To Home',
                                        SharedWsActionState(exec_speed=exec_speed, exec_acceleration=exec_acceleration, coll_threshold=coll_threshold, coll_repeats=coll_repeats),
                                        transitions={'succeeded': 'Available Screws Request', 'preempted': 'failed', 'aborted': 'failed'},
                                        autonomy={'succeeded': Autonomy.Off, 'preempted': Autonomy.Off, 'aborted': Autonomy.Off},
                                        remapping={'action_id': 'eDRIVE_TO_START', 'trajectory_id': 'ZERO', 'results': 'results'})

            # x:1374 y:211
            OperatableStateMachine.add('Sleep Until Retry',
                                        WaitState(wait_time=2),
                                        transitions={'done': 'Available Screws Request'},
                                        autonomy={'done': Autonomy.Off})

            # x:773 y:90
            OperatableStateMachine.add('WaitForScrews',
                                        WaitState(wait_time=1),
                                        transitions={'done': 'Move To Home'},
                                        autonomy={'done': Autonomy.Off})

            # x:769 y:526
            OperatableStateMachine.add('log_succeded',
                                        LogKeyState(text="MAIN: available screws \n{}", severity=Logger.REPORT_HINT),
                                        transitions={'done': 'Find Collision Free Trajectory'},
                                        autonomy={'done': Autonomy.Off},
                                        remapping={'data': 'list_available_screw_poses'})

            # x:1274 y:8
            OperatableStateMachine.add('Available Screws Request',
                                        PartAvailablePoseService(check_only_current_gripper=False),
                                        transitions={'succeeded': 'Find Collision Free Trajectory', 'aborted': 'WaitForScrews'},
                                        autonomy={'succeeded': Autonomy.Off, 'aborted': Autonomy.Off},
                                        remapping={'last_trajectory_id': 'next_trajectory', 'list_available_parts': 'list_available_screw_poses'})


        return _state_machine


    # Private functions can be added inside the following tags
    # [MANUAL_FUNC]

    # [/MANUAL_FUNC]
