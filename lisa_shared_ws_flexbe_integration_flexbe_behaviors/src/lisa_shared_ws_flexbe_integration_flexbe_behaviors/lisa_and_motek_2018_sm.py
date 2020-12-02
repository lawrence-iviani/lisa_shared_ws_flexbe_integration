#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from flexbe_states.calculation_state import CalculationState
from flexbe_states.check_condition_state import CheckConditionState
from flexbe_states.log_key_state import LogKeyState
from flexbe_states.log_state import LogState
from flexbe_states.wait_state import WaitState
from horse_flexbe_states.part_available_pose_service_state import PartAvailablePoseService
from horse_flexbe_states.set_screw_service_state import SetScrewServiceState
from horse_flexbe_states.shared_ws_action_state import SharedWsActionState
from horse_flexbe_states.shared_ws_new_traj_action_state import SharedWsNewTrajActionState
from lisa_flexbe_states_flexbe_states.lisa_utter_actionlib_state import LisaUtterActionState
from lisa_flexbe_states_flexbe_states.lisa_utter_state import LisaUtterState
from lisa_shared_ws_flexbe_integration_flexbe_behaviors.gripper_complete_sm import GripperCompleteSM
from lisa_shared_ws_flexbe_integration_flexbe_states.check_gripper_is_finished import CheckGripperIsFinished
from lisa_shared_ws_flexbe_integration_flexbe_states.create_screw_string import LisaMotekConcatenateScrewPartString
from lisa_shared_ws_flexbe_integration_flexbe_states.get_gripper_screws_state import GripperStateListener
from lisa_shared_ws_flexbe_integration_flexbe_states.reset_screws_state_gripper import ResetScrewStatesInGripper
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
        self.add_behavior(GripperCompleteSM, 'Grip Ended Check/Ask for Gripper Complete')
        self.add_behavior(MotekInsertScrewSM, 'MotekInsertScrew')

        # semantic properties of this behavior
        self._semantic_properties = {}

        # Additional initialization code can be added inside the following tags
        # [MANUAL_INIT]

        # [/MANUAL_INIT]

        # Behavior comments:

        # ! 330 253 /Grip Ended Check
        # TODO: add confirmation step here to add or not the screw to the list of done|n

        # ! 658 244 /Grip Ended Check/Group
        # There is a bug that doesnt wait the utterance (except the first time it is executed). This is a workaround

        # ! 0 186 /Grip Ended Check
        # There is a buggy condition (apparently), where the screw can be inserted but not found. The SM motek insert seems ok but the value for found doesnt change (is this a flexbe bug??)

        # O 1468 527 /Grip Ended Check
        # connect succeded from available screw in current gripper for an utterance

        # O 17 319 /Grip Ended Check/Utter Wrong Insertion
        # for a reason i cant understand the wait time doesnt work here, i use a wait as fix and set to 0 in the uttering block

        # O 740 493 /Grip Ended Check
        # if false i need to reset the gripper



    def create(self):
        exec_speed = 0.80 #was scary at 0.35
        exec_acceleration = 1.00 # was scary at 0.40
        coll_threshold = 10 # was 14 with CVL at first, then 12 before switch to D435
        coll_repeats = 0
        utter_incomplete = 'The gripper is not finished, tip: move it on the shared work space in order to enable again'
        utter_complete = 'The gripper is finished, remove from the workspace'
        utter_next_screw = 'move to next screw in the same gripper'
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

        # x:30 y:538, x:633 y:492
        _sm_utter_wrong_insertion_0 = OperatableStateMachine(outcomes=['error', 'done'], input_keys=['next_trajectory'])

        with _sm_utter_wrong_insertion_0:
            # x:30 y:113
            OperatableStateMachine.add('utter_not_inserted_text',
                                        LisaMotekConcatenateScrewPartString(text_format="{}, was not inserted", number_part_screws=4),
                                        transitions={'done': 'UtterNotInserted'},
                                        autonomy={'done': Autonomy.Off},
                                        remapping={'screw_id': 'next_trajectory', 'text_to_utter': 'text_to_utter'})

            # x:674 y:308
            OperatableStateMachine.add('Wait Fix',
                                        WaitState(wait_time=6),
                                        transitions={'done': 'done'},
                                        autonomy={'done': Autonomy.Off})

            # x:409 y:427
            OperatableStateMachine.add('log_dbg_done',
                                        LogState(text="GCE: utter exit done", severity=Logger.REPORT_HINT),
                                        transitions={'done': 'Wait Fix'},
                                        autonomy={'done': Autonomy.Off})

            # x:378 y:221
            OperatableStateMachine.add('log_dbg_premepted',
                                        LogState(text="GCE: utter exit preemted", severity=Logger.REPORT_HINT),
                                        transitions={'done': 'Wait Fix'},
                                        autonomy={'done': Autonomy.Off})

            # x:396 y:316
            OperatableStateMachine.add('log_dbg_timeouted',
                                        LogState(text="GCE: utter exit debug", severity=Logger.REPORT_HINT),
                                        transitions={'done': 'Wait Fix'},
                                        autonomy={'done': Autonomy.Off})

            # x:46 y:259
            OperatableStateMachine.add('UtterNotInserted',
                                        LisaUtterState(context_id=None, wait_time=0, suspend_time=2),
                                        transitions={'done': 'log_dbg_done', 'preempt': 'log_dbg_premepted', 'timeouted': 'log_dbg_timeouted', 'error': 'error'},
                                        autonomy={'done': Autonomy.Off, 'preempt': Autonomy.Off, 'timeouted': Autonomy.Off, 'error': Autonomy.Off},
                                        remapping={'text_to_utter': 'text_to_utter', 'error_reason': 'error_reason'})


        # x:611 y:158
        _sm_log_screws_1 = OperatableStateMachine(outcomes=['done'], input_keys=['screw_found', 'insertion_successful', 'next_trajectory'])

        with _sm_log_screws_1:
            # x:79 y:175
            OperatableStateMachine.add('log_screw_id',
                                        LogKeyState(text="GCE-ENTER: insterd screw id {}", severity=Logger.REPORT_HINT),
                                        transitions={'done': 'log_last_insertion'},
                                        autonomy={'done': Autonomy.Off},
                                        remapping={'data': 'next_trajectory'})

            # x:423 y:441
            OperatableStateMachine.add('log_screw_found',
                                        LogKeyState(text="GCE-ENTER: screw was found ? {}", severity=Logger.REPORT_HINT),
                                        transitions={'done': 'done'},
                                        autonomy={'done': Autonomy.Off},
                                        remapping={'data': 'screw_found'})

            # x:217 y:276
            OperatableStateMachine.add('log_last_insertion',
                                        LogKeyState(text="GCE-ENTER: last insertion was succesfu? {}", severity=Logger.REPORT_HINT),
                                        transitions={'done': 'log_screw_found'},
                                        autonomy={'done': Autonomy.Off},
                                        remapping={'data': 'insertion_successful'})


        # x:422 y:304, x:170 y:189
        _sm_fix_for_screwed_but_not_found_2 = OperatableStateMachine(outcomes=['found_and_inserted', 'not_inserted'], input_keys=['screw_found', 'screw_inserted'], output_keys=['screw_found'])

        with _sm_fix_for_screwed_but_not_found_2:
            # x:156 y:37
            OperatableStateMachine.add('Check is Inserted',
                                        CheckConditionState(predicate=lambda x: x == True),
                                        transitions={'true': 'check is found', 'false': 'not_inserted'},
                                        autonomy={'true': Autonomy.Off, 'false': Autonomy.Off},
                                        remapping={'input_value': 'screw_inserted'})

            # x:287 y:127
            OperatableStateMachine.add('check is found',
                                        CheckConditionState(predicate=lambda x: x == True),
                                        transitions={'true': 'found_and_inserted', 'false': 'fix for inserted not found'},
                                        autonomy={'true': Autonomy.Off, 'false': Autonomy.Off},
                                        remapping={'input_value': 'screw_found'})

            # x:511 y:219
            OperatableStateMachine.add('fix for inserted not found',
                                        CalculationState(calculation=lambda x : True),
                                        transitions={'done': 'found_and_inserted'},
                                        autonomy={'done': Autonomy.Off},
                                        remapping={'input_value': 'screw_found', 'output_value': 'screw_found'})


        # x:806 y:366, x:1468 y:130, x:1813 y:661, x:684 y:625
        _sm_grip_ended_check_3 = OperatableStateMachine(outcomes=['fail', 'completed', 'screw_available', 'incomplete'], input_keys=['next_trajectory', 'insertion_successful', 'screw_found'], output_keys=['list_available_screw_poses'])

        with _sm_grip_ended_check_3:
            # x:63 y:21
            OperatableStateMachine.add('Log screws',
                                        _sm_log_screws_1,
                                        transitions={'done': 'Fix for screwed but not found'},
                                        autonomy={'done': Autonomy.Inherit},
                                        remapping={'screw_found': 'screw_found', 'insertion_successful': 'insertion_successful', 'next_trajectory': 'next_trajectory'})

            # x:1590 y:18
            OperatableStateMachine.add('Available Screw In current Gripper',
                                        PartAvailablePoseService(check_only_current_gripper=True),
                                        transitions={'succeeded': 'screw_available', 'aborted': 'Ask for Gripper Complete'},
                                        autonomy={'succeeded': Autonomy.Off, 'aborted': Autonomy.Off},
                                        remapping={'last_trajectory_id': 'next_trajectory', 'list_available_parts': 'list_available_screw_poses'})

            # x:852 y:16
            OperatableStateMachine.add('Available Screws',
                                        GripperStateListener(input_id_is_screw=True, time_out=1, number_part_screws=4),
                                        transitions={'succeeded': 'is_gripper_complete', 'aborted': 'log_err_screws'},
                                        autonomy={'succeeded': Autonomy.Off, 'aborted': Autonomy.Off},
                                        remapping={'gripper_id': 'next_trajectory', 'gripper_status': 'gripper_status', 'list_gripper_screws_status': 'list_gripper_screws_status', 'gripper_name': 'gripper_name', 'error_reason': 'error_reason'})

            # x:12 y:112
            OperatableStateMachine.add('Fix for screwed but not found',
                                        _sm_fix_for_screwed_but_not_found_2,
                                        transitions={'found_and_inserted': 'Update Screw State', 'not_inserted': 'Utter Wrong Insertion'},
                                        autonomy={'found_and_inserted': Autonomy.Inherit, 'not_inserted': Autonomy.Inherit},
                                        remapping={'screw_found': 'screw_found', 'screw_inserted': 'insertion_successful'})

            # x:681 y:16
            OperatableStateMachine.add('Update Screw State',
                                        SetScrewServiceState(),
                                        transitions={'succeeded': 'Available Screws', 'aborted': 'log_exit_1'},
                                        autonomy={'succeeded': Autonomy.Off, 'aborted': Autonomy.Off},
                                        remapping={'screw_id': 'next_trajectory', 'finished_successful': 'insertion_successful', 'screw_found': 'screw_found', 'result': 'result'})

            # x:370 y:181
            OperatableStateMachine.add('Utter Wrong Insertion',
                                        _sm_utter_wrong_insertion_0,
                                        transitions={'error': 'fail', 'done': 'Update Screw State'},
                                        autonomy={'error': Autonomy.Inherit, 'done': Autonomy.Inherit},
                                        remapping={'next_trajectory': 'next_trajectory'})

            # x:1339 y:373
            OperatableStateMachine.add('check answer',
                                        CheckConditionState(predicate=lambda x: x == "yes" or x == "completed"),
                                        transitions={'true': 'utter_completed', 'false': 'reset_gripper'},
                                        autonomy={'true': Autonomy.Off, 'false': Autonomy.Off},
                                        remapping={'input_value': 'answer'})

            # x:955 y:532
            OperatableStateMachine.add('exit_reset',
                                        LogState(text="DBG-EXIT: incomplete, gripper resetted", severity=Logger.REPORT_HINT),
                                        transitions={'done': 'utter_exit_not_complete'},
                                        autonomy={'done': Autonomy.Off})

            # x:1179 y:17
            OperatableStateMachine.add('is_gripper_complete',
                                        CheckGripperIsFinished(number_part_screws=4),
                                        transitions={'done': 'utter_completed', 'screws_available': 'Available Screw In current Gripper', 'operator_check': 'reset_gripper', 'error': 'fail'},
                                        autonomy={'done': Autonomy.Off, 'screws_available': Autonomy.Off, 'operator_check': Autonomy.Off, 'error': Autonomy.Off},
                                        remapping={'screw_status_list': 'list_gripper_screws_status'})

            # x:1699 y:357
            OperatableStateMachine.add('log_answer',
                                        LogKeyState(text="GCE: User reply {}", severity=Logger.REPORT_HINT),
                                        transitions={'done': 'check answer'},
                                        autonomy={'done': Autonomy.Off},
                                        remapping={'data': 'answer'})

            # x:925 y:169
            OperatableStateMachine.add('log_err_screws',
                                        LogKeyState(text="Fail with available screws: {}", severity=Logger.REPORT_HINT),
                                        transitions={'done': 'fail'},
                                        autonomy={'done': Autonomy.Off},
                                        remapping={'data': 'error_reason'})

            # x:753 y:176
            OperatableStateMachine.add('log_exit_1',
                                        LogState(text="Updtae Screw State FAIL", severity=Logger.REPORT_HINT),
                                        transitions={'done': 'fail'},
                                        autonomy={'done': Autonomy.Off})

            # x:1193 y:224
            OperatableStateMachine.add('log_exit_failed',
                                        LogKeyState(text="GCE: exit failed from Ask for gripper: {}", severity=Logger.REPORT_HINT),
                                        transitions={'done': 'fail'},
                                        autonomy={'done': Autonomy.Off},
                                        remapping={'data': 'answer'})

            # x:764 y:434
            OperatableStateMachine.add('reset_gripper',
                                        ResetScrewStatesInGripper(number_part_screws=4),
                                        transitions={'succeeded': 'exit_reset', 'aborted': 'fail', 'nothing_to_do': 'utter_completed'},
                                        autonomy={'succeeded': Autonomy.Off, 'aborted': Autonomy.Off, 'nothing_to_do': Autonomy.Off},
                                        remapping={'screw_id': 'next_trajectory', 'screw_status_list': 'list_gripper_screws_status', 'result': 'result', 'reason': 'reason'})

            # x:1193 y:125
            OperatableStateMachine.add('utter_completed',
                                        LisaUtterActionState(text_to_utter=utter_complete, wait_time=10),
                                        transitions={'uttered_all': 'completed', 'timeout': 'completed', 'command_error': 'fail'},
                                        autonomy={'uttered_all': Autonomy.Off, 'timeout': Autonomy.Off, 'command_error': Autonomy.Off},
                                        remapping={'error_reason': 'error_reason'})

            # x:397 y:535
            OperatableStateMachine.add('utter_exit_not_complete',
                                        LisaUtterActionState(text_to_utter=utter_incomplete, wait_time=10),
                                        transitions={'uttered_all': 'incomplete', 'timeout': 'incomplete', 'command_error': 'fail'},
                                        autonomy={'uttered_all': Autonomy.Off, 'timeout': Autonomy.Off, 'command_error': Autonomy.Off},
                                        remapping={'error_reason': 'error_reason'})

            # x:1512 y:572
            OperatableStateMachine.add('utter_next_traj',
                                        LisaUtterActionState(text_to_utter=utter_next_screw, wait_time=10),
                                        transitions={'uttered_all': 'screw_available', 'timeout': 'screw_available', 'command_error': 'fail'},
                                        autonomy={'uttered_all': Autonomy.Off, 'timeout': Autonomy.Off, 'command_error': Autonomy.Off},
                                        remapping={'error_reason': 'error_reason'})

            # x:1512 y:191
            OperatableStateMachine.add('Ask for Gripper Complete',
                                        self.use_behavior(GripperCompleteSM, 'Grip Ended Check/Ask for Gripper Complete'),
                                        transitions={'finished': 'log_answer', 'failed': 'log_exit_failed', 'max_retry': 'reset_gripper'},
                                        autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit, 'max_retry': Autonomy.Inherit},
                                        remapping={'answer': 'answer'})



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

            # x:447 y:688
            OperatableStateMachine.add('Grip Ended Check',
                                        _sm_grip_ended_check_3,
                                        transitions={'fail': 'failed', 'completed': 'Move To Home', 'screw_available': 'Find Collision Free Trajectory', 'incomplete': 'Move To Home'},
                                        autonomy={'fail': Autonomy.Inherit, 'completed': Autonomy.Inherit, 'screw_available': Autonomy.Inherit, 'incomplete': Autonomy.Inherit},
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

            # x:784 y:133
            OperatableStateMachine.add('WaitForScrews',
                                        WaitState(wait_time=1),
                                        transitions={'done': 'Move To Home'},
                                        autonomy={'done': Autonomy.Off})

            # x:875 y:779
            OperatableStateMachine.add('log_avail_screws',
                                        LogKeyState(text="MAIN: available screws \n{}", severity=Logger.REPORT_HINT),
                                        transitions={'done': 'Grip Ended Check'},
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
