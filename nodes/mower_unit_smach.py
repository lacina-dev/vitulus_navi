import rospy
import smach
import smach_ros
from smach_ros import ServiceState
from smach import Iterator, CBState
import math
from shapely import geometry

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Bool, String, Int16
from vitulus_msgs.msg import PlannerProgram, Mower
from mbf_msgs.msg import ExePathAction
from mbf_msgs.msg import GetPathAction
from mbf_msgs.msg import RecoveryAction
from mbf_msgs.srv import CheckPath, CheckPathRequest, CheckPathResponse

import sys


class GetZoneData(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['available', 'failed'],
                             input_keys=['program', 'zone_cut_height', 'zone_rpm', 'index', 'zone_name',
                                         'zone_start_pose', 'paths'],
                             output_keys=['zone_cut_height', 'zone_rpm', 'zone_name', 'zone_start_pose', 'paths'])

    def execute(self, userdata):
        rospy.loginfo('[{}]Getting zone data - state GET_ZONE_DATA'.format(rospy.get_caller_id()))
        userdata.zone_cut_height = userdata.program.zone_list[userdata.index].cut_height
        userdata.zone_rpm = userdata.program.zone_list[userdata.index].rpm
        userdata.zone_name = userdata.program.zone_list[userdata.index].name
        userdata.zone_start_pose = userdata.program.zone_list[userdata.index].paths[0].poses[0]
        userdata.paths = userdata.program.zone_list[userdata.index].paths
        userdata.controller = 'base_local_planner/TrajectoryPlannerROS'
        print(userdata.zone_name)
        print(userdata.zone_cut_height)
        # print(userdata.zone_start_pose)
        print(len(userdata.paths))
        rospy.sleep(3)
        return 'available'


class GetPathData(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['available', 'failed'],
                             input_keys=['program', 'zone_cut_height', 'zone_rpm', 'index', 'zone_name', 'path_planner',
                                         'zone_start_pose', 'path_plan', 'index_path', 'paths', 'path_start_pose',
                                         'path'],
                             output_keys=['zone_cut_height', 'zone_rpm', 'zone_name', 'zone_start_pose', 'path_planner',
                                          'path_plan', 'paths', 'path_start_pose', 'path'])

    def execute(self, userdata):
        rospy.loginfo('[{}]Getting path data - state GET_PATH_DATA'.format(rospy.get_caller_id()))
        path = userdata.paths[userdata.index_path]
        userdata.path_planner = path
        userdata.path_start_pose = path.poses[0]
        # pose_tmp = path.poses[0]
        # userdata.path_plan = None
        # line = geometry.LineString([[path.poses[0].pose.position.x, path.poses[0].pose.position.y],
        #                             [path.poses[-1].pose.position.x, path.poses[-1].pose.position.y]])
        # # angle = self.direction(line.coords)
        # line = line.segmentize(0.05)
        # poses = []
        # path.poses = []
        # for point in line.coords:
        #     pose = PoseStamped()
        #     pose.header = pose_tmp.header
        #     pose.pose.position.x = point[0]
        #     pose.pose.position.y = point[1]
        #     pose.pose.orientation.z = pose_tmp.pose.orientation.z
        #     pose.pose.orientation.w = pose_tmp.pose.orientation.w
        #     path.poses.append(pose)
        # userdata.path = path
        userdata.path = path

        print("Path length: {}  ************************".format(len(userdata.path.poses)))
        # rospy.sleep(3)
        return 'available'

class CheckDistance(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['on_place', 'exe_path'],
                             input_keys=['path_cost'],
                             output_keys=['path_cost'])

    def execute(self, userdata):
        rospy.loginfo('[{}]Checking path distance - state CHECK_DISTANCE'.format(rospy.get_caller_id()))
        print("Path distance: {}".format(userdata.path_cost))
        if userdata.path_cost <= 0.2:
            return 'on_place'
        else:
            return 'exe_path'


def main():
    rospy.init_node('mower_unit_smach')

    # Create SMACH state machine
    sm = smach.StateMachine(outcomes=['succeeded', 'aborted', 'preempted'])

    # Define userdata
    sm.userdata.path_plan = None
    sm.userdata.path = None
    sm.userdata.path_planner = None
    sm.userdata.path_start_pose = None
    sm.userdata.program = None
    sm.userdata.zone_cut_height = None
    sm.userdata.zone_rpm = None
    sm.userdata.zone_name = None
    sm.userdata.zone_start_pose = None
    sm.userdata.paths = None
    sm.userdata.path_cost = None

    with sm:
        # Program callback for state WAIT_FOR_PROGRAM
        def callback_program(userdata, msg):
            userdata.program = msg
            return False

        # Monitor topic to get program
        smach.StateMachine.add(
            'WAIT_FOR_PROGRAM',
            smach_ros.MonitorState(
                '/program_active',
                PlannerProgram,
                callback_program,
                output_keys=['program']
            ),
            transitions={
                'invalid': 'POWER_ON_MOWER',
                'valid': 'WAIT_FOR_PROGRAM',
                'preempted': 'preempted'
            }
        )

        # Add state POWER_ON_MOWER
        @smach.cb_interface(input_keys=[], output_keys=[], outcomes=['success', 'failure'])
        def turn_on_mower_cb(userdata):
            rospy.loginfo('[{}]Turning on mower - state POWER_ON_MOWER'.format(rospy.get_caller_id()))
            pub_mower_set_power = rospy.Publisher('/mower/set_power', Bool, latch=True, queue_size=1)
            msg = Bool()
            msg.data = True
            pub_status = pub_mower_set_power.publish(msg)
            if pub_status is None:
                return 'success'

        smach.StateMachine.add('POWER_ON_MOWER', CBState(turn_on_mower_cb),
                               {'success': 'WAIT_FOR_MOWER_ON', 'failure': 'POWER_ON_MOWER'})

        # Program callback for mower status WAIT_FOR_MOWER_ON
        def callback_mower_status(userdata, msg):
            if msg.status == 'WAIT':
                return False
            else:
                return True

        smach.StateMachine.add('WAIT_FOR_MOWER_ON',
                               smach_ros.MonitorState('/mower/status', Mower, callback_mower_status, output_keys=[]),
                               transitions={
                                   'valid': 'WAIT_FOR_MOWER_ON',
                                   'invalid': 'ZONE_IT',
                                   'preempted': 'preempted'})

        # Iterator for zones
        zone_it = Iterator(outcomes=['succeeded', 'preempted', 'aborted'],
                           input_keys=['program', 'zone_cut_height', 'zone_rpm', 'zone_name', 'zone_start_pose',
                                       'path_plan', 'paths'],
                           it=lambda: range(0, len(sm.userdata.program.zone_list)),
                           output_keys=['zone_cut_height', 'zone_rpm', 'zone_name', 'zone_start_pose', 'path_plan',
                                        'paths'],
                           it_label='index',
                           exhausted_outcome='succeeded')
        zone_it.userdata = sm.userdata
        with zone_it:
            # Create smach state machine for processing zone
            process_zone_sm = smach.StateMachine(outcomes=['succeeded', 'preempted', 'aborted', 'continue'],
                                                 input_keys=['program', 'zone_cut_height', 'zone_rpm', 'index',
                                                             'zone_name', 'zone_start_pose', 'path_plan', 'paths'],
                                                 output_keys=['zone_cut_height', 'zone_rpm', 'zone_name',
                                                              'zone_start_pose', 'path_plan', 'paths'])
            process_zone_sm.userdata = zone_it.userdata
            with process_zone_sm:
                # Add state GET_ZONE_DATA
                smach.StateMachine.add('GET_ZONE_DATA', GetZoneData(),
                                       transitions={'available': 'GET_PATH_TO_START', 'failed': 'aborted'})

                # Add state GET_PATH_TO_START - Get path to start point of the zone
                def get_path_result_cb(userdata, status, result):
                    print("get path status: {}".format(status))
                    print("get path result: {}".format(result.message))
                    if result.outcome == 2:
                        return 'aborted'
                    else:
                        return 'succeeded'

                smach.StateMachine.add('GET_PATH_TO_START', smach_ros.SimpleActionState(
                    '/move_base_flex/get_path', GetPathAction, goal_slots=['target_pose'],
                    result_slots=['path'], result_cb=get_path_result_cb
                ),
                                       transitions={
                                           'succeeded': 'EXE_PATH_TO_START',
                                           'aborted': 'GET_PATH_TO_START',
                                           'preempted': 'preempted'
                                       },
                                       remapping={
                                           'target_pose': 'zone_start_pose',
                                           'path': 'path_plan'
                                       }
                                       )

                # Add state EXE_PATH_TO_START - Execute path to start point
                def get_exe_result_cb(userdata, status, result):
                    print("get path status: {}".format(status))
                    print("get path result: {}".format(result.message))
                    # print("get paths: {}".format(len(userdata.paths)))
                    if result.outcome == 2:
                        return 'aborted'
                    else:
                        return 'succeeded'

                smach.StateMachine.add('EXE_PATH_TO_START', smach_ros.SimpleActionState(
                    '/move_base_flex/exe_path', ExePathAction, goal_slots=['path'], result_cb=get_exe_result_cb,
                    input_keys=['paths']
                ),
                                       transitions={
                                           'succeeded': 'SET_CUT_HEIGHT',
                                           'aborted': 'GET_PATH_TO_START',
                                           'preempted': 'preempted'
                                       }, remapping={'path': 'path_plan'}
                                       )

                # Add state SET_CUT_HEIGHT
                @smach.cb_interface(input_keys=['zone_cut_height'], output_keys=[], outcomes=['success', 'failure'])
                def set_height_cb(userdata):
                    rospy.loginfo(
                        '[{}]Setting cut height on mower - state SET_CUT_HEIGHT'.format(rospy.get_caller_id()))
                    pub_mower_set_cut_height = rospy.Publisher('/mower/set_cut_height', Int16, latch=True, queue_size=1)
                    msg = Int16()
                    msg.data = userdata.zone_cut_height
                    pub_status = pub_mower_set_cut_height.publish(msg)
                    rospy.sleep(0.5)
                    if pub_status is None:
                        return 'success'

                smach.StateMachine.add('SET_CUT_HEIGHT', CBState(set_height_cb),
                                       {'success': 'WAIT_FOR_SET_CUT_HEIGHT', 'failure': 'SET_CUT_HEIGHT'})

                # Callback for mower set cut height WAIT_FOR_SET_CUT_HEIGHT
                def callback_set_cut_height(userdata, msg):
                    if msg.status == 'WAIT':
                        return False
                    else:
                        return True

                smach.StateMachine.add('WAIT_FOR_SET_CUT_HEIGHT',
                                       smach_ros.MonitorState('/mower/status', Mower, callback_set_cut_height,
                                                              output_keys=[], input_keys=['zone_cut_height']),
                                       transitions={
                                           'valid': 'WAIT_FOR_SET_CUT_HEIGHT',
                                           'invalid': 'SET_RPM',
                                           'preempted': 'preempted'})

                # Add state SET_RPM
                @smach.cb_interface(input_keys=['zone_rpm'], output_keys=[], outcomes=['success', 'failure'])
                def set_rpm_cb(userdata):
                    rospy.loginfo(
                        '[{}]Setting motor rpm on mower - state SET_RPM'.format(rospy.get_caller_id()))
                    pub_mower_set_motor_rpm = rospy.Publisher('/mower/set_motor_rpm', Int16, latch=True, queue_size=1)
                    msg = Int16()
                    msg.data = userdata.zone_rpm
                    pub_status = pub_mower_set_motor_rpm.publish(msg)
                    if pub_status is None:
                        return 'success'

                smach.StateMachine.add('SET_RPM', CBState(set_rpm_cb),
                                       {'success': 'SET_MOTOR_ON', 'failure': 'SET_RPM'})

                # Add state SET_MOTOR_ON
                @smach.cb_interface(input_keys=[], output_keys=[], outcomes=['success', 'failure'])
                def set_motor_on_cb(userdata):
                    rospy.loginfo(
                        '[{}]Setting mower motor on - state SET_MOTOR_ON'.format(rospy.get_caller_id()))
                    pub_mower_set_motor_on = rospy.Publisher('/mower/set_motor_on', Bool, latch=True, queue_size=1)
                    msg = Bool()
                    msg.data = True
                    pub_status = pub_mower_set_motor_on.publish(msg)
                    if pub_status is None:
                        return 'success'

                smach.StateMachine.add('SET_MOTOR_ON', CBState(set_motor_on_cb),
                                       {'success': 'WAIT_FOR_SET_RPM', 'failure': 'SET_MOTOR_ON'})

                # Callback for mower set cut height WAIT_FOR_SET_RPM
                def callback_set_rpm(userdata, msg):
                    if userdata.zone_rpm - 10 < msg.moto_rpm < userdata.zone_rpm + 10:
                        return False
                    else:
                        return True

                smach.StateMachine.add('WAIT_FOR_SET_RPM',
                                       smach_ros.MonitorState('/mower/status', Mower, callback_set_rpm,
                                                              output_keys=[], input_keys=['zone_rpm']),
                                       transitions={
                                           'valid': 'WAIT_FOR_SET_RPM',
                                           'invalid': 'PATH_IT',
                                           'preempted': 'preempted'})

                ####### Iterate trough paths ##########################
                ####### Provide mowing of paths in zone ###############

                # Iterator for paths
                path_it = Iterator(outcomes=['succeeded', 'preempted', 'aborted'],
                                   input_keys=['program', 'path_plan', 'zone_cut_height', 'zone_rpm',
                                               'zone_name', 'zone_start_pose', 'paths'],
                                   it=lambda: range(0, len(process_zone_sm.userdata.paths)),
                                   output_keys=['path_plan', 'zone_cut_height', 'zone_rpm', 'zone_name',
                                                'zone_start_pose', 'index_path'],
                                   it_label='index_path',
                                   exhausted_outcome='succeeded')
                path_it.userdata = process_zone_sm.userdata
                with path_it:
                    # Create smach state machine for processing path
                    process_path_sm = smach.StateMachine(
                        outcomes=['succeeded_path', 'preempted', 'aborted_path', 'continue_path'],
                        input_keys=['program', 'zone_cut_height', 'zone_rpm', 'index',
                                    'zone_name', 'zone_start_pose', 'path_plan', 'index_path', 'paths'],
                        output_keys=['zone_cut_height', 'zone_rpm', 'zone_name',
                                     'zone_start_pose', 'path_plan'])
                    process_path_sm.userdata = path_it.userdata
                    with process_path_sm:
                        # Add state GET_PATH_DATA
                        smach.StateMachine.add('GET_PATH_DATA', GetPathData(),
                                               transitions={'available': 'GET_PATH_TO_BEGIN_OF_PATH',
                                                            'failed': 'GET_PATH_TO_BEGIN_OF_PATH'})

                        # Add state GET_PATH_TO_BEGIN_OF_PATH - Get path to start point of the zone
                        def get_path_to_path_result_cb(userdata, status, result):
                            print("get path status: {}".format(status))
                            print("get path result.message: {}".format(result.message))
                            print("get path result outcome: {}".format(result.outcome))
                            print("get path result: {}".format(result.cost))
                            # print("get path result: {}".format(result))

                            # print("path start pose: {}".format(userdata.path_start_pose))
                            # print("###### path_planner: {}".format(userdata.path_planner))

                            if result.outcome == 0:
                                # begin_point = [result.path.poses[0].pose.position.x,
                                #                result.path.poses[0].pose.position.y]
                                # end_point = [result.path.poses[-1].pose.position.x,
                                #              result.path.poses[-1].pose.position.y]
                                # print("distance")
                                # print(math.dist(begin_point, end_point))
                                userdata.path_cost = result.cost
                                return 'succeeded'
                            else:
                                return 'aborted'

                        smach.StateMachine.add('GET_PATH_TO_BEGIN_OF_PATH', smach_ros.SimpleActionState(
                                            '/move_base_flex/get_path', GetPathAction, goal_slots=['target_pose'],
                                            result_slots=['path'], input_keys=['path_start_pose', 'path', 'path_planner', 'path_cost'],
                                            output_keys=['path_cost'], result_cb=get_path_to_path_result_cb
                                        ),
                                        transitions={
                                           'succeeded': 'CHECK_DISTANCE',
                                           'aborted': 'GET_PATH_TO_BEGIN_OF_PATH',
                                           'preempted': 'preempted'
                                        },
                                        remapping={
                                           'target_pose': 'path_start_pose',
                                           'path': 'path_plan'
                                        })

                        # Add state CHECK_DISTANCE - check if is on place or not
                        smach.StateMachine.add('CHECK_DISTANCE', CheckDistance(),
                                               transitions={'on_place': 'CHECK_PLANNER_PATH',
                                                            'exe_path': 'EXE_PATH_TO_BEGIN_OF_PATH'})

                        # Add state EXE_PATH_TO_BEGIN_OF_PATH - Execute path to start point
                        def get_exe_to_path_result_cb(userdata, status, result):
                            print("get path status: {}".format(status))
                            print("get path result: {}".format(result.message))
                            print("get paths: {}".format(len(userdata.paths)))
                            if result.outcome == 2:
                                return 'aborted'
                            else:
                                return 'succeeded'

                        smach.StateMachine.add('EXE_PATH_TO_BEGIN_OF_PATH', smach_ros.SimpleActionState(
                            '/move_base_flex/exe_path', ExePathAction, goal_slots=['path'],
                            result_cb=get_exe_to_path_result_cb,
                            input_keys=['paths']
                        ),
                                               transitions={
                                                   'succeeded': 'CHECK_PLANNER_PATH',
                                                   'aborted': 'GET_PATH_TO_BEGIN_OF_PATH',
                                                   'preempted': 'preempted'
                                               }, remapping={'path': 'path_plan'}
                                               )

                        # Add state CHECK_PLANNER_PATH - service call to check path
                        @smach.cb_interface(input_keys=['path_planner'])
                        def check_path_request_cb(userdata, request):
                            srvs_request = CheckPathRequest()
                            srvs_request.path = userdata.path_planner
                            srvs_request.safety_dist = 0.1
                            srvs_request.lethal_cost_mult = 0
                            srvs_request.inscrib_cost_mult = 0
                            srvs_request.unknown_cost_mult = 0
                            srvs_request.costmap = CheckPathRequest.GLOBAL_COSTMAP
                            srvs_request.skip_poses = 0
                            srvs_request.use_padded_fp = False
                            srvs_request.path_cells_only = False
                            return srvs_request

                        def check_path_response_cb(userdata, response):
                            print("result: {}".format(response))
                            return 'succeeded'

                        smach.StateMachine.add('CHECK_PLANNER_PATH',
                                               ServiceState('/move_base_flex/check_path_cost',
                                                            CheckPath,
                                                            request_cb=check_path_request_cb,
                                                            response_cb=check_path_response_cb,
                                                            input_keys=['path_planner']),
                                               transitions={'succeeded': 'EXE_PLANNER_PATH',
                                                            'aborted': 'CHECK_PLANNER_PATH',
                                                            'preempted': 'preempted'})

                        # Add state EXE_PLANNER_PATH - Execute path from planner
                        def get_exe_planner_path_result_cb(userdata, status, result):
                            print("exe path status: {}".format(status))
                            print("exe path result: {}".format(result.message))
                            print("exe paths len: {}".format(len(userdata.paths)))
                            print("exe outcome: {}".format(result.outcome))
                            print("exe result: {}".format(result))
                            if result.outcome == 0:
                                return 'succeeded'
                            else:
                                return 'aborted'

                        smach.StateMachine.add('EXE_PLANNER_PATH', smach_ros.SimpleActionState(
                            '/move_base_flex/exe_path', ExePathAction, goal_slots=['path'],
                            result_cb=get_exe_planner_path_result_cb,
                            input_keys=['paths']
                        ),
                                               transitions={
                                                   'succeeded': 'continue_path',
                                                   'aborted': 'GET_PATH_TO_BEGIN_OF_PATH',
                                                   'preempted': 'preempted'
                                               }, remapping={'path': 'path'}
                                               )

                    # close processing path
                    Iterator.set_contained_state('PROCESS_PATH', process_path_sm, loop_outcomes=['continue_path'])

                # close Iterator for paths
                smach.StateMachine.add('PATH_IT', path_it,
                                       {'succeeded': 'SET_MOTOR_OFF_AND_HOME', 'aborted': 'aborted'})

                # Add state SET_MOTOR_OFF_AND_HOME
                @smach.cb_interface(input_keys=[], output_keys=[], outcomes=['success', 'failure'])
                def set_motor_off_cb(userdata):
                    rospy.loginfo(
                        '[{}]Setting mower motor on - state SET_MOTOR_OFF_AND_HOME'.format(rospy.get_caller_id()))
                    pub_mower_set_motor_off = rospy.Publisher('/mower/set_home', Bool, latch=True, queue_size=1)
                    msg = Bool()
                    msg.data = True
                    pub_status = pub_mower_set_motor_off.publish(msg)
                    if pub_status is None:
                        return 'success'

                smach.StateMachine.add('SET_MOTOR_OFF_AND_HOME', CBState(set_motor_off_cb),
                                       {'success': 'WAIT_SET_MOTOR_OFF_AND_HOME', 'failure': 'SET_MOTOR_OFF_AND_HOME'})

                # Callback for mower set cut height WAIT_SET_MOTOR_OFF_AND_HOME
                def callback_set_motor_off(userdata, msg):
                    if msg.status == 'WAIT':
                        return False
                    else:
                        return True

                smach.StateMachine.add('WAIT_SET_MOTOR_OFF_AND_HOME',
                                       smach_ros.MonitorState('/mower/status', Mower, callback_set_motor_off,
                                                              output_keys=[], input_keys=[]),
                                       transitions={
                                           'valid': 'WAIT_SET_MOTOR_OFF_AND_HOME',
                                           'invalid': 'continue',
                                           'preempted': 'preempted'})

            # close processing zone
            Iterator.set_contained_state('PROCESS_ZONE', process_zone_sm, loop_outcomes=['continue'])

        # close Iterator for zones
        smach.StateMachine.add('ZONE_IT', zone_it, {'succeeded': 'POWER_OFF_MOWER', 'aborted': 'aborted'})

        # Add state POWER_OFF_MOWER
        @smach.cb_interface(input_keys=[], output_keys=[], outcomes=['success', 'failure'])
        def set_mower_off_cb(userdata):
            rospy.loginfo(
                '[{}]Setting mower off - state POWER_OFF_MOWER'.format(rospy.get_caller_id()))
            pub_mower_off = rospy.Publisher('/mower/set_power', Bool, latch=True, queue_size=1)
            msg = Bool()
            msg.data = False
            pub_status = pub_mower_off.publish(msg)
            if pub_status is None:
                return 'success'

        smach.StateMachine.add('POWER_OFF_MOWER', CBState(set_mower_off_cb),
                               {'success': 'WAIT_MOWER_OFF', 'failure': 'POWER_OFF_MOWER'})

        # Callback for mower set cut height WAIT_MOWER_OFF
        def callback_set_mower_off(userdata, msg):
            if msg.status == 'OFF':
                return False
            else:
                return True

        smach.StateMachine.add('WAIT_MOWER_OFF',
                               smach_ros.MonitorState('/mower/status', Mower, callback_set_mower_off,
                                                      output_keys=[], input_keys=[]),
                               transitions={
                                   'valid': 'WAIT_MOWER_OFF',
                                   'invalid': 'WAIT_FOR_PROGRAM',
                                   'preempted': 'preempted'})

    # Create and start introspection server
    sis = smach_ros.IntrospectionServer('smach_server', sm, '/WAIT_FOR_PROGRAM')
    sis.start()
    sm.execute()
    rospy.spin()
    sis.stop()


if __name__ == "__main__":
    main()
