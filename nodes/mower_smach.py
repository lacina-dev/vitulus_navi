import rospy
import smach
import smach_ros

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from vitulus_msgs.msg import PlannerProgram

from mbf_msgs.msg import ExePathAction
from mbf_msgs.msg import GetPathAction
from mbf_msgs.msg import RecoveryAction

# define state Select path
class SelectPath(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['available', 'finished'],
                             input_keys=['program', 'zone_id', 'path_id'],
                             output_keys=['path_plan', 'path_plan_start', 'zone_id', 'path_id'])

    def execute(self, userdata):
        rospy.loginfo('[{}]Executing state SELECT_PATH'.format(rospy.get_caller_id()))
        if userdata.zone_id < len(userdata.program.zone_list):
            if userdata.path_id < len(userdata.program.zone_list[userdata.zone_id].paths):
                userdata.path_plan = userdata.program.zone_list[userdata.zone_id].paths[userdata.path_id]
                userdata.path_plan_start = userdata.program.zone_list[userdata.zone_id].paths[userdata.path_id].poses[0]
                userdata.path_id += 1
                if userdata.path_id == len(userdata.program.zone_list[userdata.zone_id].paths):
                    userdata.path_id = 0
                    userdata.zone_id += 1
                return 'available'
        else:
            userdata.path_plan = None
            userdata.path_id = 0
            userdata.zone_id = 0
            return 'finished'




# Create smach_ros action client subscribing to topic /program_active
def main():
    rospy.init_node('mower_state_machine')

    # Create SMACH state machine
    sm = smach.StateMachine(outcomes=['succeeded', 'aborted', 'preempted'])

    # Define userdata
    sm.userdata.goal = None
    sm.userdata.path = None
    sm.userdata.path_plan = None
    sm.userdata.path_plan_start = None
    sm.userdata.program = None
    sm.userdata.zone_id = 0
    sm.userdata.path_id = 0
    sm.userdata.error = None
    sm.userdata.clear_costmap_flag = False
    sm.userdata.error_status = None

    with sm:

        # Program callback for state WAIT_FOR_PROGRAM
        def callback_program(userdata, msg):
            userdata.program = msg
            userdata.zone_id = 0
            userdata.path_id = 0
            return False

        # Monitor topic to get program
        smach.StateMachine.add(
            'WAIT_FOR_PROGRAM',
            smach_ros.MonitorState(
                '/program_active',
                PlannerProgram,
                callback_program,
                output_keys=['program', 'zone_id', 'path_id']
            ),
            transitions={
                'invalid': 'SELECT_PATH',
                'valid': 'WAIT_FOR_PROGRAM',
                'preempted': 'preempted'
            }
        )

        # Add state SELECT_PATH
        smach.StateMachine.add('SELECT_PATH', SelectPath(),
                               transitions={'available': 'GET_PATH',
                                            'finished': 'succeeded'})


        # # Goal callback for state WAIT_FOR_GOAL
        # def goal_cb(userdata, msg):
        #     userdata.goal = msg
        #     return False
        #
        # # Monitor topic to get MeshGoal from RViz plugin
        # smach.StateMachine.add(
        #     'WAIT_FOR_GOAL',
        #     smach_ros.MonitorState(
        #         '/move_base_simple/goal',
        #         PoseStamped,
        #         goal_cb,
        #         output_keys=['goal']
        #     ),
        #     transitions={
        #         'invalid': 'GET_PATH',
        #         'valid': 'WAIT_FOR_GOAL',
        #         'preempted': 'preempted'
        #     }
        # )

        def get_path_result_cb(userdata, status, result):
            print("get path status: {}".format(status))
            print("get path result: {}".format(result.message))
            if result.outcome == 2:
                return 'aborted'
            else:
                return 'succeeded'

        # Get path
        smach.StateMachine.add(
            'GET_PATH',
            smach_ros.SimpleActionState(
                '/move_base_flex/get_path',
                GetPathAction,
                goal_slots=['target_pose'],
                result_slots=['path'],
                result_cb=get_path_result_cb
            ),
            transitions={
                'succeeded': 'EXE_PATH',
                'aborted': 'SELECT_PATH',
                'preempted': 'preempted'
            },
            remapping={
                'target_pose': 'path_plan_start'
            }
        )
        # Execute path plan
        smach.StateMachine.add(
            'EXE_PATH_PLAN',
            smach_ros.SimpleActionState(
                '/move_base_flex/exe_path',
                ExePathAction,
                goal_slots=['path']
            ),
            transitions={
                'succeeded': 'SELECT_PATH',
                'aborted': 'GET_PATH',
                'preempted': 'preempted'
            },
            remapping={
                'path': 'path_plan'
            }
        )

        def exe_path_result_cb(userdata, status, result):
            print("exe path status: {}".format(status))
            print("exe path result: {}".format(result))
            if result.outcome == 2:
                return 'aborted'
            else:
                return 'succeeded'

        # Execute path
        smach.StateMachine.add(
            'EXE_PATH',
            smach_ros.SimpleActionState(
                '/move_base_flex/exe_path',
                ExePathAction,
                goal_slots=['path'],
                result_cb=exe_path_result_cb
            ),
            transitions={
                # 'succeeded': 'succeeded',
                # 'aborted': 'RECOVERY',
                'succeeded': 'EXE_PATH_PLAN',
                'aborted': 'RECOVERY',
                'preempted': 'preempted'
            }
        )

        # Goal callback for state RECOVERY
        def recovery_path_goal_cb(userdata, goal):
            if userdata.clear_costmap_flag == False:
                goal.behavior = 'clear_costmap'
                userdata.clear_costmap_flag = True
            else:
                goal.behavior = 'straf_recovery'
                userdata.clear_costmap_flag = False

        # Recovery
        smach.StateMachine.add(
            'RECOVERY',
             smach_ros.SimpleActionState(
                'move_base_flex/recovery',
                RecoveryAction,
                goal_cb=recovery_path_goal_cb,
                input_keys=["error", "clear_costmap_flag"],
                output_keys=["error_status", 'clear_costmap_flag']
            ),
            transitions={
                'succeeded': 'GET_PATH',
                'aborted': 'aborted',
                # 'aborted': 'SELECT_PATH',
                'preempted': 'preempted'
            }
        )

    # Create and start introspection server
    sis = smach_ros.IntrospectionServer('smach_server', sm, '/SM_ROOT')
    sis.start()

    # Execute SMACH plan
    sm.execute()

    # Wait for interrupt and stop introspection server
    rospy.spin()
    sis.stop()

if __name__=="__main__":
    main()