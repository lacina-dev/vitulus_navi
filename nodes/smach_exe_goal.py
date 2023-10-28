import rospy
import smach
import smach_ros

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Int16, String

from mbf_msgs.msg import ExePathAction
from mbf_msgs.msg import GetPathAction
from mbf_msgs.msg import RecoveryAction


def main():
    rospy.init_node('mbf_state_machine_goal')

    # Create SMACH state machine
    sm = smach.StateMachine(outcomes=['succeeded', 'aborted', 'preempted'])

    # Define userdata
    sm.userdata.goal = None
    sm.userdata.path = None
    sm.userdata.error = None
    sm.userdata.clear_costmap_flag = False
    sm.userdata.error_status = None

    with sm:
        # Goal callback for state WAIT_FOR_GOAL
        def goal_cb(userdata, msg):
            userdata.goal = msg
            return False

        # Monitor topic to get MeshGoal from RViz plugin
        smach.StateMachine.add(
            'WAIT_FOR_GOAL',
            smach_ros.MonitorState(
                '/move_base_smach/goal',
                PoseStamped,
                goal_cb,
                output_keys=['goal']
            ),
            transitions={
                'invalid': 'GET_PATH',
                'valid': 'WAIT_FOR_GOAL',
                'preempted': 'preempted'
            }
        )

        # Get path
        def get_path_result_cb(userdata, status, result):
            print("get path status: {}".format(status))
            print("get path result: {}".format(result.message))
            if result.outcome == 2:
                return 'aborted'
            else:
                pub_log_info = rospy.Publisher('/nextion/log_info', String, queue_size=10)
                pub_log_info.publish(String("Get path"))
                pub_pm_play_melody = rospy.Publisher('/pm/play_melody', Int16, queue_size=10)
                pub_pm_play_melody.publish(Int16(5))  # 5:short_beep, 1:beep, 2:double beep,
                return 'succeeded'

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
                'aborted': 'WAIT_FOR_GOAL',
                'preempted': 'preempted'
            },
            remapping={
                'target_pose': 'goal'
            }
        )

        # Execute path
        smach.StateMachine.add(
            'EXE_PATH',
            smach_ros.SimpleActionState(
                '/move_base_flex/exe_path',
                ExePathAction,
                goal_slots=['path']
            ),
            transitions={
                'succeeded': 'succeeded',
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
                output_keys = ["error_status", 'clear_costmap_flag']
            ),
            transitions={
                'succeeded': 'GET_PATH',
                'aborted': 'WAIT_FOR_GOAL',
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