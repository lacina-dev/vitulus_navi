import rospy
import smach
import smach_ros
from smach import Iterator, CBState

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Bool, String, Int16
from vitulus_msgs.msg import PlannerProgram, Mower


class GetZoneData(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['available', 'failed'],
                             input_keys=['program', 'zone_cut_height', 'zone_rpm', 'index'],
                             output_keys=['zone_cut_height', 'zone_rpm'])

    def execute(self, userdata):
        rospy.loginfo('[{}]Getting zone data - state GET_ZONE_DATA'.format(rospy.get_caller_id()))
        userdata.zone_cut_height = userdata.program.zone_list[userdata.index].cut_height
        userdata.zone_rpm = userdata.program.zone_list[userdata.index].rpm
        print(userdata.zone_cut_height)
        rospy.sleep(3)
        return 'available'


def main():
    rospy.init_node('mower_unit_smach')

    # Create SMACH state machine
    sm = smach.StateMachine(outcomes=['succeeded', 'aborted', 'preempted'])

    # Define userdata
    sm.userdata.path_plan = None
    sm.userdata.path_plan_start = None
    sm.userdata.program = None
    sm.userdata.zone_cut_height = None
    sm.userdata.zone_rpm = None
    sm.userdata.zone_name = None
    sm.userdata.zone_id = 0
    sm.userdata.path_id = 0

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
                'invalid': 'ZONE_IT',
                'valid': 'WAIT_FOR_PROGRAM',
                'preempted': 'preempted'
            }
        )

        # Iterator for zones
        zone_it = Iterator(outcomes=['succeeded', 'preempted', 'aborted'],
                           input_keys=['program', 'zone_cut_height', 'zone_rpm'],
                           it=lambda: range(0, len(sm.userdata.program.zone_list)),
                           output_keys=['zone_cut_height', 'zone_rpm'],
                           it_label='index',
                           exhausted_outcome='succeeded')

        with zone_it:
            # Create smach state machine for processing zone
            process_zone_sm = smach.StateMachine(outcomes=['succeeded', 'preempted', 'aborted', 'continue'],
                                                 input_keys=['program', 'zone_cut_height', 'zone_rpm', 'index'],
                                                 output_keys=['zone_cut_height', 'zone_rpm'])

            with process_zone_sm:
                # Add state GET_ZONE_DATA
                smach.StateMachine.add('GET_ZONE_DATA', GetZoneData(),
                                       transitions={'available': 'TURN_ON_MOWER', 'failed': 'aborted'})

                # Add state TURN_ON_MOWER
                @smach.cb_interface(input_keys=[], output_keys=[], outcomes=['success', 'failure'])
                def turn_on_mower_cb(userdata):
                    rospy.loginfo('[{}]Turning on mower - state TURN_ON_MOWER'.format(rospy.get_caller_id()))
                    pub_mower_set_power = rospy.Publisher('/mower/set_power', Bool, latch=True, queue_size=1)
                    msg = Bool()
                    msg.data = True
                    pub_status = pub_mower_set_power.publish(msg)
                    if pub_status is None:
                        return 'success'
                smach.StateMachine.add('TURN_ON_MOWER', CBState(turn_on_mower_cb),
                                       {'success': 'WAIT_FOR_MOWER_ON', 'failure': 'TURN_ON_MOWER'})

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
                                           'invalid': 'SET_CUT_HEIGHT',
                                           'preempted': 'preempted'})

                # Add state SET_CUT_HEIGHT
                @smach.cb_interface(input_keys=['zone_cut_height'], output_keys=[], outcomes=['success', 'failure'])
                def set_height_cb(userdata):
                    rospy.loginfo('[{}]Setting cut height on mower - state SET_CUT_HEIGHT'.format(rospy.get_caller_id()))
                    pub_mower_set_cut_height = rospy.Publisher('/mower/set_cut_height', Int16, latch=True, queue_size=1)
                    msg = Int16()
                    msg.data = userdata.zone_cut_height
                    pub_status = pub_mower_set_cut_height.publish(msg)
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
                                           'invalid': 'SET_MOTOR_OFF_AND_HOME',
                                           'preempted': 'preempted'})

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
        smach.StateMachine.add('ZONE_IT', zone_it, {'succeeded': 'succeeded', 'aborted': 'aborted'})

    # Create and start introspection server
    sis = smach_ros.IntrospectionServer('smach_server', sm, '/WAIT_FOR_PROGRAM')
    sis.start()
    sm.execute()
    rospy.spin()
    sis.stop()


if __name__ == "__main__":
    main()
