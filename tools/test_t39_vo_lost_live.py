#!/usr/bin/env python3
"""T39: nodes/dock_idle with ~vo_lost_pause against a stand-in VO, real ROS
timing, its own roscore on a private port.

    python3 tools/test_t39_vo_lost_live.py      # starts roscore on :11939 itself

Never talks to the robot's master (refuses :11311). The stand-in VO publishes
/vo/odom at 10 Hz like rtabmap's rgbd_odometry (covariance 9999 while "dark",
measured values while "light"), serves /vo/pause_odom + /vo/resume_odom (and the
/licp pair) and stops publishing while paused. The night bags carry neither
/vo/odom nor camera images, so the source is synthetic.

  1. switch OFF (default): VO lost for 3 windows -> no service call, dock_idle
     does not even subscribe /vo/odom; the dock pause works as before;
  2. switch ON, dark: pause after one window, resume for a probe after retry_s,
     pause again; the lidar odometry is never called; duty cycle of the VO;
  3. light comes: the next probe keeps the VO running;
  4. dock: the dock pause takes over and leaving the dock resumes VO + lidar;
  5. dry run: reports, calls nothing;
  6. shutdown while the watch holds the pause: the VO is resumed.
"""
import json
import os
import signal
import subprocess
import sys
import threading
import time

PORT = int(os.environ.get('VO_LOST_TEST_PORT', '11939'))
if PORT == 11311:
    sys.exit('refusing: 11311 is the robot master')
os.environ['ROS_MASTER_URI'] = 'http://127.0.0.1:%d' % PORT
os.environ['ROS_IP'] = '127.0.0.1'
os.environ.pop('ROS_HOSTNAME', None)
HERE = os.path.dirname(os.path.abspath(__file__))
NODE = os.path.join(os.path.dirname(HERE), 'nodes', 'dock_idle')
FAIL = []
WINDOW, RETRY, PROBE, HOLD = 5.0, 10.0, 4.0, 3.0


def ok(label, cond, detail=''):
    print('  %-68s %s%s' % (label, 'ok' if cond else 'FAIL', '' if cond else ' (%s)' % (detail,)))
    if not cond:
        FAIL.append(label)


def main():
    core = subprocess.Popen(['roscore', '-p', str(PORT)], stdout=subprocess.DEVNULL,
                            stderr=subprocess.DEVNULL)
    node = [None]
    try:
        time.sleep(3.0)
        import rospy
        from nav_msgs.msg import Odometry
        from std_msgs.msg import Int8, String
        from std_srvs.srv import Empty, EmptyResponse
        assert rospy.get_master().getUri()[2].rstrip('/').endswith(':%d' % PORT)
        rospy.init_node('vo_lost_test', anonymous=True, disable_signals=True)

        vo = {'paused': False, 'light': False, 'run_s': 0.0, 'mark': None}
        calls = []                       # (monotonic, name)
        status = [None]

        def srv(name, flag):
            def h(_req):
                calls.append((time.monotonic(), name))
                if name.startswith('/vo/'):
                    vo['paused'] = flag
                return EmptyResponse()
            return rospy.Service(name, Empty, h)
        for name, flag in (('/vo/pause_odom', True), ('/vo/resume_odom', False),
                           ('/licp/pause_odom', True), ('/licp/resume_odom', False)):
            srv(name, flag)
        pub = rospy.Publisher('/vo/odom', Odometry, queue_size=5)
        dock = rospy.Publisher('/dock_smach/dock_status', Int8, queue_size=2, latch=True)
        rospy.Subscriber('/dock_idle/status', String,
                         lambda m: status.__setitem__(0, json.loads(m.data)), queue_size=5)
        alive = [True]

        def vo_loop():
            while alive[0] and not rospy.is_shutdown():
                time.sleep(0.1)
                if vo['paused']:
                    continue
                vo['run_s'] += 0.1
                m = Odometry()
                m.header.stamp = rospy.Time.now()
                c = [0.0] * 36
                c[0] = c[7] = c[35] = (4e-4 if vo['light'] else 9999.0)
                m.twist.covariance = c
                pub.publish(m)
        threading.Thread(target=vo_loop, daemon=True).start()

        def start(**params):
            for k in ('vo_lost_pause', 'vo_lost_dry_run'):
                rospy.set_param('/dock_idle/' + k, bool(params.get(k, False)))
            rospy.set_param('/dock_idle/vo_lost_window_s', WINDOW)
            rospy.set_param('/dock_idle/vo_lost_retry_s', RETRY)
            rospy.set_param('/dock_idle/vo_lost_probe_s', PROBE)
            rospy.set_param('/dock_idle/vo_lost_min_msgs', 20)
            rospy.set_param('/dock_idle/dock_hold_s', HOLD)
            del calls[:]
            vo['paused'] = False
            node[0] = subprocess.Popen([sys.executable, NODE, '__name:=dock_idle'],
                                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            time.sleep(3.0)

        def stop():
            if node[0] is not None:
                node[0].send_signal(signal.SIGINT)
                try:
                    node[0].wait(timeout=15)
                except subprocess.TimeoutExpired:
                    node[0].kill()
                node[0] = None

        def names(since=0.0):
            return [n for t, n in calls if t >= since]

        def subscribers(topic):
            state = rospy.get_master().getSystemState()[2]
            return [n for tp, nodes in state[1] if tp == topic for n in nodes]

        # ---- 1. switch off ----------------------------------------------------
        print('1. switch OFF (default)')
        dock.publish(Int8(3))
        vo['light'] = False
        start()
        time.sleep(3 * WINDOW)
        ok('lost VO for 3 windows: no service call', names() == [], names())
        ok('dock_idle does not subscribe /vo/odom', '/dock_idle' not in subscribers('/vo/odom'),
           subscribers('/vo/odom'))
        ok('status has no vo_lost key', status[0] is not None and 'vo_lost' not in status[0],
           status[0])
        dock.publish(Int8(0))
        time.sleep(HOLD + 3.5)
        ok('dock pause as before: VO + lidar paused',
           names() == ['/vo/pause_odom', '/licp/pause_odom'], names())
        dock.publish(Int8(1))
        time.sleep(1.5)
        ok('leaving the dock: VO + lidar resumed',
           names()[2:] == ['/vo/resume_odom', '/licp/resume_odom'], names())
        stop()

        # ---- 2. switch on, dark ----------------------------------------------
        print('2. switch ON, VO lost on every frame')
        dock.publish(Int8(3))
        vo['light'] = False
        start(vo_lost_pause=True)
        t0 = time.monotonic() - 3.0
        ok('dock_idle subscribes /vo/odom', '/dock_idle' in subscribers('/vo/odom'))
        time.sleep(WINDOW + 2.5)
        ok('paused after one window', names() == ['/vo/pause_odom'], names())
        t_pause = calls[0][0] if calls else t0
        ok('  ... %.1f s after start (window %.0f s)' % (t_pause - t0, WINDOW),
           WINDOW - 0.5 <= t_pause - t0 <= WINDOW + 3.0, t_pause - t0)
        ok('status: vo_lost paused', (status[0] or {}).get('vo_lost', {}).get('paused') is True
           and status[0].get('paused') is False, status[0])
        run0, m0 = vo['run_s'], time.monotonic()
        time.sleep(2 * (RETRY + PROBE) + 1.0)
        seq = names()
        ok('probe: resume, then pause again (twice)',
           seq[:5] == ['/vo/pause_odom', '/vo/resume_odom', '/vo/pause_odom',
                       '/vo/resume_odom', '/vo/pause_odom'], seq)
        ok('lidar odometry never called', not [n for n in seq if n.startswith('/licp/')], seq)
        if len(calls) >= 3:
            ok('  retry after %.1f s (%.0f), probe %.1f s (%.0f)'
               % (calls[1][0] - calls[0][0], RETRY, calls[2][0] - calls[1][0], PROBE),
               abs(calls[1][0] - calls[0][0] - RETRY) < 2.0
               and abs(calls[2][0] - calls[1][0] - PROBE) < 2.0)
        duty = (vo['run_s'] - run0) / (time.monotonic() - m0)
        print('     VO ran %.0f %% of the time while lost (expected %.0f %%)'
              % (100 * duty, 100 * PROBE / (RETRY + PROBE)))
        ok('duty cycle of the lost VO near probe/(retry+probe)',
           abs(duty - PROBE / (RETRY + PROBE)) < 0.12, duty)

        # ---- 3. light ----------------------------------------------------------
        print('3. light returns')
        vo['light'] = True
        n0 = len(calls)
        time.sleep(RETRY + PROBE + 3.0)
        ok('next probe: resumed and left running', names()[n0:] == ['/vo/resume_odom']
           and not vo['paused'], names()[n0:])
        time.sleep(WINDOW + 3.0)
        ok('tracking VO stays running', names()[n0:] == ['/vo/resume_odom'], names()[n0:])
        ok('status: state run, probes_ok 1',
           (status[0] or {}).get('vo_lost', {}).get('state') == 'run'
           and status[0]['vo_lost'].get('probes_ok') == 1, status[0])

        # ---- 4. dock -----------------------------------------------------------
        print('4. dock pause over the lost watch')
        vo['light'] = False
        n0 = len(calls)
        time.sleep(WINDOW + 3.0)
        ok('dark again: paused by the watch', names()[n0:] == ['/vo/pause_odom'], names()[n0:])
        dock.publish(Int8(0))
        n0 = len(calls)
        time.sleep(HOLD + 3.5)
        ok('docked: dock pause of VO + lidar', names()[n0:] == ['/vo/pause_odom',
                                                                 '/licp/pause_odom'], names()[n0:])
        n0 = len(calls)
        time.sleep(RETRY + PROBE + 2.0)
        ok('docked: the watch does not probe', names()[n0:] == [], names()[n0:])
        ok('status: paused (dock), vo_lost not paused',
           (status[0] or {}).get('paused') is True
           and status[0]['vo_lost'].get('paused') is False, status[0])
        dock.publish(Int8(1))
        time.sleep(1.5)
        ok('undocking: VO + lidar resumed at once',
           names()[n0:] == ['/vo/resume_odom', '/licp/resume_odom'] and not vo['paused'],
           names()[n0:])
        n1 = len(calls)
        time.sleep(WINDOW - 2.5)
        ok('after undocking the watch needs a full window again', names()[n1:] == [], names()[n1:])
        time.sleep(5.0)
        ok('... then pauses the still lost VO', names()[n1:] == ['/vo/pause_odom'], names()[n1:])

        # ---- 6. shutdown while paused ----------------------------------------
        print('6. shutdown while the watch holds the pause')
        n0 = len(calls)
        stop()
        ok('VO resumed on shutdown', names()[n0:] == ['/vo/resume_odom'] and not vo['paused'],
           names()[n0:])

        # ---- 5. dry run -------------------------------------------------------
        print('5. dry run')
        dock.publish(Int8(3))
        start(vo_lost_pause=True, vo_lost_dry_run=True)
        time.sleep(WINDOW + 3.0)
        ok('dry run: no service call', names() == [], names())
        ok('dry run: status reports the pause it would make',
           (status[0] or {}).get('vo_lost', {}).get('pauses', 0) >= 1
           and status[0]['vo_lost'].get('dry_run') is True, status[0])
        stop()
        alive[0] = False
    finally:
        if node[0] is not None:
            node[0].kill()
        core.send_signal(signal.SIGINT)
        try:
            core.wait(timeout=10)
        except subprocess.TimeoutExpired:
            core.kill()
    print('FAILED: %s' % FAIL if FAIL else 'ALL OK')
    return 1 if FAIL else 0


if __name__ == '__main__':
    sys.exit(main())
