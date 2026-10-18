#!/usr/bin/env python3
"""Off-robot test: /mower_smach/stop is immediate, idempotent, never blocking.

18. 10. 2026: a STOP to an idle mower without power blocked the subscriber
callback ~40 s in safe_blade_shutdown and only then sent /dock_smach/stop;
repeated STOPs queued up and cancelled a docking the agent started a minute
later; the node could only be killed with -9.

Part A - no master (fakes):
  * GlobalStop.on_stop returns at once even when the blade shutdown takes long
  * idle mower: blade off, NO /dock_smach/stop, no mission preempt
  * program running: /dock_smach/stop at once, once for a burst, preempt each
  * a burst of STOPs merges into at most two blade-shutdown runs
  * MowerPowerWatch.unpowered: bat_out false always; silence only when idle
  * safe_blade_shutdown on an unpowered mower: motor off, no wait at all
Part B - the REAL node on a PRIVATE roscore (never :11311):
  * mower without power (bat_out false), idle: 5 STOPs -> motor off at once,
    no /dock_smach/stop within 45 s, /mower_smach/busy false
  * rosnode kill -> process gone within 6 s; SIGTERM -> gone within 6 s

Run:  python3 tools/test_global_stop.py
"""
import os
import signal
import subprocess
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(PKG, 'src'))
PORT = int(os.environ.get('GS_TEST_PORT', '11983'))
URI = 'http://127.0.0.1:%d' % PORT
if PORT == 11311:
    sys.exit('refusing: 11311 is the robot master')

FAILS = []


def check(name, cond, detail=''):
    print('%s %s %s' % ('PASS' if cond else 'FAIL', name, '' if cond else detail),
          flush=True)
    if not cond:
        FAILS.append(name)


# ---------------------------------------------------------------- part A
def part_a():
    import rospy
    import rospy.rostime
    rospy.rostime.set_rostime_initialized(True)
    from std_msgs.msg import Bool
    from mower_unit_smach import global_stop, helpers

    class Pub(object):
        def __init__(self):
            self.msgs = []

        def publish(self, m):
            self.msgs.append(getattr(m, 'data', m))

    class Pubs(object):
        def __init__(self):
            self.mower_set_motor_on = Pub()
            self.dock_cancel = Pub()
            self.mower_set_height = Pub()

    class Latch(object):
        def __init__(self, running):
            self.running = running
            self.requests = 0

        def request(self):
            if self.running:
                self.requests += 1
            return self.running

    def slow_shutdown(**kw):
        runs.append(kw)
        time.sleep(1.0)
        return True

    print('A1 idle mower', flush=True)
    runs = []
    pubs, latch = Pubs(), Latch(False)
    gs = global_stop.GlobalStop(pubs, latch, lambda: False, shutdown_fn=slow_shutdown)
    t0 = time.monotonic()
    for _ in range(5):
        gs.on_stop(Bool(True))
    took = time.monotonic() - t0
    check('5 STOPs handled in < 0.1 s (callback never blocks)', took < 0.1, '%.3f s' % took)
    check('blade off sent for every STOP', pubs.mower_set_motor_on.msgs == [False] * 5,
          pubs.mower_set_motor_on.msgs)
    check('idle: no /dock_smach/stop', pubs.dock_cancel.msgs == [], pubs.dock_cancel.msgs)
    check('idle: no mission preempt', latch.requests == 0)
    time.sleep(2.6)
    check('burst merged into <= 2 blade-shutdown runs', 1 <= len(runs) <= 2, len(runs))
    check('blade shutdown told no program runs',
          all(r.get('program_running') is False for r in runs), runs)
    gs.on_stop(Bool(False))
    check('STOP false ignored', len(pubs.mower_set_motor_on.msgs) == 5)

    print('A2 program running', flush=True)
    runs = []
    pubs, latch = Pubs(), Latch(True)
    clock = [100.0]
    gs = global_stop.GlobalStop(pubs, latch, lambda: True, shutdown_fn=slow_shutdown,
                                clock=lambda: clock[0])
    for _ in range(4):
        gs.on_stop(Bool(True))
    check('program: /dock_smach/stop at once, once for the burst',
          pubs.dock_cancel.msgs == [True], pubs.dock_cancel.msgs)
    check('program: every STOP re-preempts the mission', latch.requests == 4, latch.requests)
    clock[0] += 5.0
    gs.on_stop(Bool(True))
    check('program: a later STOP propagates again', pubs.dock_cancel.msgs == [True, True],
          pubs.dock_cancel.msgs)
    time.sleep(2.6)
    check('program: blade shutdown runs with program_running=True',
          runs and all(r.get('program_running') is True for r in runs), runs)

    print('A3 busy unknown counts as running', flush=True)
    pubs = Pubs()

    def broken():
        raise RuntimeError('x')
    gs = global_stop.GlobalStop(pubs, None, broken, shutdown_fn=lambda **k: True,
                                start_worker=False)
    gs.on_stop(Bool(True))
    check('busy_fn failing -> treated as running (stop propagated)',
          pubs.dock_cancel.msgs == [True])

    print('A4 MowerPowerWatch', flush=True)
    now = [1000.0]
    w = helpers.MowerPowerWatch(clock=lambda: now[0])
    w.started = now[0]
    check('just started, nothing known -> not verified off',
          w.unpowered(program_running=False) == (False, ''))
    now[0] += 3.5
    check('idle + /mower/status silent 3.5 s -> off',
          w.unpowered(program_running=False)[0] is True)
    check('program + silence -> NOT off (could be a comm fault)',
          w.unpowered(program_running=True)[0] is False)

    class PS(object):
        bat_out_switch = False
    w.on_power(PS())
    check('bat_out_switch false -> off even with a program',
          w.unpowered(program_running=True)[0] is True)
    now[0] += 5.0
    check('bat_out evidence stale (> 3 s) -> not used', w.unpowered(program_running=True)[0] is False)

    class St(object):
        status = 'IDLE'
        moto_rpm = 0
        current_height = 50
        max_height = 80
    w.on_status(St())
    check('fresh /mower/status, idle -> not off', w.unpowered(program_running=False)[0] is False)

    print('A5 safe_blade_shutdown, unpowered mower', flush=True)
    w2 = helpers.MowerPowerWatch()
    w2.started = time.monotonic()
    w2.on_power(PS())
    helpers._power_watch['watch'] = w2
    real_wait = rospy.wait_for_message

    def no_wait(*a, **k):
        raise AssertionError('waited for a message')
    rospy.wait_for_message = no_wait
    rospy.get_caller_id = lambda: '/test'
    pubs = Pubs()
    t0 = time.monotonic()
    try:
        ok = helpers.safe_blade_shutdown(source='TEST', pubs=pubs, program_running=False)
        err = None
    except AssertionError as exc:
        ok, err = False, exc
    took = time.monotonic() - t0
    rospy.wait_for_message = real_wait
    helpers._power_watch['watch'] = None
    check('returns True at once (< 0.2 s), no waiting', ok and took < 0.2 and err is None,
          (ok, took, err))
    check('motor off sent, no height command', pubs.mower_set_motor_on.msgs == [False]
          and pubs.mower_set_height.msgs == [], (pubs.mower_set_motor_on.msgs,
                                                  pubs.mower_set_height.msgs))


# ---------------------------------------------------------------- part B
NODE_WRAPPER = r'''
import sys, runpy, actionlib
actionlib.SimpleActionClient.wait_for_server = lambda self, t=None: True
sys.argv = [%(node)r]
runpy.run_path(%(node)r, run_name='__main__')
'''


def env():
    e = dict(os.environ)
    e['ROS_MASTER_URI'] = URI
    e['ROS_IP'] = '127.0.0.1'
    e.pop('ROS_HOSTNAME', None)
    e['PYTHONPATH'] = os.path.join(PKG, 'src') + os.pathsep + e.get('PYTHONPATH', '')
    return e


def start_node():
    code = NODE_WRAPPER % {'node': os.path.join(PKG, 'nodes', 'mower_unit_smach')}
    return subprocess.Popen([sys.executable, '-u', '-c', code], env=env(),
                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
                            start_new_session=True)


def gone(proc, timeout):
    try:
        proc.wait(timeout=timeout)
        return True
    except subprocess.TimeoutExpired:
        return False


def part_b():
    import xmlrpc.client
    try:
        xmlrpc.client.ServerProxy(URI).getPid('/t')
        sys.exit('port %d already has a master' % PORT)
    except (ConnectionError, OSError):
        pass
    os.environ.update(ROS_MASTER_URI=URI, ROS_IP='127.0.0.1')
    os.environ.pop('ROS_HOSTNAME', None)
    core = subprocess.Popen(['roscore', '-p', str(PORT)], stdout=subprocess.DEVNULL,
                            stderr=subprocess.DEVNULL, start_new_session=True)
    master_pid = None
    node = None
    try:
        for _ in range(60):
            try:
                master_pid = xmlrpc.client.ServerProxy(URI).getPid('/t')[2]
                break
            except (ConnectionError, OSError):
                time.sleep(0.25)
        import rospy
        from std_msgs.msg import Bool, String
        from vitulus_ups.msg import power_status
        rospy.init_node('global_stop_test', disable_signals=True)
        got = {'dock': [], 'motor': [], 'busy': [], 'status': []}
        lk = threading.Lock()

        def rec(key):
            def cb(m):
                with lk:
                    got[key].append((time.monotonic(), m.data))
            return cb
        rospy.Subscriber('/dock_smach/stop', Bool, rec('dock'), queue_size=100)
        rospy.Subscriber('/mower/set_motor_on', Bool, rec('motor'), queue_size=100)
        rospy.Subscriber('/mower_smach/busy', Bool, rec('busy'), queue_size=100)
        rospy.Subscriber('/mower_smach/status', String, rec('status'), queue_size=100)
        pm = rospy.Publisher('/pm/power_status', power_status, queue_size=1)
        stop = rospy.Publisher('/mower_smach/stop', Bool, queue_size=10)
        alive = threading.Event()

        def power():
            while not alive.is_set():
                m = power_status()
                m.bat_out_switch = False
                pm.publish(m)
                time.sleep(0.2)
        threading.Thread(target=power, daemon=True).start()

        print('B1 real node, idle mower without power', flush=True)
        node = start_node()
        out = []
        threading.Thread(target=lambda: [out.append(l) for l in node.stdout],
                         daemon=True).start()
        deadline = time.time() + 40
        while time.time() < deadline and not any(d == 'Ready' for _, d in got['status']):
            time.sleep(0.2)
        check('node up (status Ready)', any(d == 'Ready' for _, d in got['status']),
              ''.join(out[-20:]))
        deadline = time.time() + 10
        while time.time() < deadline and stop.get_num_connections() == 0:
            time.sleep(0.1)
        time.sleep(2.0)
        check('busy latched false while idle', got['busy'] and got['busy'][-1][1] is False,
              got['busy'])
        with lk:
            m0 = len(got['motor'])
        t0 = time.monotonic()
        for _ in range(5):
            stop.publish(Bool(True))
            time.sleep(0.05)
        time.sleep(1.0)
        with lk:
            motor = [t for t, d in got['motor'][m0:] if d is False]
        check('blade off within 1 s of the first STOP',
              bool(motor) and motor[0] - t0 < 1.0, motor[:3])
        time.sleep(44.0)   # the old handler sent it ~40 s late
        with lk:
            dock = list(got['dock'])
        check('no /dock_smach/stop at all (idle mower) in 45 s', not dock, dock)

        print('B2 rosnode kill ends the node', flush=True)
        t0 = time.time()
        subprocess.call(['rosnode', 'kill', '/mower_unit_smach'], env=env(),
                        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        check('gone within 6 s after rosnode kill', gone(node, 6.0),
              '%.1f s' % (time.time() - t0))
        if node.poll() is None:
            os.killpg(node.pid, signal.SIGKILL)

        print('B3 SIGTERM ends the node', flush=True)
        got['status'] = []
        node = start_node()
        deadline = time.time() + 40
        while time.time() < deadline and not any(d == 'Ready' for _, d in got['status']):
            time.sleep(0.2)
        stop.publish(Bool(True))           # a STOP in flight while it ends
        time.sleep(0.2)
        node.send_signal(signal.SIGTERM)
        check('gone within 6 s after SIGTERM', gone(node, 6.0))
        alive.set()
    finally:
        if node is not None and node.poll() is None:
            os.killpg(node.pid, signal.SIGKILL)
        try:
            os.killpg(core.pid, signal.SIGKILL)
        except OSError:
            pass
        if master_pid:
            try:
                os.kill(master_pid, signal.SIGKILL)
            except OSError:
                pass


if __name__ == '__main__':
    which = sys.argv[1] if len(sys.argv) > 1 else 'all'
    if which in ('a', 'all'):
        # part A in a child: it fakes rospy functions the real node test needs
        if which == 'all':
            rc = subprocess.call([sys.executable, '-u', __file__, 'a'])
            if rc:
                FAILS.append('part A')
        else:
            part_a()
    if which in ('b', 'all'):
        part_b()
    print('%s (%d failed)' % ('ALL PASS' if not FAILS else 'FAILED', len(FAILS)))
    sys.exit(1 if FAILS else 0)
