#!/usr/bin/env python3
"""T36 E5 (~navsat_async): navsat_transform is launched in a thread of its own
instead of inside the 30 Hz loop that broadcasts map->odom and odom->base_link
(roslaunch takes 1.1-1.9 s; /tf stood still that long).

Drives the real loop30hz of the node (see _t36_replay.py) with a fake roslaunch
whose start() takes 0.4 s:
  - off: the tick that launches navsat blocks for the whole start (as before)
    and the launch runs in the thread of the loop,
  - on: no tick takes longer than 0.1 s, TF goes out during the start, the
    launch runs in another thread, exactly once,
  - what the rest of the node asks ('launch_navsat is not None') answers the
    same during the start as before,
  - a failed launch is retried, not before navsat_async_retry_s,
  - a launch stopped during its start (indoor -> outdoor restart) does not
    stay running beside the next one,
and the launch condition itself (pure function). Only this parameter is
switched on.

Run:  python3 test_t36_e5_navsat_async.py   (needs rospy - robot env)
"""
import os
import sys
import threading
import time
import types

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import _t36_replay as R  # noqa: E402

import rospy  # noqa: E402

ON = {'navsat_async': True}
START_S = 0.4


class FakeParent(object):
    """roslaunch.parent.ROSLaunchParent: start() takes START_S."""
    made = []
    fail = [0]          # this many next starts raise

    def __init__(self, uuid, files):
        self.files = files
        self.thread = None
        self.started = False
        self.stopped = False
        FakeParent.made.append(self)

    def start(self):
        self.thread = threading.current_thread()
        time.sleep(START_S)
        if FakeParent.fail[0] > 0:
            FakeParent.fail[0] -= 1
            raise RuntimeError('fake roslaunch failure')
        self.started = True

    def shutdown(self):
        self.stopped = True


def fake_roslaunch():
    return types.SimpleNamespace(
        parent=types.SimpleNamespace(ROSLaunchParent=FakeParent),
        rlutil=types.SimpleNamespace(get_or_generate_uuid=lambda *a: 'uuid'),
        configure_logging=lambda uuid: None)


def node(nt, params=None):
    """Node with the real init_navsat, an EKF message to broadcast and the
    launch condition met (position and heading fused just now)."""
    R.CLOCK[0] = 9000.0
    n = R.make_node(nt, params)
    del n.init_navsat                    # make_node stubs it: back to the real one
    n.callback_ekf_wheel_nav_odom(R.odom_msg(9000.0, 1.0, 2.0, 0.0, 0.3, 0.0, 0.0))
    n.get_base_link_transform = lambda stamp=None: None
    del FakeParent.made[:]
    FakeParent.fail[0] = 0
    return n


def ticks(n, seconds, keep_fused=True):
    """Run loop30hz in real time for `seconds`; returns [(duration of the tick,
    transforms sent, launch_navsat is not None after it)]."""
    out = []
    end = time.time() + seconds
    while time.time() < end:
        R.CLOCK[0] += 1.0 / 30.0
        if keep_fused:
            n.nav_last_fused_time = rospy.Time.from_sec(R.CLOCK[0])
            n.fix_last_fused_time = rospy.Time.from_sec(R.CLOCK[0])
        n.tf2_TransformBroadcaster.last = None
        a = time.time()
        n.loop30hz()
        d = time.time() - a
        sent = n.tf2_TransformBroadcaster.last
        out.append((d, len(sent) if sent else 0, n.launch_navsat is not None))
        time.sleep(max(0.0, 1.0 / 30.0 - d))
    return out


def wait_idle(n, timeout=3.0):
    end = time.time() + timeout
    while n._navsat_starting and time.time() < end:
        time.sleep(0.01)


def main():
    fails = []

    def check(name, cond, detail=''):
        print('  %-74s %s %s' % (name, 'OK' if cond else 'FAIL', detail))
        if not cond:
            fails.append(name)

    nt = R.load('navi_transform_t36_e5')
    nt.roslaunch = fake_roslaunch()
    me = threading.current_thread()

    print('off (default): as before')
    n = node(nt)
    check('parameter off by default', n._navsat_async is False)
    tk = ticks(n, 0.8)
    longest = max(d for d, _, _ in tk)
    check('the launching tick blocks for the whole start (%.2f s, start %.1f s)'
          % (longest, START_S), longest >= START_S)
    check('... one launch, in the thread of the loop, started',
          len(FakeParent.made) == 1 and FakeParent.made[0].thread is me
          and FakeParent.made[0].started and n.launch_navsat is FakeParent.made[0])
    check('... no launch thread was made', n._navsat_starting is False
          and not [t for t in threading.enumerate() if t.name == 'navsat_launch'])
    check('... the right launch file', FakeParent.made[0].files[0].endswith(
        os.path.join('launch', 'vitulus_navsat.launch')))
    n = node(nt)
    R.CLOCK[0] += 2.0
    ticks(n, 0.3, keep_fused=False)
    check('launch condition not met (fused > 1 s ago): no launch', FakeParent.made == [])

    print('on:')
    n = node(nt, ON)
    tk = ticks(n, 1.0)
    longest = max(d for d, _, _ in tk)
    during = [x for x in tk[1:int(START_S * 30) - 1]]
    check('no tick longer than 0.1 s (longest %.3f s)' % longest, longest < 0.1)
    check('TF goes out in every tick, also during the start (%d ticks)' % len(tk),
          all(sent >= 1 for _, sent, _ in tk))
    check('exactly one launch, in another thread, started',
          len(FakeParent.made) == 1 and FakeParent.made[0].thread is not me
          and FakeParent.made[0].started and not FakeParent.made[0].stopped
          and n.launch_navsat is FakeParent.made[0])
    check('launch_navsat is set during the start, as before (callers see "launched")',
          len(during) > 3 and all(up for _, _, up in during))
    check('the right launch file', FakeParent.made[0].files[0].endswith(
        os.path.join('launch', 'vitulus_navsat.launch')))
    wait_idle(n)
    check('thread done, nothing waits', n._navsat_starting is False)

    n = node(nt, ON)
    R.CLOCK[0] += 2.0
    ticks(n, 0.3, keep_fused=False)
    check('launch condition not met (fused > 1 s ago): no launch', FakeParent.made == [])

    print('on, the launch fails:')
    n = node(nt, dict(ON, navsat_async_retry_s=5.0))
    FakeParent.fail[0] = 1
    tk = ticks(n, 1.0)
    wait_idle(n)
    check('one attempt, navsat not launched, the loop keeps running (longest tick %.3f s)'
          % max(d for d, _, _ in tk), len(FakeParent.made) == 1 and n.launch_navsat is None
          and max(d for d, _, _ in tk) < 0.1)
    check('... the failed launch was shut down', FakeParent.made[0].stopped)
    check('... no retry before navsat_async_retry_s (%.1f s of node time passed)'
          % (R.CLOCK[0] - 9000.0), len(FakeParent.made) == 1 and (R.CLOCK[0] - 9000.0) < 5.0)
    R.CLOCK[0] += 5.0
    ticks(n, 0.8)
    wait_idle(n)
    check('... retried after it and launched', len(FakeParent.made) == 2
          and FakeParent.made[1].started and n.launch_navsat is FakeParent.made[1])

    print('on, stopped during its start (indoor -> outdoor restart of navsat):')
    n = node(nt, ON)
    ticks(n, 0.15)
    first = FakeParent.made[0]
    n.is_indoor = True
    n.callback_navi_indoor(types.SimpleNamespace(data=False))     # shutdown + None
    check('callback_navi_indoor stops it and clears launch_navsat (as before)',
          first.stopped and n.launch_navsat is None)
    ticks(n, 1.2)
    wait_idle(n)
    check('the next launch waits for the first thread, then one more launch',
          len(FakeParent.made) == 2 and FakeParent.made[1].started
          and not FakeParent.made[1].stopped and n.launch_navsat is FakeParent.made[1])

    print('launch condition (pure):')
    f = nt.navsat_async_should_start
    check('fused position and heading < 1 s, not launched: start', f(False, False, 0.2, 0.3, 10.0, 0.0))
    check('already launched: no', not f(True, False, 0.2, 0.3, 10.0, 0.0))
    check('a launch thread is running: no', not f(False, True, 0.2, 0.3, 10.0, 0.0))
    check('position fused 1.0 s ago: no (strict <, as init_navsat)', not f(False, False, 1.0, 0.3, 10.0, 0.0))
    check('heading fused 1.5 s ago: no', not f(False, False, 0.2, 1.5, 10.0, 0.0))
    check('before the retry time: no; at it: yes', not f(False, False, 0.2, 0.3, 10.0, 10.5)
          and f(False, False, 0.2, 0.3, 10.5, 10.5))

    print('\n%s' % ('ALL OK' if not fails else 'FAILED: %d' % len(fails)))
    return 1 if fails else 0


if __name__ == '__main__':
    sys.exit(main())
