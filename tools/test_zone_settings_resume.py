#!/usr/bin/env python3
"""Off-robot test: zone settings and resume from a checkpoint (mower_unit_smach).

Covers
  * GetZoneData: cut height / rpm <= 0 means "not set" - program override
    falls back to the zone, a zone without a value keeps the mower's current
    setting (None = nothing is sent); valid override values still win
  * SET_CUT_HEIGHT / SET_RPM never publish a None value; WaitForRpmReached
    with rpm None accepts any running blade (> 1000 rpm)
  * GetPathData clamps a stale resume window into the path
  * MONITOR_PROGRAM_UNFINISHED: a checkpoint whose zone is gone mows the whole
    program; a path index outside the zone resumes at the zone start
  * CHECK_ACTIVE_MAP rejects a program without zones / a zone without paths
    BEFORE undocking (it used to undock, run nothing and report success)

Real states against fake rospy (no master, no init_node, simulated clock).
Run:  python3 test_zone_settings_resume.py   (needs rospy, smach, vitulus_msgs)
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'src'))

import rospy
import smach
import actionlib
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from vitulus_msgs.msg import PlannerProgram

PUBLISHED = []
CLOCK = [100.0]


class FakePub(object):
    def __init__(self, topic, *a, **kw):
        self.topic = topic

    def publish(self, msg):
        PUBLISHED.append((self.topic, msg))


class FakeSub(object):
    def __init__(self, *a, **kw):
        pass

    def unregister(self):
        pass


class FakeTimer(object):
    def __init__(self, *a, **kw):
        pass

    def shutdown(self):
        pass


def _advance(d):
    CLOCK[0] += d.to_sec() if hasattr(d, 'to_sec') else float(d)


RPM_SCRIPT = []


def fake_wait_for_message(topic, msg_type, timeout=None):
    _advance(0.1)
    if RPM_SCRIPT:
        return RPM_SCRIPT.pop(0) if len(RPM_SCRIPT) > 1 else RPM_SCRIPT[0]
    raise rospy.ROSException('timeout')


rospy.Publisher = FakePub
rospy.Subscriber = FakeSub
rospy.ServiceProxy = lambda *a, **kw: None
rospy.wait_for_message = fake_wait_for_message
rospy.get_param = lambda name, default=None: default
rospy.Timer = FakeTimer
rospy.sleep = _advance
rospy.is_shutdown = lambda: False
rospy.Time.now = staticmethod(lambda: rospy.Time.from_sec(CLOCK[0]))
rospy.get_rostime = lambda: rospy.Time.from_sec(CLOCK[0])
for _name in ('logdebug', 'loginfo', 'logwarn', 'logerr', 'loginfo_throttle'):
    setattr(rospy, _name, lambda *a, **kw: None)
actionlib.SimpleActionClient = lambda *a, **kw: None
smach.set_loggers(lambda m: None, lambda m: None, lambda m: None, lambda m: None)

from mower_unit_smach.publishers import MowerPublishers          # noqa: E402
from mower_unit_smach import mission, states                      # noqa: E402

FAILURES = []


def check(name, ok, detail=''):
    print('%s  %s%s' % ('ok  ' if ok else 'FAIL', name, '' if ok else '  %r' % (detail,)))
    if not ok:
        FAILURES.append(name)


def path(n):
    p = Path()
    for i in range(n):
        ps = PoseStamped()
        ps.pose.position.x = float(i)
        ps.pose.orientation.w = 1.0
        p.poses.append(ps)
    return p


def zone(name, cut_height=0, rpm=0, paths=1, poses=3):
    from vitulus_msgs.msg import MapEditZone
    z = MapEditZone()
    z.name = name
    z.cut_height = cut_height
    z.rpm = rpm
    z.paths = [path(poses) for _ in range(paths)]
    return z


def program(zones, override=False, cut_height=0, rpm=0):
    p = PlannerProgram()
    p.name = 'P'
    p.map_name = 'GARDEN***env*OUTDOOR'   # legacy: CHECK_ACTIVE_MAP needs no site
    p.zone_list = zones
    p.override_zone = override
    p.cut_height = cut_height
    p.rpm = rpm
    return p


def zone_data(prog):
    ud = smach.UserData()
    ud.program = prog
    ud.index = 0
    ud.unfinished_active = False
    ud.unfinished_zone = ''
    ud.unfinished_path = '0'
    ud.unfinished_window = 0
    ud.path_window_start_index = 0
    st = states.GetZoneData(PUBS)
    out = st.execute(ud)
    return out, ud.zone_cut_height, ud.zone_rpm


PUBS = MowerPublishers()


def test_zone_values():
    print('\nGetZoneData: <= 0 = not set')
    out, h, r = zone_data(program([zone('A', 45, 3000)], override=True, cut_height=0, rpm=-1))
    check('override with unset values falls back to the zone', (out, h, r) == ('available', 45, 3000),
          (out, h, r))
    out, h, r = zone_data(program([zone('A', 45, 3000)], override=True, cut_height=60, rpm=3500))
    check('valid override wins', (h, r) == (60, 3500), (h, r))
    out, h, r = zone_data(program([zone('A', 45, 3000)]))
    check('no override = zone values', (h, r) == (45, 3000), (h, r))
    out, h, r = zone_data(program([zone('A', 0, 0)], override=True, cut_height=0, rpm=0))
    check('nothing set anywhere = None (keep the mower setting)', (h, r) == (None, None), (h, r))


def test_wait_rpm_none():
    print('\nWaitForRpmReached with rpm None')
    from vitulus_msgs.msg import Mower as Mower_status
    ud = smach.UserData()
    ud.zone_rpm = None
    slow = Mower_status()
    slow.status = 'RUN'
    slow.moto_rpm = 200
    fast = Mower_status()
    fast.status = 'RUN'
    fast.moto_rpm = 2500
    RPM_SCRIPT[:] = [slow, slow, fast]
    st = states.WaitForRpmReached(PUBS, timeout=10.0)
    check('any running blade counts', st.execute(ud) == 'reached')
    RPM_SCRIPT[:] = [slow]
    check('a blade below 1000 rpm times out', st.execute(ud) == 'timeout')
    ud.zone_rpm = 3000
    RPM_SCRIPT[:] = [fast]
    check('a set rpm still needs its band (2500 vs 3000)', st.execute(ud) == 'timeout')
    RPM_SCRIPT[:] = []


def test_resume_window_clamped():
    print('\nGetPathData: stale resume window clamped')
    prog = program([zone('A', 45, 3000, paths=2, poses=4)])
    ud = smach.UserData()
    ud.program = prog
    ud.index = 0
    ud.index_path = 1
    ud.paths = prog.zone_list[0].paths
    ud.unfinished_active = True
    ud.unfinished_path = '1'
    ud.unfinished_window = 100000
    ud.path_window_start_index = 0
    ud.zone_cut_height = 45
    ud.zone_rpm = 3000
    for k in ('zone_name', 'zone_start_pose', 'unfinished_zone', 'path_chunk',
              'path_start_pose', 'path', 'path_chunk_trim_count', 'restore_height_pending',
              'skipped_spans', 'consecutive_nav_failures'):
        if k not in ud:
            ud[k] = None
    st = states.GetPathData(PUBS)
    try:
        out = st.execute(ud)
    except Exception as exc:                                  # noqa: BLE001
        out = 'raised %r' % (exc,)
    n = len(ud.path.poses) if ud.path is not None else 0
    check('window 100000 (older, longer path) is clamped into the segmented path',
          out == 'available' and n > 0 and ud.path_window_start_index == n - 1,
          (out, ud.path_window_start_index, n))


def unfinished_cb():
    cc = mission.build_wait_for_program(PUBS)
    ms = cc.get_children()['MONITOR_PROGRAM_UNFINISHED']
    return ms._cond_cb


def test_resume_checkpoint_validated():
    print('\nMONITOR_PROGRAM_UNFINISHED: checkpoint must exist')
    cb = unfinished_cb()

    def run(last_result, zones):
        prog = program(zones)
        prog.last_result = last_result
        ud = smach.UserData()
        cb(ud, prog)
        return ud

    ud = run('failed: on_path-A-1-5', [zone('A', paths=3)])
    check('valid checkpoint resumes there',
          ud.unfinished_active and ud.unfinished_zone == 'A' and ud.unfinished_path == '1'
          and ud.unfinished_window == 5, dict(ud))
    ud = run('failed: on_path-GONE-1-5', [zone('A', paths=3)])
    check('zone gone -> whole program', ud.unfinished_active is False, dict(ud))
    ud = run('failed: on_path-A-7-5', [zone('A', paths=3)])
    check('path outside the zone -> zone start',
          ud.unfinished_active and ud.unfinished_path == '0' and ud.unfinished_window == 0,
          dict(ud))


def test_empty_program_rejected():
    print('\nCHECK_ACTIVE_MAP: nothing to mow -> rejected before undocking')
    st = states.CheckActiveMapState(PUBS)

    def run(prog):
        ud = smach.UserData()
        ud.program = prog
        del PUBLISHED[:]
        out = st.execute(ud)
        reasons = [m.data for t, m in PUBLISHED if t.endswith('stop_reason')]
        return out, reasons

    out, reasons = run(program([]))
    check('no zones -> rejected:no_zones', out == 'rejected' and 'rejected:no_zones' in reasons,
          (out, reasons))
    out, reasons = run(program([zone('A', paths=0)]))
    check('zone without paths -> rejected:no_paths',
          out == 'rejected' and 'rejected:no_paths' in reasons, (out, reasons))
    out, reasons = run(program([zone('A', paths=1)]))
    check('legacy program with paths -> ok', out == 'ok', (out, reasons))


if __name__ == '__main__':
    for t in (test_zone_values, test_wait_rpm_none, test_resume_window_clamped,
              test_resume_checkpoint_validated, test_empty_program_rejected):
        try:
            t()
        except Exception as exc:                                 # noqa: BLE001
            import traceback
            traceback.print_exc()
            FAILURES.append('%s raised %r' % (t.__name__, exc))
    print('\n%s' % ('FAILED %d: %s' % (len(FAILURES), FAILURES) if FAILURES else 'all ok'))
    sys.exit(1 if FAILURES else 0)
