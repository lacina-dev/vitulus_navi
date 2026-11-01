#!/usr/bin/env python3
"""Stored paths against the map rules: navi_man's refusal and the smach backstop,
and the mission's side of the guard.  No ROS master; needs the sourced workspace.

    python3 test/test_map_rules_paths.py
"""
import importlib.machinery
import importlib.util
import json
import os
import sys
import time
import types
import unittest

os.environ['ROS_MASTER_URI'] = 'http://127.0.0.1:1'     # never the robot's master

import rospy  # noqa: E402
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion  # noqa: E402
from nav_msgs.msg import Path  # noqa: E402
from std_msgs.msg import String  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.join(HERE, '..', 'src')
try:
    import docker  # noqa: F401
except ImportError:
    sys.modules['docker'] = types.ModuleType('docker')

# This tree's packages, then devel again: vitulus_navi.srv is generated there.
sys.path.insert(0, SRC)
from navi_man import path_rules  # noqa: E402
from mower_unit_smach import rule_gate  # noqa: E402
sys.path.remove(SRC)


def load(name):
    path = os.path.join(HERE, '..', 'nodes', name)
    loader = importlib.machinery.SourceFileLoader(name + '_under_test', path)
    spec = importlib.util.spec_from_loader(name + '_under_test', loader)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def box(x0, y0, x1, y1):
    return [[x0, y0], [x1, y0], [x1, y1], [x0, y1]]


def rules_json(*polys):
    return json.dumps({'site': 'A',
                       'rules': [{'type': 'nogo', 'name': 'Bazen', 'points': p} for p in polys]
                       + [{'type': 'slow', 'name': 'S', 'points': box(0, 0, 9, 9)}]})


class Sink:
    def __init__(self):
        self.sent = []

    def publish(self, msg):
        self.sent.append(msg)


class PathRules(unittest.TestCase):
    NOGO = path_rules.parse_rules(rules_json(box(2, -1, 3, 1)))

    def test_only_nogo_is_a_refusal(self):
        self.assertEqual(len(self.NOGO), 1)

    def test_crossing(self):
        self.assertEqual(path_rules.path_refusal([(0, 0), (5, 0)], self.NOGO),
                         'crosses nogo Bazen')

    def test_sparse_path_is_resampled(self):
        thin = path_rules.parse_rules(rules_json(box(2.5, -1, 2.55, 1)))
        self.assertTrue(path_rules.path_refusal([(0, 0), (5, 0)], thin))

    def test_clear(self):
        self.assertIsNone(path_rules.path_refusal([(0, 2), (5, 2)], self.NOGO))

    def test_body_width(self):
        self.assertTrue(path_rules.path_refusal([(0, 1.2), (5, 1.2)], self.NOGO))

    def test_nothing_active_is_nothing(self):
        self.assertIsNone(path_rules.path_refusal([(0, 0), (5, 0)]))

    def test_lethal_edit_line_and_free(self):
        objs = json.dumps({'objects': [
            {'family': 'edit', 'type': 'wall', 'name': 'Plot', 'geometry': 'line',
             'points': [[2, -1], [2, 1]]},
            {'family': 'edit', 'type': 'free', 'name': 'Vrata', 'geometry': 'area',
             'points': box(1, -2, 3, 2)},
            {'family': 'zone', 'type': 'zone', 'name': 'Z', 'points': box(0, 0, 1, 1)}]})
        lethal, free = path_rules.parse_edits(objs)
        self.assertEqual(path_rules.path_refusal([(0, 0), (5, 0)], lethal=lethal),
                         'crosses edit Plot')
        self.assertIsNone(path_rules.path_refusal([(1.5, 0), (2.5, 0)], lethal=lethal, free=free))


class NaviMan(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rospy.rostime.set_rostime_initialized(True)
        cls.M = load('navi_man')

    def node(self, xs):
        n = self.M.Node.__new__(self.M.Node)
        n.running_map_data = self.M.MapData('t')
        path = self.M.MapPath('p', 'map')
        path.poses = [Pose(Point(x, 0, 0), Quaternion(0, 0, 0, 1)) for x in xs]
        n.running_map_data.paths.append(path)
        n.status_msg_pub = Sink()
        n.map_path_goal_arr_pub = Sink()
        n.move_base_client_state = 'READY'
        n.rule_set, n.rule_lethal, n.rule_free = path_rules.RuleSet(), [], []
        n.refuse_lethal_edits = True
        return n

    def test_no_rules_goes_out(self):
        n = self.node([0, 5])
        n.callback_execute_path(String('p'))
        self.assertEqual(len(n.map_path_goal_arr_pub.sent), 1)

    def test_refused_before_publish_with_a_reason(self):
        n = self.node([0, 5])
        n.callback_map_rules(String(rules_json(box(2, -1, 3, 1))))
        n.callback_execute_path(String('p'))
        self.assertEqual(n.map_path_goal_arr_pub.sent, [])
        self.assertEqual(n.status_msg_pub.sent, ['p path refused: crosses nogo Bazen'])

    def test_clear_path_with_rules_goes_out(self):
        n = self.node([0, 1])
        n.callback_map_rules(String(rules_json(box(2, -1, 3, 1))))
        n.callback_execute_path(String('p'))
        self.assertEqual(len(n.map_path_goal_arr_pub.sent), 1)

    def test_bad_rules_do_not_raise(self):
        n = self.node([0, 5])
        n.callback_map_rules(String('garbage'))
        n.callback_execute_path(String('p'))
        self.assertEqual(len(n.map_path_goal_arr_pub.sent), 1)


class Backstop(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.S = load('smach_exe_path')

    def path(self, xs):
        path = Path()
        for x in xs:
            ps = PoseStamped()
            ps.pose.position.x = float(x)
            path.poses.append(ps)
        return path

    def state(self, nogos):
        st = self.S.GetPathData.__new__(self.S.GetPathData)
        st.rule_set = types.SimpleNamespace(nogos=nogos)
        st.check_path = None
        st.status_pub = Sink()
        return st

    def run_state(self, st, xs):
        ud = types.SimpleNamespace(path_to_exe=self.path(xs), path=None, path_start_pose=None)
        rospy.rostime.set_rostime_initialized(True)
        real = self.S.rospy.Publisher
        self.S.rospy.Publisher = lambda *a, **k: Sink()
        try:
            return st.execute(ud), ud
        finally:
            self.S.rospy.Publisher = real

    def test_no_rules_asks_nothing(self):
        def never(_path):
            raise AssertionError('no costmap question without rules')
        self.assertIsNone(self.S.rule_refusal(self.path([0, 5]), [], never))

    def test_refusal_outcome_is_not_available(self):
        nogos = path_rules.parse_rules(rules_json(box(2, -1, 3, 1)))
        st = self.state(nogos)
        outcome, ud = self.run_state(st, [0, 5])
        self.assertEqual(outcome, 'refused')
        self.assertIsNone(ud.path)
        self.assertEqual(st.status_pub.sent, ['Path refused: crosses nogo Bazen'])

    def test_lethal_index_refuses_only_with_rules(self):
        nogos = path_rules.parse_rules(rules_json(box(2, 0.5, 3, 2)))   # beside the path
        self.assertIn('lethal', self.S.rule_refusal(self.path([0, 2.5, 5]), nogos, lambda p: 0))
        self.assertIsNone(self.S.rule_refusal(self.path([0, 2.5, 5]), nogos, lambda p: -1))

    def test_a_nogo_elsewhere_asks_the_costmap_nothing(self):
        far = path_rules.parse_rules(rules_json(box(20, 20, 21, 21)))

        def never(_path):
            raise AssertionError('a live obstacle far from any nogo is not a rule')
        self.assertIsNone(self.S.rule_refusal(self.path([0, 5]), far, never))

    def test_only_the_poses_near_the_nogo_are_asked_about(self):
        near = path_rules.parse_rules(rules_json(box(2, 0.5, 3, 2)))
        asked = []

        def lethal(path):
            asked.extend(p.pose.position.x for p in path.poses)
            return -1
        self.S.rule_refusal(self.path([0.0, 2.5, 4.9, 9.0]), near, lethal)
        self.assertEqual(asked, [2.5])

    def slow_state(self):
        st = self.state([])
        st.CHECK_TIMEOUT_S = 0.2
        st.check_path = lambda req: time.sleep(5.0)
        return st

    def test_l10_a_hanging_service_does_not_hang_the_state(self):
        st = self.slow_state()
        st.preempt_requested = lambda: False
        t0 = time.monotonic()
        self.assertEqual(st.lethal_index(self.path([0, 1])), -1)
        self.assertLess(time.monotonic() - t0, 1.0)

    def test_l10_a_preempt_ends_the_wait(self):
        st = self.slow_state()
        st.CHECK_TIMEOUT_S = 4.0
        st.preempt_requested = lambda: True
        t0 = time.monotonic()
        self.assertEqual(st.lethal_index(self.path([0, 1])), -1)
        self.assertLess(time.monotonic() - t0, 1.0)

    def test_refused_goes_to_wait_for_path(self):
        with open(os.path.join(HERE, '..', 'nodes', 'smach_exe_path')) as handle:
            src = handle.read()
        self.assertIn("'refused': 'WAIT_FOR_PATH'", src)


class Sites(unittest.TestCase):
    A = rules_json(box(2, -1, 3, 1))

    def test_rules_of_the_served_site(self):
        rs = path_rules.RuleSet()
        rs.on_rules(self.A)
        rs.on_status(json.dumps({'serving': {'site': 'A'}}))
        self.assertEqual(len(rs.nogos), 1)

    def test_another_site_is_dropped(self):
        rs = path_rules.RuleSet()
        rs.on_rules(self.A)
        rs.on_status(json.dumps({'serving': {'site': 'B'}}))
        self.assertEqual(rs.nogos, [])

    def test_site_null_and_no_rules_key_keep_the_last(self):
        rs = path_rules.RuleSet()
        rs.on_rules(self.A)
        rs.on_rules(json.dumps({'site': None, 'rules': []}))
        rs.on_rules(json.dumps({'site': 'A'}))
        rs.on_rules('garbage')
        self.assertEqual(len(rs.nogos), 1)


class RuleGate(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.G = rule_gate

    def gate(self, state):
        g = self.G.RuleGate()
        if state is not None:
            g._cb(String(json.dumps(state)))
        return g

    def test_no_guard_changes_nothing(self):
        g = self.gate(None)
        self.assertFalse(g.capped_below(0.2))
        self.assertFalse(g.blade_held())

    def test_no_rules_changes_nothing(self):
        g = self.gate({'limit': {'rules': 0, 'v_fwd': 0.0, 'v_back': 0.0}, 'blade': 'ok'})
        self.assertFalse(g.capped_below(0.2))
        self.assertFalse(g.blade_held())

    def test_cap_and_blade(self):
        g = self.gate({'limit': {'rules': 1, 'v_fwd': 0.1, 'v_back': 0.1},
                       'blade': 'stopped_by_rule'})
        self.assertTrue(g.capped_below(0.2))
        self.assertFalse(g.capped_below(0.05))
        self.assertTrue(g.blade_held())

    def test_old_state_is_no_state(self):
        g = self.gate({'limit': {'rules': 1, 'v_fwd': 0.1, 'v_back': 0.1},
                       'blade': 'stopped_by_rule'})
        g._t = time.monotonic() - 3.0
        self.assertFalse(g.capped_below(0.2))
        self.assertFalse(g.blade_held())

    def test_the_cap_of_the_direction_of_travel(self):
        g = self.gate({'limit': {'rules': 1, 'v_fwd': 0.0, 'v_back': 1.4}})
        self.assertTrue(g.capped_below(0.2))
        self.assertFalse(g.capped_below(0.2, reverse=True))

    def test_noblade_here(self):
        self.assertFalse(self.gate(None).noblade_here())
        self.assertFalse(self.gate({'in': [{'type': 'slow'}], 'blade': 'ok'}).noblade_here())
        self.assertTrue(self.gate({'in': [{'type': 'noblade'}], 'blade': 'ok'}).noblade_here())
        self.assertTrue(self.gate({'in': [], 'blade': 'stopped_by_rule'}).noblade_here())

    def test_blade_failed(self):
        self.assertTrue(self.gate({'blade': 'restore_failed'}).blade_failed())
        self.assertFalse(self.gate({'blade': 'ok'}).blade_failed())
        self.assertFalse(self.gate(None).blade_failed())

    def simulate(self, speed, capped):
        """Stall test at `speed`, sampled at 0.5 s for 15 s: longest time 'not moving'."""
        anchor = self.G.StallAnchor()
        eps = 0.08
        last, last_move, worst = (0.0, 0.0), 0.0, 0.0
        for i in range(1, 31):
            t = i * 0.5
            cur = (speed * t, 0.0)
            moved = anchor.moved(last, cur, cur[0] - last[0], capped)
            if moved > eps:
                last_move = t
                anchor.reset()
            last = cur
            worst = max(worst, t - last_move)
        return worst

    def test_slow_zone_is_no_stall(self):
        self.assertLess(self.simulate(0.05, True), 5.0)
        self.assertLess(self.simulate(0.10, True), 5.0)

    def test_the_old_test_would_have_stalled(self):
        self.assertGreaterEqual(self.simulate(0.10, False), 5.0)

    def test_uncapped_is_the_sample_itself(self):
        anchor = self.G.StallAnchor()
        self.assertEqual(anchor.moved((0, 0), (9, 9), 0.123, False), 0.123)

    def test_a_real_stop_under_a_cap_still_stalls(self):
        self.assertGreaterEqual(self.simulate(0.0, True), 5.0)


class Terminal(unittest.TestCase):
    def test_the_latch_reason_goes_to_terminal_error(self):
        sys.path.insert(0, SRC)
        from mower_unit_smach import mission, stale_blade
        sys.path.remove(SRC)
        latch = types.SimpleNamespace(reason=stale_blade.TERMINAL_REASON)
        ud = types.SimpleNamespace(error_reason='')
        self.assertTrue(mission.rules_terminal(latch, ud))
        self.assertEqual(ud.error_reason, 'MAP_RULES_FROZEN')

    def test_a_window_reason_does_not(self):
        sys.path.insert(0, SRC)
        from mower_unit_smach import mission
        sys.path.remove(SRC)
        ud = types.SimpleNamespace(error_reason='')
        self.assertFalse(mission.rules_terminal(types.SimpleNamespace(reason='window'), ud))
        self.assertEqual(ud.error_reason, '')

    def test_preempted_is_the_way_to_terminal_error(self):
        with open(os.path.join(HERE, '..', 'nodes', 'mower_unit_smach')) as handle:
            src = handle.read()
        cc = src[src.index("smach.StateMachine.add('MISSION_CONCURRENCE'"):]
        self.assertIn("'preempted': 'TERMINAL_ERROR'", cc[:cc.index('# ====')])


class MissionSeesTheBlade(unittest.TestCase):
    def test_a_failed_blade_is_blocked_with_a_reason(self):
        sys.path.insert(0, SRC)
        from mower_unit_smach import states
        sys.path.remove(SRC)
        rospy.rostime.set_rostime_initialized(True)
        st = states.ExecutePathWithFeedback.__new__(states.ExecutePathWithFeedback)
        st._preempt_requested = False
        st.pubs = types.SimpleNamespace(log_info=Sink(), smach_status=Sink())
        st.last_feedback = None
        st._feedback_sub = None
        st._mower_status = None
        st.STALL_SAMPLE_DT, st.STALL_TIME = 0.5, 5.0
        st.STALL_MOVE_EPS = 0.08
        calls = []
        st._client = types.SimpleNamespace(
            send_goal=lambda goal: None, get_state=lambda: 1,
            cancel_goal=lambda: calls.append('cancel'),
            wait_for_result=lambda t: True)
        real = (states.RULES, states.get_robot_xy_in_map, states.rospy.Subscriber)
        states.RULES = types.SimpleNamespace(blade_failed=lambda: True,
                                             capped_below=lambda *a, **k: False)
        states.get_robot_xy_in_map = lambda timeout=0.3: (0.0, 0.0)
        states.rospy.Subscriber = lambda *a, **k: types.SimpleNamespace(unregister=lambda: None)
        chunk = Path()
        chunk.poses = [PoseStamped(), PoseStamped()]
        ud = types.SimpleNamespace(path_chunk=chunk, path=chunk, paths=[chunk],
                                   path_window_start_index=0, zone_name='Z', index_path=0)
        try:
            self.assertEqual(st.execute(ud), 'blocked')
        finally:
            states.RULES, states.get_robot_xy_in_map, states.rospy.Subscriber = real
        self.assertEqual(calls, ['cancel'])
        self.assertIn('blade', st.pubs.log_info.sent[-1].data.lower())


class HoldSeesTheLimit(unittest.TestCase):
    def gate(self):
        sys.path.insert(0, SRC)
        from mower_unit_smach import hold_gate
        sys.path.remove(SRC)
        rospy.rostime.set_rostime_initialized(True)
        return hold_gate.HoldGate()

    def status(self, stale):
        return String(json.dumps({'limit': None if stale is None else {'stale': stale}}))

    def test_stale_is_held(self):
        h = self.gate()
        h._status_cb(self.status(True))
        self.assertTrue(h._held)
        h._status_cb(self.status(False))
        self.assertFalse(h._held)

    def test_no_limit_is_not_held(self):
        h = self.gate()
        h._status_cb(self.status(None))
        self.assertFalse(h._held)

    def test_hold_and_stale_end_separately(self):
        h = self.gate()
        h._cb(types.SimpleNamespace(data=True))
        h._status_cb(self.status(True))
        h._status_cb(self.status(False))
        self.assertTrue(h._held, 'the mux hold still stands')
        h._cb(types.SimpleNamespace(data=False))
        self.assertFalse(h._held)

    def test_garbage_status_changes_nothing(self):
        h = self.gate()
        h._status_cb(String('garbage'))
        self.assertFalse(h._held)


if __name__ == '__main__':
    unittest.main()
