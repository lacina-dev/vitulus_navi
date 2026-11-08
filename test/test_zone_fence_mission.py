#!/usr/bin/env python3
"""The mission's side of the zone fence, offline: which zone it announces to
map_rules_guard and how rule_gate reads the guard's answer.

    python3 test/test_zone_fence_mission.py      (sourced workspace)
"""
import json
import os
import sys
import time
import types
import unittest

os.environ['ROS_MASTER_URI'] = 'http://127.0.0.1:1'     # never the robot's master

import rospy  # noqa: E402,F401
from geometry_msgs.msg import Point32  # noqa: E402
from std_msgs.msg import String  # noqa: E402
from vitulus_msgs.msg import MapEditZone  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.join(HERE, '..', 'src')
sys.path.insert(0, SRC)
from mower_unit_smach import helpers, hold_gate, publishers, rule_gate  # noqa: E402
sys.path.remove(SRC)


def zone(name='Bigger', pts=((0, 0), (10, 0), (10, 10)), frame='map'):
    z = MapEditZone()
    z.name = name
    z.polygon.header.frame_id = frame
    z.polygon.polygon.points = [Point32(x=x, y=y, z=0.0) for x, y in pts]
    return z


class Payload(unittest.TestCase):
    def test_zone(self):
        got = json.loads(publishers.zone_payload('Bigger', zone(pts=((0, 0), (1.23456, 0), (1, 1)))))
        self.assertEqual(got, {'program': 'Bigger', 'zone': 'Bigger', 'frame': 'map',
                               'points': [[0.0, 0.0], [1.235, 0.0], [1.0, 1.0]]})

    def test_none_and_no_outline(self):
        self.assertEqual(publishers.zone_payload('P', None), '{}')
        self.assertEqual(publishers.zone_payload('P', zone(pts=((0, 0), (1, 1)))), '{}')

    def test_frame_default(self):
        self.assertEqual(json.loads(publishers.zone_payload('P', zone(frame='')))['frame'], 'map')

    def test_zone_method_publishes(self):
        said = []
        pubs = types.SimpleNamespace(active_zone=types.SimpleNamespace(publish=said.append))
        publishers.MowerPublishers.zone(pubs, 'P', zone())
        publishers.MowerPublishers.zone(pubs)
        self.assertEqual(json.loads(said[0].data)['zone'], 'Bigger')
        self.assertEqual(said[1].data, '{}')


def gate(state):
    g = rule_gate.RuleGate()
    g._cb(String(data=json.dumps(state)))
    return g


class Gate(unittest.TestCase):
    def test_outside_the_zone_is_a_noblade_place(self):
        g = gate({'blade': 'ok', 'in': [], 'zone': {'name': 'A', 'outside': True}})
        self.assertTrue(g.noblade_here())
        self.assertFalse(g.blade_held())         # nothing spins: nothing held

    def test_held_outside(self):
        g = gate({'blade': 'stopped_by_rule', 'in': [],
                  'zone': {'name': 'A', 'outside': True}})
        self.assertTrue(g.blade_held())
        self.assertTrue(g.noblade_here())

    def test_inside_or_no_zone(self):
        self.assertFalse(gate({'blade': 'ok', 'in': [],
                               'zone': {'name': 'A', 'outside': False}}).noblade_here())
        self.assertFalse(gate({'blade': 'ok', 'in': [], 'zone': None}).noblade_here())
        self.assertFalse(gate({'blade': 'ok', 'in': []}).noblade_here())   # older guard

    def test_stale_guard_says_nothing(self):
        g = gate({'blade': 'ok', 'in': [], 'zone': {'outside': True}})
        g._t = time.monotonic() - 10.0
        self.assertFalse(g.noblade_here())


def guard(blade='ok', held_by=None, outside=False):
    return {'blade': blade, 'held_by': held_by, 'in': [],
            'zone': {'name': 'A', 'outside': outside}}


def spin(steps, t0=0.0, dt=0.2):
    """[(guard state, (fw, rpm, setpoint))] -> [waiting]."""
    z = rule_gate.ZoneSpinUp()
    return [z.tick(t0 + i * dt, st, fw) for i, (st, fw) in enumerate(steps)]


OFF = ('READY', 0, 3400)
RAMP = ('RUN', 2100, 3400)
UP = ('RUN', 3380, 3400)


class SpinUp(unittest.TestCase):
    def test_waits_at_the_edge_until_up(self):
        steps = ([(guard('stopped_by_rule', 'zone', True), OFF)] * 3      # outside: drive
                 + [(guard('restoring', 'zone'), OFF)] * 5                # back in, gap
                 + [(guard('ok'), RAMP)] * 5                              # guard's True
                 + [(guard('ok'), UP)] * 2)
        out = spin(steps)
        self.assertEqual(out, [False] * 3 + [True] * 10 + [False] * 2)

    def test_noblade_restoring_is_not_waited_for(self):
        self.assertEqual(spin([(guard('restoring', 'noblade'), OFF)] * 5), [False] * 5)

    def test_rpm_without_setpoint(self):
        steps = [(guard('restoring', 'zone'), OFF), (guard('ok'), ('RUN', 900, 0)),
                 (guard('ok'), ('RUN', 1100, 0))]
        self.assertEqual(spin(steps), [True, True, False])

    def test_lets_go(self):
        start = [(guard('restoring', 'zone'), OFF)]
        for name, then in (('restore failed', (guard('restore_failed'), OFF)),
                           ('fault', (guard('restoring', 'zone'), ('ERR', 0, 3400))),
                           ('temp', (guard('restoring', 'zone'), ('TEMP', 0, 3400))),
                           ('out again', (guard('stopped_by_rule', 'zone', True), OFF)),
                           ('mission said False', (guard('ok'), OFF)),
                           ('guard silent', (None, OFF))):
            self.assertEqual(spin(start + [then]), [True, False], name)

    def test_going_home_ends_the_wait(self):
        z = rule_gate.ZoneSpinUp()
        self.assertTrue(z.tick(0.0, guard('restoring', 'zone'), OFF))
        self.assertFalse(z.tick(0.2, guard('restoring', 'zone'), RAMP, leaving=True))
        self.assertFalse(z.tick(0.4, guard('restoring', 'zone'), RAMP, leaving=True))

    def test_dropped_debt_ends_the_wait(self):
        # stop_reason / home: the guard forgets its debt, nothing spins the blade.
        self.assertEqual(spin([(guard('restoring', 'zone'), OFF), (guard('ok'), OFF)]),
                         [True, False])

    def test_ramp_after_the_guards_true_is_waited_for(self):
        self.assertEqual(spin([(guard('restoring', 'zone'), OFF), (guard('ok'), RAMP),
                               (guard('ok'), UP)]), [True, True, False])

    def test_gives_up_after_max(self):
        steps = [(guard('restoring', 'zone'), OFF)] * 200
        out = spin(steps)
        self.assertTrue(out[0])
        self.assertFalse(out[-1])

    def test_zone_start_behind_the_edge(self):
        # SET_MOTOR_ON behind the edge: the guard answers False, WaitForRpmReached goes
        # on (blade_held), the robot drives in and stands until the blade is up.
        steps = ([(guard('stopped_by_rule', 'zone', True), OFF)] * 2
                 + [(guard('restoring', 'zone'), OFF)] * 2 + [(guard('ok'), UP)])
        self.assertEqual(spin(steps), [False, False, True, True, False])


class Hold(unittest.TestCase):
    def test_rule_wait_holds(self):
        h = hold_gate.HoldGate()
        rospy.rostime.set_rostime_initialized(True)
        h.set_rule_wait(True)
        self.assertTrue(h._held)
        self.assertTrue(h.rule_wait())
        self.assertEqual(h._why('x'), 'Held: blade spinning up')
        h.set_rule_wait(False)
        self.assertFalse(h._held)


class StatusAfterHold(unittest.TestCase):
    def test_mowing_status(self):
        from mower_unit_smach import states
        ud = types.SimpleNamespace(zone_name='A', index_path=2, paths=[1, 2, 3, 4])
        self.assertEqual(states.mowing_status(ud), 'A: exe path 3/4')
        self.assertEqual(states.mowing_status(object()), 'Mowing')

    def test_execute_path_says_it_again(self):
        src = open(os.path.join(HERE, '..', 'src', 'mower_unit_smach', 'states.py')).read()
        i = src.index('HOLD.wait(self, self.pubs.smach_status):\n                        #')
        self.assertIn('mowing_status(userdata)', src[i:i + 300])


class ShutdownOrder(unittest.TestCase):
    """safe_blade_shutdown: the False before the zone ends (the guard drops its
    debt on a zone change)."""

    def setUp(self):
        self.said = []
        said = self.said
        self.pubs = types.SimpleNamespace(
            mower_set_motor_on=types.SimpleNamespace(publish=lambda m: said.append(('motor', m.data))),
            mower_set_height=types.SimpleNamespace(publish=lambda m: said.append(('height', m.data))),
            zone=lambda *a: said.append(('zone', a)))
        self._watch = helpers._power_watch['watch']
        self._wait = helpers.rospy.wait_for_message

    def tearDown(self):
        helpers._power_watch['watch'] = self._watch
        helpers.rospy.wait_for_message = self._wait

    def test_unpowered(self):
        helpers._power_watch['watch'] = types.SimpleNamespace(
            unpowered=lambda program_running=True: (True, 'off'))
        helpers.safe_blade_shutdown('T', pubs=self.pubs)
        self.assertEqual(self.said, [('motor', False), ('zone', ())])

    def test_powered(self):
        helpers._power_watch['watch'] = None
        status = types.SimpleNamespace(status='READY', moto_rpm=0, current_height=80,
                                       max_height=80)
        helpers.rospy.wait_for_message = lambda *a, **k: status
        rospy.rostime.set_rostime_initialized(True)
        helpers.safe_blade_shutdown('T', pubs=self.pubs)
        self.assertEqual(self.said[:2], [('motor', False), ('zone', ())])


class Start(unittest.TestCase):
    def test_node_clears_the_zone_at_start(self):
        src = open(os.path.join(HERE, '..', 'nodes', 'mower_unit_smach')).read()
        main = src[src.index('def main():'):]
        self.assertLess(main.index('pubs.zone()'), main.index('RULES.start(HOLD)'))


if __name__ == '__main__':
    unittest.main()
