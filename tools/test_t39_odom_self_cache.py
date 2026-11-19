#!/usr/bin/env python3
"""T39 (~odom_self_cache): the node keeps the /odometry/odom message it
publishes instead of subscribing to its own topic.

Through the node's real __init__ and loop30hz (see _t36_replay.py, no master):
  - off (default): the subscription to /odometry/odom is made as before and
    publishing does not touch base_odom_msg;
  - on, outdoor: no subscription; after every tick base_odom_msg IS the message
    just published - with ~odom_twist_from_wheels the copy with the measured
    twist, not the EKF message; a late message from the old subscription is
    ignored;
  - on, indoor (the node does not publish, the indoor EKF does): subscribed,
    from start and again after is_indoor True; unregistered on is_indoor False;
  - /tf, /odometry/odom and nav_fused are the same with the switch off and on.

Run:  python3 tools/test_t39_odom_self_cache.py   (needs rospy - robot env)
"""
import io
import os
import sys
import types
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import _t36_replay as R  # noqa: E402

import rospy  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402

SUBS = []


class Sub(object):
    def __init__(self, topic, *a, **k):
        self.topic, self.alive = topic, True
        SUBS.append(self)

    def unregister(self):
        self.alive = False


def odom_subs():
    return [s for s in SUBS if s.topic == '/odometry/odom' and s.alive]


def wire(m):
    b = io.BytesIO()
    m.serialize(b)
    return b.getvalue()


class SelfCache(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.nt = R.load('nt_t39_odom')
        cls.data = R.load_data()

    def make(self, **params):
        """The node through its real __init__ with a recording Subscriber."""
        del SUBS[:]
        R.PARAMS.clear()
        R.PARAMS.update(params)
        old = rospy.Subscriber
        rospy.Subscriber = Sub
        self.addCleanup(setattr, rospy, 'Subscriber', old)
        return self.nt.NavTfNode()

    def test_off_subscribes_as_before(self):
        n = self.make()
        self.assertFalse(n._odom_self_cache)
        self.assertEqual(len(odom_subs()), 1)
        n.callback_navi_indoor(types.SimpleNamespace(data=False))
        self.assertEqual(len(odom_subs()), 1)
        m = Odometry()
        n.callback_base_odom(m)
        self.assertIs(n.base_odom_msg, m)

    def test_on_subscribed_only_indoor(self):
        n = self.make(odom_self_cache=True)
        self.assertTrue(n.is_indoor)
        self.assertEqual(len(odom_subs()), 1)          # start: indoor until told
        m = Odometry()
        n.callback_base_odom(m)
        self.assertIs(n.base_odom_msg, m)
        n.callback_navi_indoor(types.SimpleNamespace(data=False))
        self.assertEqual(len(odom_subs()), 0)
        self.assertIsNone(n.base_odom_sub)
        n.callback_base_odom(Odometry())               # late message: ignored
        self.assertIs(n.base_odom_msg, m)
        n.callback_navi_indoor(types.SimpleNamespace(data=False))
        self.assertEqual(len(odom_subs()), 0)
        n.callback_navi_indoor(types.SimpleNamespace(data=True))
        self.assertEqual(len(odom_subs()), 1)
        n.callback_navi_indoor(types.SimpleNamespace(data=True))
        self.assertEqual(len(odom_subs()), 1)

    def run_window(self, **params):
        seen = []

        def hook(n, t, kind):
            if kind == 'ob':
                seen.append((n.base_odom_msg is n.odometry_odom_pub.last,
                             n.odometry_odom_pub.last is n.ekf_wheel_nav_odom_msg))

        keep = {}

        def setup(n):
            for name in ('odometry_odom_pub', 'fused_nav_pub'):
                getattr(n, name).keep = keep.setdefault(name, [])
        tr, n = R.replay(self.nt, self.data, 315.0, 333.0, params=params, hook=hook, setup=setup)
        return tr, n, seen, {k: [wire(m) for m in v] for k, v in keep.items()}

    def test_on_keeps_the_published_message(self):
        for twist in (False, True):
            tr, n, seen, _ = self.run_window(odom_self_cache=True, odom_twist_from_wheels=twist)
            self.assertGreater(len(seen), 400)
            self.assertTrue(all(s[0] for s in seen))
            if twist:       # the copy with the measured twist, not the EKF message
                self.assertGreater(sum(1 for s in seen if not s[1]), 400)
            else:
                self.assertTrue(all(s[1] for s in seen))

    def test_off_publish_does_not_touch_the_cache(self):
        tr, n, seen, _ = self.run_window(odom_twist_from_wheels=True)
        self.assertFalse(any(s[0] for s in seen))

    def test_outputs_equal_off_and_on(self):
        a = self.run_window(odom_twist_from_wheels=True)
        b = self.run_window(odom_twist_from_wheels=True, odom_self_cache=True)
        for k in a[0]:
            self.assertTrue((a[0][k] == b[0][k]).all() if hasattr(a[0][k], 'all') else a[0][k] == b[0][k], k)
        self.assertEqual(a[3], b[3])
        self.assertGreater(len(a[3]['odometry_odom_pub']), 400)


if __name__ == '__main__':
    unittest.main()
