#!/usr/bin/env python3
"""A map taken over for real (vitulus-field#46).

navi_man says on /navi_manager/map_ready when it holds the map, and how far the
live pose is from that map's dock. navi_transform and dock_localization_seed use
it to drop the OLD map's DOCKED anchor: on a served-site change active_map stays
the constant 'SITE', so before this the dock hold kept the old map's pose for up
to 30 s and then refused the jump as creep.

Run:  python3 test_map_ready.py   (needs rospy importable — robot env)
"""
import importlib.machinery
import importlib.util
import json
import os
import sys
import types

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import _ros_env  # noqa: E402
_ros_env.ensure()
from std_msgs.msg import String  # noqa: E402

FAILS = []


def check(name, cond, detail=''):
    print('  %-58s %s %s' % (name, 'OK' if cond else 'FAIL', detail))
    if not cond:
        FAILS.append(name)


def load(name, path):
    spec = importlib.util.spec_from_loader(
        name, importlib.machinery.SourceFileLoader(name, path))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def point(name, x=0.0, y=0.0):
    pose = types.SimpleNamespace(position=types.SimpleNamespace(x=x, y=y, z=0.0))
    return types.SimpleNamespace(name=name, pose=pose)


class Tf(object):
    def __init__(self, x=0.0, y=0.0, ok=True):
        self.x, self.y, self.ok = x, y, ok

    def lookup_transform(self, *_a):
        if not self.ok:
            raise RuntimeError('no tf')
        t = types.SimpleNamespace(translation=types.SimpleNamespace(x=self.x, y=self.y))
        return types.SimpleNamespace(transform=t)


def navi_node(served='BACK', points_site='BACK', points=(), legacy=False, loading=False,
              active='SITE', tf=None):
    return types.SimpleNamespace(
        _served_site=served, _points_site=points_site, _legacy_maps=legacy,
        _map_loading=loading, active_map=active, tf2_Buffer=tf or Tf(),
        running_map_data=types.SimpleNamespace(points=list(points)))


def test_map_ready():
    print('navi_man map_ready:')
    Node = load('navi_man_node', os.path.join(HERE, '..', 'nodes', 'navi_man')).Node
    state = Node.map_ready_state
    pts = [point('DOCKED', 10.0, 20.0), point('DOCK', 11.0, 20.0)]
    s = state(navi_node(points=pts, tf=Tf(10.3, 20.4)))
    check('taken over', s['ready'] and s['site'] == 'BACK')
    check('dock error measured', s['dock_err_m'] == 0.5, str(s['dock_err_m']))
    check('has its DOCK', s['has_dock'] and s['has_docked'])
    s = state(navi_node(points_site='LADA', points=pts))
    check('import still pending: not ready', not s['ready'] and s['site'] == 'LADA'
          and s['dock_err_m'] is None)
    s = state(navi_node(served=None, points_site=None))
    check('nothing served: not ready', not s['ready'])
    s = state(navi_node(points=[point('UNDOCK')]))
    check('map without dock points', s['ready'] and not s['has_dock']
          and s['dock_err_m'] is None)
    s = state(navi_node(points=pts, tf=Tf(ok=False)))
    check('no TF: ready, error unknown', s['ready'] and s['dock_err_m'] is None)
    legacy = 'A***env*OUTDOOR'
    s = state(navi_node(legacy=True, loading=True, active=legacy, points=pts))
    check('legacy: load running: not ready', not s['ready'])
    s = state(navi_node(legacy=True, active=legacy, points=pts))
    check('legacy: load done: ready', s['ready'] and s['active_map'] == legacy)
    s = state(navi_node(legacy=True, active='New'))
    check('legacy: new map: not ready', not s['ready'])
    json.dumps(s)


def msg(site):
    return String(json.dumps({'site': site}))


def test_dock_hold():
    print('navi_transform dock anchor:')
    Cls = load('navi_transform_node', os.path.join(HERE, '..', 'nodes', 'navi_transform')).NavTfNode
    node = types.SimpleNamespace(_SITE_UNSET=object(), _dock_anchor_pose=(1.0, 2.0, 0.0),
                                 _dock_anchor_fetch_t=99.0, _dock_hold_active=True,
                                 _dock_anchor_point='DOCKED', _dock_anchor_map='SITE')
    node._dock_anchor_site = node._SITE_UNSET
    node._drop_dock_anchor = lambda what, name: Cls._drop_dock_anchor(node, what, name)
    Cls.cb_map_ready(node, msg('BACK'))
    check('first report keeps the anchor', node._dock_anchor_pose is not None
          and node._dock_hold_active)
    Cls.cb_map_ready(node, msg('BACK'))
    check('same site keeps the anchor', node._dock_anchor_pose is not None)
    Cls.cb_map_ready(node, msg('LADA'))
    check('new site drops the anchor', node._dock_anchor_pose is None
          and node._dock_anchor_fetch_t == 0.0)
    check('next engage is a placement, not creep', node._dock_hold_active is False)
    node._dock_anchor_pose = (3.0, 4.0, 0.0)
    Cls.cb_map_ready(node, String('not json'))
    check('garbage changes nothing', node._dock_anchor_pose is not None)
    Cls.cb_active_map(node, String('SITE'))
    check('constant active_map changes nothing', node._dock_anchor_pose is not None)
    Cls.cb_active_map(node, String('X***env*OUTDOOR'))
    check('legacy map change still drops it', node._dock_anchor_pose is None)


if __name__ == '__main__':
    test_map_ready()
    test_dock_hold()
    print('FAILED: %s' % FAILS if FAILS else 'OK')
    sys.exit(1 if FAILS else 0)
