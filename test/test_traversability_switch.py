#!/usr/bin/env python3
"""Traversability switch of vitulus_navi_mbf.launch (README "Traversability layer"): off unless both
VITULUS_TRAVERSABILITY_NAV and VITULUS_TRAVERSABILITY are on; off = parameters exactly as without the switch; on =
traversability_layer before inflation_layer, its parameters inside /move_base_flex. Also keeps the plugin list of
config/local_costmap_traversability.yaml in step with config/local_costmap_params.yaml. No ROS master.

    python3 test/test_traversability_switch.py      (ROS Noetic, e.g. vitulus-dev python3 test/...)
"""
import os
import unittest

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)
# this tree's vitulus_navi first ($(find vitulus_navi) in the launch file)
os.environ['ROS_PACKAGE_PATH'] = os.path.dirname(PKG) + ':' + os.environ.get('ROS_PACKAGE_PATH', '/opt/ros/noetic/share')
os.environ['ROS_MASTER_URI'] = 'http://127.0.0.1:1'     # never the robot's master

import roslaunch.config  # noqa: E402
import rospkg  # noqa: E402

LAUNCH = os.path.join(PKG, 'launch', 'vitulus_navi_mbf.launch')
NS = '/move_base_flex/local_costmap/'


def load_params(env):
    saved = {k: os.environ.get(k) for k in ('VITULUS_TRAVERSABILITY', 'VITULUS_TRAVERSABILITY_NAV')}
    try:
        for k in saved:
            os.environ.pop(k, None)
        os.environ.update(env)
        cfg = roslaunch.config.load_config_default([(LAUNCH, ['map_topic:=/navi_manager/map'])], None, verbose=False)
        return {k: p.value for k, p in cfg.params.items()}
    finally:
        for k, v in saved.items():
            if v is None:
                os.environ.pop(k, None)
            else:
                os.environ[k] = v


def names(plugins):
    return [p['name'] for p in plugins]


class Switch(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.assert_this_tree = rospkg.RosPack().get_path('vitulus_navi')
        cls.base = load_params({})

    def test_this_tree(self):
        self.assertEqual(os.path.realpath(self.assert_this_tree), os.path.realpath(PKG))

    def test_off_is_unchanged(self):
        with open(os.path.join(PKG, 'config', 'local_costmap_params.yaml')) as f:
            yaml_plugins = yaml.safe_load(f)['local_costmap']['plugins']
        self.assertEqual(self.base[NS + 'plugins'], yaml_plugins)
        self.assertFalse(any('traversability' in k for k in self.base))
        # only one of the two, a typo, "false": all the same as unset
        for env in ({'VITULUS_TRAVERSABILITY_NAV': 'true'}, {'VITULUS_TRAVERSABILITY': 'true'},
                    {'VITULUS_TRAVERSABILITY': 'true', 'VITULUS_TRAVERSABILITY_NAV': 'ture'},
                    {'VITULUS_TRAVERSABILITY': 'true', 'VITULUS_TRAVERSABILITY_NAV': 'false'},
                    {'VITULUS_TRAVERSABILITY': '', 'VITULUS_TRAVERSABILITY_NAV': ''}):
            self.assertEqual(load_params(env), self.base, env)

    def test_on_adds_the_layer_only(self):
        for val in ('true', '1', ' ON ', 'Yes'):
            p = load_params({'VITULUS_TRAVERSABILITY': 'true', 'VITULUS_TRAVERSABILITY_NAV': val})
            want = names(self.base[NS + 'plugins'])
            want.insert(want.index('inflation_layer'), 'traversability_layer')
            self.assertEqual(names(p[NS + 'plugins']), want, val)
            self.assertEqual(p[NS + 'traversability_layer/map_topic'], '/traversability/local_costmap')
            self.assertIs(p[NS + 'traversability_layer/use_maximum'], True)
            self.assertIs(p[NS + 'traversability_layer/track_unknown_space'], False)
            self.assertIs(p[NS + 'traversability_layer/trinary_costmap'], False)
            self.assertEqual(p[NS + 'traversability_layer/lethal_cost_threshold'], 100)
            extra = {k for k in p if k not in self.base}
            self.assertTrue(all(k.startswith(NS + 'traversability_layer/') for k in extra), extra)
            changed = {k for k in self.base if p[k] != self.base[k]}
            self.assertEqual(changed, {NS + 'plugins'})

    def test_plugin_lists_in_step(self):
        with open(os.path.join(PKG, 'config', 'local_costmap_params.yaml')) as f:
            base = yaml.safe_load(f)['local_costmap']['plugins']
        with open(os.path.join(PKG, 'config', 'local_costmap_traversability.yaml')) as f:
            trav = yaml.safe_load(f)['local_costmap']['plugins']
        want = list(base)
        want.insert(names(base).index('inflation_layer'),
                    {'name': 'traversability_layer', 'type': 'costmap_2d::StaticLayer'})
        self.assertEqual(trav, want, 'update config/local_costmap_traversability.yaml with local_costmap_params.yaml')


if __name__ == '__main__':
    unittest.main(verbosity=2)
