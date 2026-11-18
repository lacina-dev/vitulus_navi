#!/usr/bin/env python3
"""Trial of the corridor preference with the real planner on a PRIVATE master (T36).

    source <devel with libvitulus_navi_corridor_layer.so>/setup.bash
    python3 tools/corridor_plan_trial.py --world live.pkl --out result.json [--port 11457]

Never the robot's master: starts its own roscore on --port (refuses 11311), its own
move_base_flex from launch/vitulus_navi_mbf.launch (the robot's configuration), once
without the switch (today) and once with corridor_prefer:=true, and stops both by PID.

world = pickle {'/navi_manager/map': {frame, w, h, res, ox, oy, q, data(bytes)},
                '/map_objects/objects': json} read once from the robot.
The corridor grid is built by navi_man's own code (navi_man.map_corridor.Corridors).

For every start - goal pair the plan of mbf get_path: length, share of it inside
a corridor, and how far it is from the plan without the switch (dev_m; the bytes
differ even without a corridor: the planner seeds its start with neutral_cost).
"""
import argparse
import hashlib
import json
import math
import os
import pickle
import signal
import struct
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'src'))
from navi_man import map_corridor  # noqa: E402
sys.path.pop(0)

import numpy as np  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--world', required=True)
    ap.add_argument('--out', required=True)
    ap.add_argument('--port', type=int, default=11457)
    ap.add_argument('--cost', type=int, default=7)
    ap.add_argument('--dock-radius', type=float, default=2.5)
    args = ap.parse_args()
    if args.port == 11311:
        sys.exit('not the robot\'s master')
    os.environ['ROS_MASTER_URI'] = 'http://127.0.0.1:%d' % args.port
    os.environ['ROS_IP'] = '127.0.0.1'
    os.environ.pop('ROS_HOSTNAME', None)
    with open(args.world, 'rb') as f:
        world = pickle.load(f)

    procs = []

    def start(cmd, log):
        p = subprocess.Popen(cmd, stdout=open(log, 'w'), stderr=subprocess.STDOUT, preexec_fn=os.setsid)
        procs.append(p)
        return p

    def stop(p):
        if p.poll() is None:
            os.killpg(p.pid, signal.SIGINT)
            try:
                p.wait(20)
            except subprocess.TimeoutExpired:
                os.killpg(p.pid, signal.SIGKILL)
        if p in procs:
            procs.remove(p)

    logdir = os.path.dirname(os.path.abspath(args.out))
    core = start(['roscore', '-p', str(args.port)], os.path.join(logdir, 'trial_roscore.log'))
    try:
        result = run(args, world, start, stop, logdir)
        with open(args.out, 'w') as f:
            json.dump(result, f, indent=1)
    finally:
        for p in list(procs):
            if p is not core:
                stop(p)
        stop(core)


def run(args, world, start, stop, logdir):
    import actionlib
    import rosgraph
    import rospy
    import tf2_ros
    from geometry_msgs.msg import TransformStamped
    from mbf_msgs.msg import GetPathAction, GetPathGoal
    from nav_msgs.msg import OccupancyGrid
    from sensor_msgs.msg import LaserScan

    for _ in range(100):
        if rosgraph.is_master_online():
            break
        time.sleep(0.2)
    rospy.init_node('corridor_trial', disable_signals=True)
    m = world['/navi_manager/map']
    base = np.frombuffer(m['data'], dtype=np.int8).reshape(m['h'], m['w'])

    def grid_msg(arr, frame='map', res=None, ox=None, oy=None):
        g = OccupancyGrid()
        g.header.frame_id = frame
        g.info.height, g.info.width = arr.shape
        g.info.resolution = m['res'] if res is None else res
        g.info.origin.position.x = m['ox'] if ox is None else ox
        g.info.origin.position.y = m['oy'] if oy is None else oy
        g.info.origin.orientation.w = 1.0
        g.data = arr.ravel().tolist()
        return g

    map_pub = rospy.Publisher('/navi_manager/map', OccupancyGrid, queue_size=1, latch=True)
    cor_pub = rospy.Publisher(map_corridor.TOPIC, OccupancyGrid, queue_size=1, latch=True)
    small = np.zeros((40, 40), dtype=np.int8)
    keep = []
    for t in ('/local_costmap/lidar_map', '/local_costmap/cloud_map'):      # the local costmap waits for them
        p = rospy.Publisher(t, OccupancyGrid, queue_size=1, latch=True)
        p.publish(grid_msg(small, 'odom', 0.05, -1.0, -1.0))
        keep.append(p)
    scan_pub = rospy.Publisher('/rplidar/scan_5m', LaserScan, queue_size=1)

    objs = json.loads(world['/map_objects/objects'])
    wp = {o['name']: tuple(o['points'][0]) for o in objs['objects'] if o['type'] == 'waypoint'}
    robot = wp.get('UNDOCK', (0.0, 0.0))
    br = tf2_ros.StaticTransformBroadcaster()
    tfs = []
    for parent, child, xy in (('map', 'odom', (0.0, 0.0)), ('odom', 'base_link', robot)):
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id, t.child_frame_id = parent, child
        t.transform.translation.x, t.transform.translation.y = xy
        t.transform.rotation.w = 1.0
        tfs.append(t)
    br.sendTransform(tfs)

    def scan(_e):       # a live sensor: the global costmap updates its usual window every cycle
        s = LaserScan()
        s.header.stamp, s.header.frame_id = rospy.Time.now(), 'base_link'
        s.angle_min, s.angle_max, s.angle_increment = -math.pi, math.pi, math.pi / 180
        s.range_min, s.range_max = 0.1, 6.0
        s.ranges = [float('inf')] * 361
        scan_pub.publish(s)
    rospy.Timer(rospy.Duration(0.1), scan)

    info = grid_msg(np.zeros((1, 1), dtype=np.int8)).info
    info.height, info.width = base.shape

    def cell(x, y):
        return int((y - m['oy']) / m['res']), int((x - m['ox']) / m['res'])

    def corridors_of(objects_json):
        return map_corridor.parse(objects_json)[1]

    real = corridors_of(world['/map_objects/objects'])

    class Mbf(object):
        def __init__(self, switch):
            self.p = start(['roslaunch', 'vitulus_navi', 'vitulus_navi_mbf.launch', 'map_topic:=/navi_manager/map',
                            'corridor_prefer:=%s' % ('true' if switch else 'false')],
                           os.path.join(logdir, 'trial_mbf_%s.log' % ('on' if switch else 'off')))
            self.ac = actionlib.SimpleActionClient('/move_base_flex/get_path', GetPathAction)
            if not self.ac.wait_for_server(rospy.Duration(90)):
                raise RuntimeError('move_base_flex did not come up')
            self.pid = None
            for _ in range(50):
                out = subprocess.run(['rosnode', 'info', '/move_base_flex'], capture_output=True, text=True).stdout
                for line in out.splitlines():
                    if line.startswith('Pid:'):
                        self.pid = int(line.split()[1])
                if self.pid:
                    break
                time.sleep(0.2)

        def cpu(self):
            with open('/proc/%d/stat' % self.pid) as f:
                v = f.read().rsplit(')', 1)[1].split()
            return (int(v[11]) + int(v[12])) / float(os.sysconf('SC_CLK_TCK'))

        def plan(self, a, b):
            g = GetPathGoal()
            g.use_start_pose = True
            for pose, (x, y) in ((g.start_pose, a), (g.target_pose, b)):
                pose.header.frame_id = 'map'
                pose.pose.position.x, pose.pose.position.y = x, y
                pose.pose.orientation.w = 1.0
            g.tolerance = 0.3
            t0 = time.time()
            self.ac.send_goal(g)
            self.ac.wait_for_result(rospy.Duration(30))
            r = self.ac.get_result()
            dt = time.time() - t0
            if r is None or r.outcome != 0:
                return {'outcome': None if r is None else r.outcome, 'message': '' if r is None else r.message}
            xy = [(p.pose.position.x, p.pose.position.y) for p in r.path.poses]
            raw = b''.join(struct.pack('<7d', p.pose.position.x, p.pose.position.y, p.pose.position.z,
                                       p.pose.orientation.x, p.pose.orientation.y, p.pose.orientation.z,
                                       p.pose.orientation.w) for p in r.path.poses)
            return {'outcome': 0, 'xy': xy, 'n': len(xy), 'sha': hashlib.sha1(raw).hexdigest(),
                    'len_m': sum(math.hypot(q[0] - p[0], q[1] - p[1]) for p, q in zip(xy, xy[1:])), 'plan_s': dt}

        def costmap(self):
            c = rospy.wait_for_message('/move_base_flex/global_costmap/costmap', OccupancyGrid, timeout=30)
            return np.array(c.data, dtype=np.int8).reshape(c.info.height, c.info.width)

    def summary(p, mask=None, static=None):
        if p.get('outcome') != 0:
            return p
        out = {k: p[k] for k in ('n', 'sha', 'plan_s')}
        out['len_m'] = round(p['len_m'], 2)
        cells = [cell(x, y) for x, y in p['xy']]
        if mask is not None:
            out['in_corridor'] = round(sum(bool(mask[c]) for c in cells) / float(len(cells)), 3)
        if static is not None:
            out['on_obstacle'] = int(sum(static[c] == 100 for c in cells))
        return out

    def set_map(arr):
        map_pub.publish(grid_msg(arr))

    def set_corridors(corridors, cost, mbf=None, probe=None):
        """navi_man's code builds the grid for the map on the topic."""
        c = map_corridor.Corridors(cost, args.dock_radius)
        c.corridors, c.site = corridors, 'x'
        c.dock = map_corridor.parse(world['/map_objects/objects'])[2]
        c.info, c.frame = info, 'map'
        t0 = time.time()
        out = c.build(OccupancyGrid, None)
        build_ms = (time.time() - t0) * 1e3
        t0 = time.time()
        cor_pub.publish(out)
        pub_ms = (time.time() - t0) * 1e3
        time.sleep(3.5)                                  # > 2 cycles of the global costmap (1 Hz)
        return {'build_ms': round(build_ms, 1), 'publish_ms': round(pub_ms, 1), 'corridor_cells': c.cells,
                'cells': len(out.data)}

    # ---- the pairs ---------------------------------------------------------
    pairs = [
        ('UNDOCK->zadni-vrata (koridor vede k cili)', wp['UNDOCK'], wp['zadni-vrata']),
        ('UNDOCK->Jirka (koridor vede k cili)', wp['UNDOCK'], wp['Jirka']),
        ('zadni-vrata->TEST (zpet)', wp['zadni-vrata'], wp['TEST']),
        ('napric pasem pres koridor (-12,-45)->(-3,-48)', (-12.0, -45.0), (-3.0, -48.0)),
        ('podel, 5 m vedle koridoru (-3,-45)->(-6.5,-55)', (-3.0, -45.0), (-6.5, -55.0)),
        ('kratky skok TEST->UNDOCK', wp['TEST'], wp['UNDOCK']),
    ]
    real_mask = map_corridor.mask(real, info)

    # a dog-leg of a given detour in open space (1.4 m from anything in the map): S -> M -> G, L = 8 m
    S, G = (-21.0, -75.0), (-23.0, -82.75)
    L = math.hypot(G[0] - S[0], G[1] - S[1])
    ux, uy = (G[0] - S[0]) / L, (G[1] - S[1]) / L
    ratios = (1.05, 1.10, 1.15, 1.20, 1.25, 1.30, 1.40)

    def dogleg(r):
        h = 0.5 * L * math.sqrt(r * r - 1.0)
        M = ((S[0] + G[0]) / 2 + uy * h, (S[1] + G[1]) / 2 - ux * h)
        return (('dogleg', 0.6, (S, M, G)),)

    # a wall across the corridor terasa-dozadu near (-9.2, -42.4): 4 m long, 0.4 m thick
    blocked = base.copy()
    j0, i0 = cell(-11.2, -42.6)
    blocked[j0:j0 + 8, i0:i0 + 80] = 100
    res = {'map': {'w': m['w'], 'h': m['h'], 'res': m['res']}, 'cost': args.cost, 'corridors': [c[0] for c in real],
           'dogleg_L_m': round(L, 2)}

    # ---- A: without the switch (today) --------------------------------------
    set_map(base)
    off = Mbf(False)
    time.sleep(4.0)
    t0, c0 = time.time(), off.cpu()
    time.sleep(30.0)
    res['cpu_off_pct'] = round(100.0 * (off.cpu() - c0) / (time.time() - t0), 2)
    plans_off = {name: off.plan(a, b) for name, a, b in pairs}
    plans_off['dogleg'] = off.plan(S, G)
    cm_off = off.costmap()
    set_map(blocked)
    time.sleep(3.5)
    plans_off['blocked'] = off.plan(pairs[0][1], pairs[0][2])
    cm_off_blocked = off.costmap()
    stop(off.p)
    time.sleep(2.0)

    # ---- B: with the switch --------------------------------------------------
    set_map(base)
    cor_pub.publish(OccupancyGrid())                        # nothing sent yet = empty
    on = Mbf(True)
    time.sleep(4.0)
    rows = []

    def row(scenario, name, a, b, ref, mask, static=base):
        p = on.plan(a, b)
        r = {'scenario': scenario, 'pair': name, 'off': summary(ref, mask, static), 'on': summary(p, mask, static),
             'same_bytes': p.get('sha') is not None and p.get('sha') == ref.get('sha')}
        if p.get('outcome') == 0 and ref.get('outcome') == 0:
            a, b = np.array(p['xy']), np.array(ref['xy'])
            d = np.sqrt(((a[:, None, :] - b[None, ::4, :]) ** 2).sum(axis=2)).min(axis=1) if len(a) * len(b) < 4e6 else None
            r['dev_m'] = None if d is None else round(float(d.max()), 3)
        rows.append(r)
        return p

    # B1 no corridor in the map: the plan must be the very same
    res['no_corridor_grid'] = set_corridors((), args.cost)
    for name, a, b in pairs:
        row('zadny koridor', name, a, b, plans_off[name], real_mask)
    cm = on.costmap()
    blocking = lambda a: (a >= 99) | (a < 0)      # lethal, inscribed, unknown as published
    res['no_corridor_blocking_cells_identical'] = bool((blocking(cm) == blocking(cm_off)).all())
    cm_none = cm

    # B2 the owner's corridors
    res['real_grid'] = set_corridors(real, args.cost)
    t0, c0 = time.time(), on.cpu()
    time.sleep(30.0)
    res['cpu_on_pct'] = round(100.0 * (on.cpu() - c0) / (time.time() - t0), 2)
    for name, a, b in pairs:
        row('koridory majitele', name, a, b, plans_off[name], real_mask)
    cm = on.costmap()
    # what is passable did not change, and only corridor cells differ from "no corridor"
    res['costmap_blocking_cells_identical'] = bool((blocking(cm) == blocking(cm_off)).all())
    res['costmap_cells_differing_from_no_corridor'] = int((cm != cm_none).sum())
    res['costmap_differing_outside_corridors'] = int(((cm != cm_none) & ~real_mask).sum())
    res['costmap_corridor_cells_dearer_than_no_corridor'] = int((cm > cm_none).sum())

    # B3 the corridor blocked by a wall (in the map, as a nogo area or an obstacle edit is)
    set_map(blocked)
    time.sleep(1.0)
    res['blocked_grid'] = set_corridors(real, args.cost)
    row('koridor zablokovany zdi', pairs[0][0], pairs[0][1], pairs[0][2], plans_off['blocked'], real_mask, blocked)
    cm = on.costmap()
    res['blocked_wall_cells'] = int((blocked != base).sum())
    res['blocked_wall_lethal_in_costmap'] = bool((cm[blocked != base] == 100).all())
    res['blocked_costmap_blocking_cells_identical'] = bool(((cm >= 99) == (cm_off_blocked >= 99)).all())
    set_map(base)
    time.sleep(1.0)

    # B4 how long a detour the corridor is worth
    for cost in sorted(set((6, args.cost, 8))):
        for r in ratios:
            cs = dogleg(r)
            set_corridors(cs, cost)
            mask = map_corridor.mask(cs, info)
            p = row('zajizdka %.2f, cena %d' % (r, cost), 'S->G %.1f m' % L, S, G, plans_off['dogleg'], mask)
            rows[-1]['detour'] = r
            rows[-1]['cost'] = cost
            rows[-1]['took_corridor'] = bool(p.get('outcome') == 0 and rows[-1]['on']['in_corridor'] > 0.8)
    res['rows'] = rows
    stop(on.p)
    return res


if __name__ == '__main__':
    main()
