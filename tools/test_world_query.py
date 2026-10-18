#!/usr/bin/env python3
"""Self-test dotazů na svět — `vitulus_navi/src/vitulus_navi/world_query.py`.

    python3 tools/test_world_query.py

Běží BEZ ROSu, nad syntetickou mřížkou 12 × 8 m (obvodová zeď, příčka se
dvěma průchody, nová překážka, která v uložené mapě není) a syntetickým
skenem.  Hlídá to, co jde pokazit potichu: směry vůči robotu (vlevo je
vlevo), známá vs nová překážka, dosažitelnost a objížďka, uvěznění, a že
modul nic nepublikuje.
"""
import math
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'src'))

from vitulus_navi import world_query as wq          # noqa: E402

FAILURES = []


def ok(label, condition):
    print('  %-70s %s' % (label, 'ok' if condition else 'FAIL'))
    if not condition:
        FAILURES.append(label)


RES = 0.05
W, H = 240, 160            # 12 × 8 m, origin (0, 0)


def blank():
    return [0] * (W * H)


def put(data, x0, y0, x1, y1, value):
    for r in range(int(y0 / RES), int(y1 / RES)):
        for c in range(int(x0 / RES), int(x1 / RES)):
            if 0 <= r < H and 0 <= c < W:
                data[r * W + c] = value


def inflate(data, radius_m=0.35):
    """Hrubá inflace jako costmap_2d: lethal 100, do poloměru 99 (inscribed)."""
    import numpy as np
    from scipy import ndimage
    a = np.asarray(data).reshape(H, W)
    dist = ndimage.distance_transform_edt(a < 100) * RES
    out = np.where(a >= 100, 100, np.where(dist <= 0.26, 99,
                   np.where(dist <= radius_m, 60, 0)))
    return [int(v) for v in out.ravel()]


def grid_payload(data):
    return {'w': W, 'h': H, 'res': RES, 'ox': 0.0, 'oy': 0.0, 'oyaw': 0.0,
            'data': data, 'topic': 'syntetika', 'frame': 'map'}


def world():
    static = blank()
    put(static, 0, 0, 12, 0.1, 100)          # jižní zeď
    put(static, 0, 7.9, 12, 8, 100)          # severní
    put(static, 0, 0, 0.1, 8, 100)           # západní
    put(static, 11.9, 0, 12, 8, 100)         # východní
    put(static, 6, 0, 6.1, 3, 100)           # příčka x=6 s průchodem y 3..5
    put(static, 6, 5, 6.1, 8, 100)
    cost = list(static)
    put(cost, 6, 3.0, 6.2, 5.0, 100)         # NOVÁ překážka zavře průchod
    return static, inflate(cost)


def main():
    static, cost = world()
    mapgrid = wq.Grid(grid_payload(static))
    costgrid = wq.Grid(grid_payload(cost))
    robot = {'x': 2.0, 'y': 4.0, 'yaw': 0.0}

    print('geometrie')
    rel = wq.relative(robot, 3.0, 5.0)
    ok('bod vpředu vlevo je front_left', rel['sector'] == 'front_left')
    rel = wq.relative({'x': 0, 'y': 0, 'yaw': math.pi / 2}, 1.0, 0.0)
    ok('robot míří na sever: bod na východě je vpravo', rel['sector'] == 'right')
    ok('UTM ↔ map tam a zpět',
       all(abs(a - b) < 1e-6 for a, b in zip(
           wq.mc.utm_to_map(*wq.map_to_utm(1.5, -2.0, (100.0, 200.0, 0.3)),
                            (100.0, 200.0, 0.3)), (1.5, -2.0))))

    print('překážky')
    obs = wq.obstacles(costgrid, mapgrid, robot, radius_m=8.0)
    kinds = set(o['kind'] for o in obs)
    ok('nalezena nová překážka (v mapě není)', 'static_new' in kinds)
    ok('zdi jsou známé z mapy', 'static_known' in kinds)
    new = [o for o in obs if o['kind'] == 'static_new'][0]
    ok('nová překážka je vpředu, ~4 m', new['rel']['sector'] == 'front'
       and 3.5 < new['dist_m'] < 4.5)

    print('plán')
    goal = (9.0, 4.0)
    clr = wq._clearance(costgrid)
    res = wq.plan(costgrid, (robot['x'], robot['y']), goal, clr, window_m=0)
    ok('za zavřeným průchodem je cíl nedosažitelný', res['reachable'] is False)
    blk = wq.blocked_at(costgrid, (robot['x'], robot['y']), goal)
    ok('přímka narazí na překážku u x≈6', blk is not None and 5.3 < blk['x'] < 6.2)
    free_goal = (4.0, 6.5)
    res = wq.plan(costgrid, (robot['x'], robot['y']), free_goal, clr)
    ok('volný cíl ve stejné místnosti je dosažitelný', res['reachable'] is True)
    ok('délka cesty >= přímá vzdálenost', res['length_m'] >= res['straight_m'] - 0.01)
    ok('cesta má body a min. vůli', res.get('points') and res.get('min_clearance_m'))

    # otevři průchod nahoře: příčka do y=6, nad ní volno -> objížďka existuje
    static2 = list(static)
    cost2 = blank()
    for i, v in enumerate(static2):
        cost2[i] = v
    put(cost2, 6, 5.0, 6.1, 8, 0)
    put(cost2, 6, 7.0, 6.1, 8, 100)
    put(cost2, 6, 3.0, 6.2, 5.0, 100)
    cg2 = wq.Grid(grid_payload(inflate(cost2)))
    clr2 = wq._clearance(cg2)
    res2 = wq.plan(cg2, (robot['x'], robot['y']), goal, clr2)
    ok('s průchodem nahoře je cíl dosažitelný oklikou', res2['reachable'] is True
       and res2['length_m'] > 7.2)
    blk2 = wq.blocked_at(cg2, (robot['x'], robot['y']), goal)
    vias = wq.via_candidates(cg2, (robot['x'], robot['y']), goal,
                             {'NAHORE': (6.0, 6.0), 'DOCK': (1.0, 1.0)},
                             clearance=clr2, blocker=blk2)
    ok('objížďka přes pojmenovaný bod NAHORE', any(v['name'] == 'NAHORE' for v in vias))
    ok('DOCK se jako mezilehlý bod nenabízí', not any(v['name'] == 'DOCK' for v in vias))
    ok('objížďka bokem překážky (detour)', any(v['kind'] == 'detour' for v in vias))

    print('uvěznění')
    cage = blank()
    put(cage, 1.0, 1.0, 5.0, 1.1, 100)
    put(cage, 1.0, 5.0, 5.0, 5.1, 100)
    put(cage, 1.0, 1.0, 1.1, 5.1, 100)
    put(cage, 5.0, 1.0, 5.1, 5.1, 100)
    cg3 = wq.Grid(grid_payload(inflate(cage)))
    tr = wq.trapped(cg3, {'x': 3.0, 'y': 3.0, 'yaw': 0.0}, radius_m=2.5)
    ok('robot v kleci je uvězněný', tr['trapped'] is True)
    tr = wq.trapped(costgrid, robot, radius_m=1.5)
    ok('robot v místnosti uvězněný není', tr['trapped'] is False and tr['exits'])

    print('lidar')
    pts = [(2.0 + 1.0, 4.0 + 0.5 + 0.01 * i) for i in range(12)]     # vpředu vlevo
    pts += [(0.1, 4.0 + 0.02 * i) for i in range(30)]                # západní zeď
    cl = wq.scan_clusters({'points': pts}, robot, mapgrid)
    near = cl[0]
    ok('nejbližší shluk vpředu vlevo, nový', near['sector'] in ('front_left', 'front')
       and near['kind'] == 'static_new')
    ok('zeď za robotem je známá', any(c['kind'] == 'static_known' and c['sector'] == 'back'
                                     for c in cl))
    mov = wq.moving_objects({'objects': [{'x': 1.0, 'y': 0.5, 'id': 3, 'cls': 'cat'}]}, robot)
    cl2 = wq.scan_clusters({'points': pts}, robot, mapgrid, moving=mov)
    ok('shluk na místě pohyblivého objektu je moving', cl2[0]['kind'] == 'moving')

    print('analyze nad podvrženým sběrem')
    collected = {'data': {'robot': dict(robot, age_s=0.0, source='syntetika', rtk='fixed'),
                          'map': grid_payload(static), 'costmap': grid_payload(cost),
                          'waypoints': {'points': [('TEST', 9.0, 4.0), ('BOK', 4.0, 6.5)],
                                        'paths': []},
                          'zones': {'zones': [('Z1', [(1, 1), (5, 1), (5, 7), (1, 7)])]}},
                 'missing': {}, 'empty': {}, 'site': 'syn', 'datum': (0.0, 0.0, 0.0), 'raw': {}}
    an = wq.analyze('test', collected=collected)
    ok('cíl podle jména bez ohledu na velikost písmen', an['reach']['goal']['name'] == 'TEST')
    ok('TEST nedosažitelný, věta to říká', an['reach']['reachable'] is False
       and 'NEDOSAŽITELNÝ' in an['summary'])
    loc = wq.locate('BOK', collected=collected)
    ok('locate: robot v zóně Z1', loc['in_zones'] == ['Z1'])
    ok('locate: UTM z data', loc['utm']['east'] == 2.0)
    ok('locate: cíl BOK vpředu vlevo', loc['goal']['sector'] == 'front_left')

    print('jen čtení')
    src = open(os.path.join(ROOT, 'src', 'vitulus_navi', 'world_query.py')).read()
    ok('world_query nemá Publisher/ServiceProxy/akce',
       not any(w in src for w in ('Publisher(', 'ServiceProxy(', 'SimpleActionClient(',
                                  '.publish(', 'send_goal')))

    if FAILURES:
        print('\ntest_world_query: %d FAIL' % len(FAILURES))
        return 1
    print('\ntest_world_query: all checks passed')
    return 0


if __name__ == '__main__':
    sys.exit(main())
