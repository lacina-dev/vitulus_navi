#!/usr/bin/env python3
"""Self-test plátna mapy — `vitulus_navi/src/vitulus_navi/map_canvas.py`.

    python3 tools/test_map_canvas.py
    MAP_CANVAS_SAMPLES=/tmp/x python3 tools/test_map_canvas.py   # kam uložit ukázková PNG

Běží BEZ ROSu: syntetická mapa (12 × 8 m, obvodová zeď, sloup 6–7 × 3–4 m,
zóna ZoneA 1–5 × 1–6 m) se podává jako `collected`, a když na robotu existuje
site bundle 10_2026, vykreslí se i z něj (`source='bundle'`) — a test ověří,
že se rospy vůbec nenačetl.  Hlídá:

  * převod pixel -> mapa -> pixel tam a zpět do 1 px,
  * souřadnicovou mřížku (čára na celém metru je vidět) a čísla os v okraji,
  * skicu v obrázku (jantarový bod tam, kam patří),
  * kontrolu bodu: v překážce / v inflaci / volno v zóně / mimo mapu,
    nejbližší volné místo, dosažitelnost,
  * geometrii zóny: nejvzdálenější bod (i s odstupem), bod uvnitř s odstupem,
    obvod s odstupem ven i dovnitř, jméno „zóna A" -> ZoneA,
  * že modul nic nepublikuje.
"""
import math
import os
import re
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'src'))

from vitulus_navi import map_canvas as mcv          # noqa: E402
from vitulus_navi import map_compose as mc          # noqa: E402

FAILURES = []
SAMPLES = os.environ.get('MAP_CANVAS_SAMPLES') or tempfile.mkdtemp(prefix='map_canvas_')


def ok(label, condition):
    print('  %-70s %s' % (label, 'ok' if condition else 'FAIL'))
    if not condition:
        FAILURES.append(label)


RES = 0.1
W, H = 120, 80
ZONE = [(1.0, 1.0), (5.0, 1.0), (5.0, 6.0), (1.0, 6.0)]


def synthetic():
    data = [0] * (W * H)
    for x in range(W):
        data[x] = 100
        data[(H - 1) * W + x] = 100
    for y in range(H):
        data[y * W] = 100
        data[y * W + W - 1] = 100
    for y in range(30, 40):
        for x in range(60, 70):
            data[y * W + x] = 100                # sloup 6–7 × 3–4 m
    grid = {'w': W, 'h': H, 'res': RES, 'ox': 0.0, 'oy': 0.0, 'oyaw': 0.0,
            'data': data, 'topic': '/test/grid', 'frame': 'map'}
    return {'data': {'map': grid,
                     'zones': {'zones': [('ZoneA', list(ZONE))]},
                     'waypoints': {'points': [('TEST', 8.0, 6.0), ('docked', 10.5, 1.0)],
                                   'paths': [('P1', [(2.0, 7.0), (10.0, 7.0)])],
                                   'source': 'test'},
                     'robot': {'x': 9.0, 'y': 2.0, 'yaw': 0.0, 'age_s': 0.0,
                               'source': 'test', 'rtk': None}},
            'missing': {}, 'empty': {'geofence': 'test: žádná'}, 'site': None,
            'datum': None, 'dock': {'name': 'docked', 'x': 10.5, 'y': 1.0, 'yaw': 0.0},
            'notes': [], 'raw': {}, 'source': 'bundle'}


def pixel(path, xy):
    from PIL import Image
    image = Image.open(path).convert('RGB')
    return image.getpixel((int(round(xy[0])), int(round(xy[1]))))


def near(a, b, tol=40):
    return all(abs(int(p) - int(q)) <= tol for p, q in zip(a, b))


def test_view_and_transform():
    print('výřez, mřížka, převody')
    col = synthetic()
    out = os.path.join(SAMPLES, 'canvas_synthetic_zone.png')
    res = mcv.view(center='zóna A', collected=col, out=out, size=700,
                   overlay={'markers': [{'x': 2.5, 'y': 7.4, 'kind': 'sketch', 'label': 'b1'}]})
    ok('view(center="zóna A") z podaných dat vrátí ok', res.get('ok'))
    if not res.get('ok'):
        print('   ', res)
        return
    meta = res['meta']
    t = meta['transform']
    ok('meta nese rám map, bbox, transform, mřížku mapy a krok os',
       meta['frame'] == 'map' and meta['bbox'] and t['m_per_px'] > 0
       and meta['grid']['res'] == RES and t['axes_step_m'] == 1.0)
    ok('mapa je odsazená okrajem pro čísla os (x0, y0 > 0)', t['x0'] > 0 and t['y0'] > 0)
    ok('výřez obsahuje celou zónu',
       meta['bbox']['west'] <= 1.0 and meta['bbox']['east'] >= 5.0
       and meta['bbox']['south'] <= 1.0 and meta['bbox']['north'] >= 6.0)
    worst = 0.0
    for px in ([t['x0'] + 3, t['y0'] + 5], [400, 300], [t['x0'] + t['w'] - 2, t['y0'] + t['h'] - 2],
               [123.4, 456.7]):
        back = mcv.to_px(meta, mcv.to_map(meta, px))
        worst = max(worst, abs(back[0] - px[0]), abs(back[1] - px[1]))
    ok('pixel -> mapa -> pixel tam a zpět do 1 px (nejhůř %.2f px)' % worst, worst <= 1.0)
    xy = (3.3, 4.7)
    back = mcv.to_map(meta, mcv.to_px(meta, xy))
    ok('mapa -> pixel -> mapa do 1 px (%.3f m)' % max(abs(back[0] - xy[0]), abs(back[1] - xy[1])),
       max(abs(back[0] - xy[0]), abs(back[1] - xy[1])) <= t['m_per_px'])
    many = mcv.to_map(meta, [[100, 100], [200, 200]])
    ok('to_map bere i seznam bodů', len(many) == 2 and len(many[0]) == 2)
    path = res['path']
    ok('sloup (6.5, 3.5) je v obrázku červený tam, kam ho staví transformace',
       near(pixel(path, mcv.to_px(meta, (6.5, 3.5))), mc.C_OCC[:3]))
    on_line = pixel(path, mcv.to_px(meta, (3.0, 6.55)))
    off_line = pixel(path, mcv.to_px(meta, (3.3, 6.55)))
    ok('čára mřížky na x = 3 m je vidět (světlejší než volno vedle)',
       sum(on_line) > sum(off_line) + 30)
    lx, ly = t['x0'] - 20, mcv.to_px(meta, (0.0, 3.0))[1]
    from PIL import Image
    img = Image.open(path).convert('L')
    label = max(img.getpixel((x, y)) for x in range(4, int(t['x0']) - 6)
                for y in range(int(ly) - 8, int(ly) + 8))
    ok('v levém okraji u y = 3 m je číslo osy (světlé pixely)', label > 180)
    ok('legenda vysvětluje souřadnice', any('Souřadnice' in r for r in meta['legend']))
    sk = pixel(path, [mcv.to_px(meta, (2.5, 7.4))[0] + 5, mcv.to_px(meta, (2.5, 7.4))[1]])
    ok('skica: bod b1 je v obrázku jantarový na svém místě', near(sk, mc.C_SKETCH, 50))
    kinds = dict((o['kind'], o) for o in meta['objects'])
    ok('objekty: zóna s těžištěm a prstencem, robot, trasa ve výřezu',
       kinds.get('zone', {}).get('name') == 'ZoneA' and kinds['zone']['centroid'] == [3.0, 3.5]
       and len(kinds['zone']['ring']) == 4 and 'robot' in kinds and 'path' in kinds)
    ok('bod a dok mimo výřez v objektech nejsou', 'point' not in kinds and 'dock' not in kinds)
    ok('objekty mají i pixely (model je najde v obrázku)',
       'px' in kinds['robot'] and 'centroid_px' in kinds['zone'])
    res2 = mcv.view(center=[9.0, 2.0], span_m=6, collected=col,
                    out=os.path.join(SAMPLES, 'canvas_synthetic_xy.png'))
    b = res2.get('meta', {}).get('bbox') or {}
    ok('center=[x, y] + span_m=6 dá výřez 6 m kolem bodu',
       res2.get('ok') and abs((b['east'] - b['west']) - 6.0) < 0.6
       and abs(0.5 * (b['east'] + b['west']) - 9.0) < 0.3)
    res3 = mcv.view(center='robot', fit=['ZoneA', 'TEST', [11.0, 7.5]], collected=col,
                    out=os.path.join(SAMPLES, 'canvas_synthetic_fit.png'))
    b = res3.get('meta', {}).get('bbox') or {}
    ok('fit = [zóna, bod, souřadnice] — vejde se všechno',
       res3.get('ok') and b['west'] <= 1.0 and b['east'] >= 11.0 and b['north'] >= 7.5)
    kinds3 = set(o['kind'] for o in res3['meta']['objects'])
    ok('… a v objektech je i dok a bod TEST', {'dock', 'point'} <= kinds3)
    bad = mcv.view(center='ZoneZ', collected=col)
    ok('neznámé jméno = ok False s nabídkou známých jmen',
       not bad.get('ok') and 'ZoneA' in (bad.get('error') or ''))


def test_check():
    print('kontrola bodů')
    col = synthetic()
    res = mcv.check([{'x': 6.5, 'y': 3.5, 'label': 'sloup'},
                     {'x': 6.5, 'y': 2.85, 'label': 'u sloupu'},
                     {'x': 3.0, 'y': 3.0, 'label': 'v zóně'},
                     {'x': 8.0, 'y': 3.0, 'label': 'mimo zónu'},
                     [50.0, 50.0], 'TEST', 'ZoneA'], collected=col)
    ok('check vrátí ok a řádek na každý bod', res.get('ok') and len(res['points']) == 7)
    rows = dict((r['label'], r) for r in res['points'])
    sloup = rows['sloup']
    ok('bod v překážce: cell lethal, není ok', sloup['cell'] == 'lethal' and not sloup['ok'])
    nf = sloup.get('nearest_free') or {}
    ok('… a nabídne nejbližší volné místo mimo sloup a jeho inflaci',
       nf and math.hypot(max(6.05 - nf['x'], 0, nf['x'] - 6.95),
                     max(3.05 - nf['y'], 0, nf['y'] - 3.95)) > 0.26 and nf['dist_m'] < 1.0)
    ok('bod těsně u sloupu je v inflaci', rows['u sloupu']['cell'] == 'inflated')
    inner = rows['v zóně']
    ok('bod v zóně: volno, in_zones = [ZoneA], dosažitelný od robota',
       inner['free'] and inner['in_zones'] == ['ZoneA'] and inner['reach']['reachable']
       and inner['ok'])
    ok('… vzdálenost a délka cesty od robota dávají smysl',
       abs(inner['dist_m'] - math.hypot(6.0, 1.0)) < 0.05
       and inner['reach']['length_m'] >= inner['dist_m'] - 0.2)
    ok('bod mimo zónu: in_zones prázdné, ale volno', rows['mimo zónu']['in_zones'] == []
       and rows['mimo zónu']['free'])
    out = rows['50.00, 50.00']
    ok('bod mimo mapu: cell outside, není ok', out['cell'] == 'outside' and not out['ok'])
    ok('jméno bodu se přeloží (TEST = 8, 6)', rows['TEST']['x'] == 8.0 and rows['TEST']['y'] == 6.0)
    ok('jméno zóny = její nejvnitřnější bod uvnitř zóny',
       rows['ZoneA']['in_zones'] == ['ZoneA'] and rows['ZoneA']['free'])
    ok('každý řádek má českou větu', all(r.get('summary') for r in res['points']))
    ok('start bez polohy robota spadne na dok',
       mcv.check([[3.0, 3.0]], collected=dict(synthetic(), data=dict(
           (k, v) for k, v in synthetic()['data'].items() if k != 'robot')),
           reach=False)['start'].get('kind') == 'dock')


def test_geometry():
    print('geometrie zóny')
    col = synthetic()
    far = mcv.farthest_point(ZONE, (0.0, 0.0), margin_m=0.0)
    ok('nejvzdálenější bod zóny od (0,0) je roh (5, 6)', (far['x'], far['y']) == (5.0, 6.0))
    far = mcv.farthest_point(ZONE, (0.0, 0.0), margin_m=0.5)
    ok('… s odstupem 0.5 m je (4.5, 5.5) a 0.5 m od hrany',
       abs(far['x'] - 4.5) < 0.02 and abs(far['y'] - 5.5) < 0.02 and abs(far['edge_m'] - 0.5) < 0.02)
    ip = mcv.interior_point(ZONE, 0.5)
    ok('bod uvnitř s odstupem 0.5 m je uvnitř a dost daleko od hrany',
       ip['ok'] and mcv.inside((ip['x'], ip['y']), ZONE) and ip['edge_m'] >= 0.48)
    ipn = mcv.interior_point(ZONE, 0.5, near=(9.0, 2.0))
    ok('bod uvnitř nejblíž robotu leží u východní hrany (x ≈ 4.5)',
       ipn['ok'] and abs(ipn['x'] - 4.5) < 0.05 and abs(ipn['y'] - 2.0) < 0.05)
    ok('úzká zóna na velký odstup = ok False', not mcv.interior_point(ZONE, 3.0)['ok'])
    grown = mcv.offset_ring(ZONE, 0.5)
    ok('obvod s odstupem 0.5 m ven: délka ≈ 18 + π (%.2f m)' % grown.get('length_m', 0),
       grown['ok'] and abs(grown['length_m'] - (18.0 + math.pi)) < 0.3)
    ok('… každý jeho bod je 0.5 m od zóny a mimo ni',
       all(not mcv.inside(p, ZONE) and abs(mcv.distance_to_edge(p, ZONE) - 0.5) < 0.05
           for p in grown['ring']))
    shrunk = mcv.offset_ring(ZONE, -0.5)
    ok('obvod 0.5 m dovnitř: obdélník 3 × 4 m (14 m)',
       shrunk['ok'] and abs(shrunk['length_m'] - 14.0) < 0.05)
    nearest = mcv.nearest_point(ZONE, (9.0, 2.0))
    ok('nejbližší bod zóny k robotu je (5, 2), 4 m', nearest == {'x': 5.0, 'y': 2.0,
                                                               'dist_m': 4.0, 'inside': False})
    geo = mcv.zone_geometry('zona a', collected=col, offset_m=0.4)
    ok('zone_geometry("zona a"): ZoneA, plocha 20 m², obvod 18 m, těžiště (3, 3.5)',
       geo['ok'] and geo['name'] == 'ZoneA' and geo['area_m2'] == 20.0
       and geo['perimeter_m'] == 18.0 and geo['centroid'] == [3.0, 3.5])
    ok('… nejvzdálenější bod od robota (9, 2) je severozápadní roh s odstupem',
       geo['from']['kind'] == 'robot' and abs(geo['farthest']['x'] - 1.3) < 0.02
       and abs(geo['farthest']['y'] - 5.7) < 0.02)
    ok('… a obvod s odstupem 0.4 m', geo['offset']['ok'] and geo['offset']['offset_m'] == 0.4)
    ok('L-zóna: těžiště mimo, bod uvnitř je uvnitř', _l_zone())


def _l_zone():
    ring = [(0, 0), (4, 0), (4, 1), (1, 1), (1, 4), (0, 4)]
    c = mcv.centroid(ring)
    ip = mcv.interior_point(ring, 0.2)
    return (not mcv.inside(c, ring)) and ip['ok'] and mcv.inside((ip['x'], ip['y']), ring)


def test_bundle():
    print('render z uložených dat (bez ROSu)')
    site = '10_2026'
    root = os.path.expanduser('~/.vitulus/mapping_v3/%s' % site)
    if not os.path.isdir(root):
        print('  (site %s na tomhle stroji není — přeskočeno)' % site)
        return
    col = mcv.collect('bundle', site=site)
    ok('bundle: mapa z rastru, zóny, body, dok', 'map' in col['data'] and 'zones' in col['data']
       and 'waypoints' in col['data'] and col['dock'])
    res = mcv.view(center='ZoneB', collected=col,
                   out=os.path.join(SAMPLES, 'canvas_10_2026_zoneB.png'))
    ok('bundle: výřez zóny B se vykreslí', res.get('ok'))
    whole = mcv.view(center='map', collected=col, size=1000,
                     out=os.path.join(SAMPLES, 'canvas_10_2026_site.png'))
    ok('bundle: celá site se vykreslí', whole.get('ok'))
    geo = mcv.zone_geometry('zóna B', from_xy='dock', collected=col, offset_m=0.5)
    ok('bundle: nejvzdálenější bod zóny B od doku je uvnitř zóny B',
       geo.get('ok') and mcv.inside((geo['farthest']['x'], geo['farthest']['y']),
                                    [tuple(p) for p in geo['ring']]))
    chk = mcv.check([geo['farthest']], collected=col, start='dock')
    row = (chk.get('points') or [{}])[0]
    ok('bundle: ten bod je volný, v ZoneB a dosažitelný od doku',
       row.get('free') and row.get('in_zones') == ['ZoneB']
       and (row.get('reach') or {}).get('reachable'))
    ok('celý render šel bez ROSu (rospy se nenačetl)', 'rospy' not in sys.modules)
    print('  ukázková PNG: %s' % SAMPLES)


def test_read_only():
    print('jen čtení')
    path = os.path.join(ROOT, 'src', 'vitulus_navi', 'map_canvas.py')
    with open(path) as stream:
        src = stream.read()
    ok('map_canvas nemá Publisher/ServiceProxy/akce',
       not re.search(r'Publisher|ServiceProxy|SimpleActionClient|\.publish\(', src))


if __name__ == '__main__':
    os.makedirs(SAMPLES, exist_ok=True)
    test_view_and_transform()
    test_check()
    test_geometry()
    test_bundle()
    test_read_only()
    if FAILURES:
        print('\ntest_map_canvas: %d FAIL' % len(FAILURES))
        sys.exit(1)
    print('\ntest_map_canvas: all checks passed')
