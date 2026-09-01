#!/usr/bin/env python3
"""Self-test skládání mapy — `vitulus_navi/src/vitulus_navi/map_compose.py`.

    python3 tools/test_map_compose.py

Běží BEZ ROSu: sběrače jsou podvržené a mřížka je syntetická (12 × 8 m,
obvodová zeď, jedna překážka uprostřed, robot u západní stěny).  Testuje se
to, co jde pokazit potichu:

  * **Orientace.**  Řádek 0 occupancy gridu je nejnižší y, řádek 0 obrázku je
    horní.  Zrcadlená mapa by naučila agenta zrcadlený svět a nic dál po
    proudu by to nechytilo — proto se čte barva konkrétního pixelu na místě,
    kde má být překážka, a kontroluje se, že je NAHOŘE ta strana, která má být.
  * **Azimut vs. yaw.**  Popisek je to, co čtenář cituje; kdyby se
    `heading()` spletl o reflexi, popisek by protiřečil šipce nakreslené
    z téhož čísla.
  * **Vrstvy jsou volitelné a kombinovatelné**, aliasy fungují a neznámé
    jméno je varování, ne pád.
  * **Chybějící vrstva se pozná od prázdné.**  „Nevím" a „nic tam není" jsou
    pro robota opačné fakty a legenda je musí říct jinak.
"""
import math
import os
import shutil
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'src'))

from vitulus_navi import map_compose as mc          # noqa: E402

FAILURES = []


def ok(label, condition):
    print('  %-68s %s' % (label, 'ok' if condition else 'FAIL'))
    if not condition:
        FAILURES.append(label)


# ---------------------------------------------------------------- synthetika
RES = 0.1
W, H = 120, 80                  # 12 × 8 m


def synthetic_grid():
    """Obvodová zeď + jeden sloup; volno uvnitř, roh vlevo dole NEZMAPOVANÝ."""
    data = [0] * (W * H)
    for x in range(W):
        data[x] = 100                        # jižní zeď (řádek 0 = nejnižší y)
        data[(H - 1) * W + x] = 100          # severní zeď
    for y in range(H):
        data[y * W] = 100
        data[y * W + W - 1] = 100
    for y in range(30, 40):
        for x in range(60, 70):
            data[y * W + x] = 100            # sloup uprostřed
    for y in range(1, 10):
        for x in range(1, 10):
            data[y * W + x] = -1             # nezmapovaný roh
    return {'w': W, 'h': H, 'res': RES, 'ox': 0.0, 'oy': 0.0, 'oyaw': 0.0,
            'data': data, 'topic': '/test/grid', 'frame': 'map'}


ROBOT = {'x': 1.0, 'y': 4.0, 'yaw': 0.0, 'age_s': 0.0,
         'source': 'test', 'rtk': None}


def stub_ros(monkey):
    """Odpoj ROS: uzel se nezakládá, TF se nestaví, sběrače odpovídají z paměti."""
    monkey['ensure_node'] = mc.ensure_node
    monkey['prime'] = mc._prime_tf
    monkey['grid'] = mc._collect_grid
    monkey['robot'] = mc._collect_robot
    monkey['scan'] = mc._collect_scan
    monkey['objects'] = mc._collect_lidar_objects
    mc.ensure_node = lambda *a, **kw: False
    mc._prime_tf = lambda timeout_s: None
    mc._collect_grid = lambda topic, timeout: (synthetic_grid(), None)
    mc._collect_robot = lambda timeout: (dict(ROBOT), None)
    mc._collect_scan = lambda timeout: (
        {'points': [(1.0 + 0.5 * i, 4.0) for i in range(8)],
         'frame': 'base_scan', 'total': 8}, None)
    mc._collect_lidar_objects = lambda timeout: (None, 'topic mlčí (test)')


def unstub(monkey):
    mc.ensure_node = monkey['ensure_node']
    mc._prime_tf = monkey['prime']
    mc._collect_grid = monkey['grid']
    mc._collect_robot = monkey['robot']
    mc._collect_scan = monkey['scan']
    mc._collect_lidar_objects = monkey['objects']


# --------------------------------------------------------------------- testy
def test_layers_are_optional_and_combinable():
    print('\nvrstvy jsou volitelné, kombinovatelné a aliasy platí')
    got, complaints = mc.normalise_layers('lidar,robot,grid')
    ok('pořadí kreslení se drží, ne pořadí zadání',
       got == ['grid', 'lidar', 'robot'] and not complaints)
    got, _ = mc.normalise_layers(['map', 'scan', 'path'])
    ok('stará jména z agenta jsou aliasy (map/scan/path)',
       got == ['grid', 'plan', 'lidar'] or got == ['grid', 'lidar', 'plan'])
    got, complaints = mc.normalise_layers('grid,neexistuje')
    ok('neznámá vrstva je VAROVÁNÍ, ne pád', got == ['grid']
       and complaints and 'neznámá' in complaints[0])
    got, complaints = mc.normalise_layers('aerial')
    ok('odmítnutá vrstva řekne proč a spadne na výchozí',
       complaints and 'satelitní' in complaints[0]
       and sorted(got) == sorted(mc.DEFAULT_LAYERS))
    ok('"all" znamená všechno včetně lidar_objects',
       mc.normalise_layers('all')[0] == list(mc.LAYERS))


def test_heading_never_confuses_azimuth_with_yaw():
    print('\nazimut a yaw jsou dvě různá čísla a popisek to musí vědět')
    azimuth, yaw, point = mc.heading(0.0)
    ok('yaw 0 (východ) = azimut 90°, slovem V', abs(azimuth - 90.0) < 1e-6
       and abs(yaw) < 1e-6 and point == 'V')
    azimuth, _yaw, point = mc.heading(math.pi / 2)
    ok('yaw 90° (sever) = azimut 0°, slovem S',
       abs(azimuth) < 1e-6 and point == 'S')
    azimuth, _yaw, point = mc.heading(math.pi)
    ok('yaw 180° (západ) = azimut 270°, slovem Z',
       abs(azimuth - 270.0) < 1e-6 and point == 'Z')


def test_utm_round_trip():
    print('\npřepočet UTM -> map je inverzí data, ne přibližně')
    datum = (500000.0, 5500000.0, 0.35)
    x, y = 7.0, -3.0
    east = datum[0] + x * math.cos(datum[2]) - y * math.sin(datum[2])
    north = datum[1] + x * math.sin(datum[2]) + y * math.cos(datum[2])
    bx, by = mc.utm_to_map(east, north, datum)
    ok('tam a zpátky na milimetr', abs(bx - x) < 1e-6 and abs(by - y) < 1e-6)


def test_render_of_a_synthetic_site():
    print('\nsyntetická mřížka se vykreslí a obrázek je správně orientovaný')
    monkey = {}
    stub_ros(monkey)
    tmp = tempfile.mkdtemp(prefix='vitulus_map_compose_test_')
    try:
        out = os.path.join(tmp, 'render.png')
        result = mc.compose(layers='grid,lidar,robot,lidar_objects',
                            size=600, center='map', out=out,
                            timeout_s=0.2, site='Test')
        ok('ok=True', result.get('ok') is True)
        ok('PNG existuje a není prázdné',
           os.path.isfile(out) and os.path.getsize(out) > 2000)
        ok('nakreslené vrstvy: grid, lidar, robot',
           result.get('layers') == ['grid', 'lidar', 'robot'])
        ok('lidar_objects je NEDOSTUPNÁ (nevím), ne prázdná',
           result.get('missing') == ['lidar_objects'] and not result.get('empty'))
        ok('legenda ten rozdíl říká slovy',
           any('NEDOSTUPNÁ' in line for line in result.get('legend') or []))
        ok('a říká i orientaci', any('sever nahoru' in line
                                     for line in result.get('legend') or []))
        ok('věta pro model nese polohu robota',
           'x=1.00' in mc.summary(result))

        from PIL import Image
        image = Image.open(out).convert('RGB')
        width, _height = image.size
        # Pixel se počítá z `bbox`, který render sám vrací — kdyby se počítal
        # z „vím přece, jak je mřížka velká", testoval by se předpoklad, ne
        # obrázek.  Výška mapové části je (šířka × poměr výřezu), tak ji
        # `_compose` odvozuje; legenda je pod ní a do měření nezasahuje.
        box = result['bbox']
        span_x = box['east'] - box['west']
        span_y = box['north'] - box['south']
        proj_h = int(round(width * span_y / span_x))

        def at(mx, my):
            px = int(width * (mx - box['west']) / span_x)
            py = int(proj_h * (box['north'] - my) / span_y)
            return image.getpixel((min(width - 1, max(0, px)),
                                   min(proj_h - 1, max(0, py))))

        def is_unknown(pixel):
            return abs(pixel[0] - mc.C_UNKNOWN[0]) < 30 \
                and abs(pixel[1] - mc.C_UNKNOWN[1]) < 30
        ok('nezmapovaný roh je VLEVO DOLE (mřížka se překlopila správně)',
           is_unknown(at(0.5, 0.5)) and not is_unknown(at(0.5, 7.5)))
        ok('sloup uprostřed je červený',
           at(6.5, 3.5)[0] > 150 and at(6.5, 3.5)[1] < 90)
    finally:
        unstub(monkey)
        shutil.rmtree(tmp, ignore_errors=True)


def test_missing_everything_is_an_answer_not_a_crash():
    print('\nbez jediné vrstvy je to odpověď s důvodem, ne výjimka')
    monkey = {}
    stub_ros(monkey)
    mc._collect_grid = lambda topic, timeout: (None, 'topic mlčí (test)')
    mc._collect_robot = lambda timeout: (None, 'poloha nedostupná (test)')
    mc._collect_scan = lambda timeout: (None, 'lidar mlčí (test)')
    try:
        result = mc.compose(layers='grid,lidar,robot', size=400,
                            timeout_s=0.2, site='Test')
        ok('ok=False', result.get('ok') is False)
        ok('a je tam jednořádkový český důvod',
           'Není z čeho' in (result.get('error') or ''))
        ok('summary() z toho udělá větu, ne výjimku',
           isinstance(mc.summary(result), str) and mc.summary(result))
    finally:
        unstub(monkey)


def main():
    test_layers_are_optional_and_combinable()
    test_heading_never_confuses_azimuth_with_yaw()
    test_utm_round_trip()
    test_render_of_a_synthetic_site()
    test_missing_everything_is_an_answer_not_a_crash()
    print()
    if FAILURES:
        print('%d FAILURE(S):' % len(FAILURES))
        for item in FAILURES:
            print('  - %s' % item)
        return 1
    print('test_map_compose: all checks passed')
    return 0


if __name__ == '__main__':
    sys.exit(main())
