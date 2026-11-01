"""Mapa jako plátno — výřez mapy se souřadnicemi, převody pixel↔mapa a kontrola bodů.

Proč to tady je
---------------
`map_compose` umí svět robota NAKRESLIT a `world_query` SPOČÍTAT.  Kdo plánuje
(operátor agenta, majitel, UI), potřebuje obojí najednou a ještě jednu věc
navíc: z obrázku zpátky k číslům.  „Dojeď do nejvzdálenější části zóny B"
nejde splnit, dokud plánovač nevidí, kde zóna B je, nedokáže si do mapy
ukázat bod a nepřevede ten bod na souřadnice, které robot pochopí.

Tenhle modul proto:

* vykreslí **výřez na přání** — střed robot / dok / bod / zóna / trasa /
  souřadnice, rozsah v metrech nebo „ať se vejde X" — s **čitelnou souřadnicovou
  mřížkou** a popisky os v metrech rámu `map` (`map_compose(axes=True)`), a k
  obrázku vrátí **strojová metadata**: rámec, bbox, mřížku mapy, transformaci
  pixel↔mapa a seznam vykreslených objektů s jejich souřadnicemi i pixely;
* převádí **pixel→mapa a mapa→pixel** přes ta metadata (`to_map`, `to_px`);
* **zkontroluje body** (`check`): volno v costmapě / v inflaci / v překážce /
  nezmapované, uvnitř které zóny a geofence, dosažitelnost od robota (A* nad
  costmapou z `world_query`), délku cesty a nejbližší volné místo;
* dá **geometrii zóny** (`zone_geometry`): těžiště, plocha, obvod, bod uvnitř
  s odstupem od hrany, nejvzdálenější bod zóny od daného bodu, obvod zóny
  s odstupem dovnitř/ven (pro „objeď zónu").

Je to robotí vlastnost (svět robota a jeho mapa), ne agentí: funguje bez
agenta — CLI `rosrun vitulus_navi map_canvas view|check|zone`.  Skica plánu
patřící misi je agentí věc a žije v agentovi (`vitulus_claude.agent.mapcanvas`),
sem chodí jen jako `overlay` ke kreslení.

Zdroje dat (`source`)
---------------------
* ``'live'``   — ROS (topicy, TF) přes `world_query.collect`;
* ``'bundle'`` — uložený site bundle (`~/.vitulus/mapping_v3/<site>/`): mapa
  z rastru, zóny/body/trasy/geofence z GeoJSONu přes datum.  Bez ROSu, bez
  robota (poloha robota pak chybí a legenda to řekne);
* ``'auto'``   — živě, a co ze statických vrstev (mapa, zóny, body, geofence)
  živě mlčí, doplní bundle — s poznámkou do `notes`.

Domácí pravidla jako `map_compose`/`world_query`: jen čtení, nikdy nevyhazuje
(vždy dict s `ok`), „nevím" není „nic tam není".
Rámy: všechno v `map` (x = východ, y = sever), délky v metrech.
"""
import json
import math
import os
import time

try:
    import numpy as np
except Exception:                                       # noqa: BLE001
    np = None

try:
    from . import map_compose as mc
    from . import world_query as wq
except Exception:                                       # noqa: BLE001
    import importlib.util as _util

    def _load(name):
        spec = _util.spec_from_file_location(
            'vitulus_navi_local_' + name,
            os.path.join(os.path.dirname(os.path.abspath(__file__)), name + '.py'))
        module = _util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module
    mc = _load('map_compose')
    wq = _load('world_query')

FRAME = 'map'
SOURCES = ('auto', 'live', 'bundle')
STATIC = ('map', 'waypoints', 'zones', 'geofence')

#: Vrstvy plátna.  Jména `map_compose` plus dvě vlastní: `dock` (dok jako
#: výrazná značka, ne jen bod mezi body) a `sketch` (překryv skici od volajícího).
LAYERS = mc.LAYERS + ('dock', 'sketch')
DEFAULT_LAYERS = ('grid', 'zones', 'waypoints', 'geofence', 'plan', 'robot', 'dock')
ALIASES = dict(mc.ALIASES, obstacles='costmap', costmapa='costmap',
               prekazky_mapa='costmap', dok='dock', skica='sketch', zony='zones')

DEFAULT_SIZE = 900
DEFAULT_SPAN_M = {'robot': 16.0, 'dock': 12.0, 'point': 12.0, 'xy': 12.0}
FIT_PAD_M = 2.0
MIN_SPAN_M = mc.MIN_SPAN_M
NEAR_FREE_RADIUS_M = 3.0
DEFAULT_MARGIN_M = 0.3

#: Kódy stavu buňky v kontrole bodu, s českým slovem do věty.
CELL_WORDS = {'free': 'volno', 'inflated': 'v inflaci překážky (robot se tam nevejde)',
              'lethal': 'v překážce', 'unknown': 'nezmapované (nevím, co tam je)',
              'outside': 'mimo mapu'}


def _r(value, digits=2):
    try:
        return round(float(value), digits)
    except (TypeError, ValueError):
        return None


def _xy(value):
    """(x, y) z [x, y], (x, y), {'x','y'} nebo 'x,y'; jinak None."""
    if value is None:
        return None
    if isinstance(value, dict):
        try:
            return float(value['x']), float(value['y'])
        except (KeyError, TypeError, ValueError):
            return None
    if isinstance(value, (list, tuple)) and len(value) >= 2:
        try:
            return float(value[0]), float(value[1])
        except (TypeError, ValueError):
            return None
    if isinstance(value, str) and ',' in value:
        try:
            a, b = value.split(',', 1)
            return float(a), float(b)
        except ValueError:
            return None
    return None


# ---------------------------------------------------------------------------
# site bundle bez ROSu: mapa z rastru, zóny/body/trasy/geofence přes datum
# ---------------------------------------------------------------------------
def _bundle_root():
    sb = mc._sitebundle()
    return getattr(sb, 'BUNDLE_ROOT', os.path.expanduser('~/.vitulus/mapping_v3'))


def _served_raster(site):
    """Jméno rastru, který mapping naposled servíroval (`.last_served`), nebo 'combined'."""
    try:
        with open(os.path.join(_bundle_root(), '.last_served'), 'r') as stream:
            data = json.load(stream)
        if isinstance(data, dict) and data.get('site') == site and data.get('raster'):
            return str(data['raster'])
    except (IOError, OSError, ValueError):
        pass
    return 'combined'


def load_bundle_grid(site, raster=None):
    """(payload, chyba) mřížky z rastru bundlu — tvar jako `map_compose._collect_grid`.

    PGM + YAML ve formátu map_server (trinary): p = (255 - pixel)/255 (bez
    negate), p > occupied_thresh = 100, p < free_thresh = 0, jinak -1.
    Řádek 0 obrázku je HORNÍ, řádek 0 OccupancyGrid NEJNIŽŠÍ y — překlápí se.
    """
    if np is None:
        return None, 'numpy chybí, rastr bundlu nejde načíst'
    raster = raster or _served_raster(site)
    folder = os.path.join(_bundle_root(), site, 'rasters', raster)
    yaml_path = os.path.join(folder, raster + '.yaml')
    if not os.path.isfile(yaml_path):
        return None, 'site %s nemá rastr %s (%s)' % (site, raster, yaml_path)
    try:
        import yaml
        with open(yaml_path, 'r') as stream:
            meta = yaml.safe_load(stream) or {}
        pil = mc._pil()
        if pil is None:
            return None, 'Pillow chybí, rastr bundlu nejde načíst'
        image = pil[0].open(os.path.join(folder, meta.get('image') or raster + '.pgm'))
        pix = np.asarray(image.convert('L'), dtype=np.float32)
    except Exception as exc:                            # noqa: BLE001
        return None, 'rastr %s nejde přečíst: %s' % (raster, exc)
    if int(meta.get('negate') or 0):
        p = pix / 255.0
    else:
        p = (255.0 - pix) / 255.0
    occ = float(meta.get('occupied_thresh', 0.65))
    free = float(meta.get('free_thresh', 0.196))
    grid = np.full(pix.shape, -1, dtype=np.int16)
    grid[p > occ] = 100
    grid[p < free] = 0
    grid = grid[::-1, :]                                # řádek 0 = jih
    origin = meta.get('origin') or [0.0, 0.0, 0.0]
    h, w = grid.shape
    return {'w': int(w), 'h': int(h), 'res': float(meta.get('resolution') or 0.05),
            'ox': float(origin[0]), 'oy': float(origin[1]),
            'oyaw': float(origin[2] if len(origin) > 2 else 0.0),
            'data': grid.reshape(-1).tolist(),
            'topic': 'bundle %s/%s' % (site, raster), 'frame': FRAME}, None


def _bundle_features(site, filename):
    path = os.path.join(_bundle_root(), site, filename)
    try:
        with open(path, 'r') as stream:
            return (json.load(stream) or {}).get('features') or []
    except (IOError, OSError, ValueError):
        return []


def load_bundle(site=None):
    """Statické vrstvy site bundlu v rámu `map` — tvar jako `world_query.collect`.

    {'data': {map, waypoints, zones, geofence}, 'missing', 'empty', 'site',
     'datum', 'dock', 'notes', 'source': 'bundle'}.  Nikdy nevyhazuje.
    """
    site = site or mc.active_site()
    out = {'data': {}, 'missing': {}, 'empty': {}, 'site': site, 'datum': None,
           'dock': None, 'notes': [], 'raw': {}, 'source': 'bundle'}
    if not site:
        for name in STATIC:
            out['missing'][name] = 'není žádný site bundle'
        out['missing']['robot'] = 'uložená data: poloha robota se z bundlu nezná'
        return out
    datum = mc._datum(site)
    out['datum'] = datum
    grid, error = load_bundle_grid(site)
    if grid is None:
        out['missing']['map'] = error
    else:
        out['data']['map'] = grid
    out['missing']['robot'] = 'uložená data: poloha robota se z bundlu nezná'
    if datum is None:
        why = 'site %s nemá datum.yaml — UTM nejde převést do rámu map' % site
        for name in ('waypoints', 'zones', 'geofence'):
            out['missing'][name] = why
        return out
    points, dock = [], None
    for feature in _bundle_features(site, 'waypoints.geojson'):
        geometry = feature.get('geometry') or {}
        props = feature.get('properties') or {}
        coords = geometry.get('coordinates') or []
        if geometry.get('type') != 'Point' or not props.get('name') or len(coords) < 2:
            continue
        x, y = mc.utm_to_map(coords[0], coords[1], datum)
        name = str(props['name'])
        points.append((name, x, y))
        if name.lower() in wq.DOCK_NAMES and (dock is None or name.lower() == 'docked'):
            yaw = props.get('yaw_rad')
            try:
                yaw = float(yaw) - datum[2] if yaw is not None else None
            except (TypeError, ValueError):
                yaw = None
            dock = {'name': name, 'x': x, 'y': y, 'yaw': yaw}
    paths = []
    for feature in _bundle_features(site, 'paths.geojson'):
        geometry = feature.get('geometry') or {}
        name = (feature.get('properties') or {}).get('name')
        if geometry.get('type') != 'LineString' or not name:
            continue
        line = [mc.utm_to_map(p[0], p[1], datum) for p in geometry.get('coordinates') or []
                if len(p) >= 2]
        if line:
            paths.append((str(name), line))
    if points or paths:
        out['data']['waypoints'] = {'points': points, 'paths': paths,
                                    'source': 'site bundle %s (přes datum)' % site}
    else:
        out['empty']['waypoints'] = 'site %s nemá uložené body ani trasy' % site
    zones = []
    for feature in _bundle_features(site, 'zones.geojson'):
        geometry = feature.get('geometry') or {}
        name = (feature.get('properties') or {}).get('name')
        rings = geometry.get('coordinates') or []
        if geometry.get('type') != 'Polygon' or not name or not rings:
            continue
        ring = [mc.utm_to_map(p[0], p[1], datum) for p in rings[0] if len(p) >= 2]
        if len(ring) > 3 and ring[0] == ring[-1]:
            ring = ring[:-1]
        if len(ring) >= 3:
            zones.append((str(name), ring))
    if zones:
        out['data']['zones'] = {'zones': zones}
    else:
        out['empty']['zones'] = 'site %s nemá uložené zóny' % site
    payload, error = mc._collect_geofence(site, datum)
    if isinstance(payload, dict) and payload.get('empty'):
        out['empty']['geofence'] = payload['empty']
    elif payload is None:
        out['missing']['geofence'] = error
    else:
        out['data']['geofence'] = payload
    out['dock'] = dock
    return out


def collect(source='auto', timeout_s=2.0, site=None, sources=None):
    """Data pro plátno: {'data', 'missing', 'empty', 'site', 'datum', 'dock', 'notes', 'source'}.

    `source` 'bundle' = jen uložená data (bez ROSu), 'live' = jen ROS,
    'auto' = ROS a statické vrstvy, které mlčí, z bundlu (s poznámkou).
    """
    source = source if source in SOURCES else 'auto'
    if source == 'bundle':
        return load_bundle(site)
    wanted = tuple(sources or ('robot', 'map', 'costmap', 'waypoints', 'zones',
                               'geofence', 'plan'))
    try:
        col = wq.collect(wanted, timeout_s, site)
    except Exception as exc:                            # noqa: BLE001
        col = {'data': {}, 'missing': dict((k, 'ROS: %s' % exc) for k in wanted),
               'empty': {}, 'site': site or mc.active_site(), 'datum': None, 'raw': {}}
    col.setdefault('notes', [])
    col['source'] = source
    col['dock'] = None
    if source == 'auto':
        bundle = load_bundle(col.get('site'))
        for name in STATIC:
            if name in col['data'] or name not in wanted:
                continue
            if name in bundle['data']:
                col['data'][name] = bundle['data'][name]
                why = col['missing'].pop(name, None) or col['empty'].pop(name, '')
                col['notes'].append('%s živě mlčí (%s) — vzato z uloženého bundlu'
                                    % (name, why or 'bez důvodu'))
        col['dock'] = bundle.get('dock')
        if col.get('datum') is None:
            col['datum'] = bundle.get('datum')
    if col['dock'] is None:
        pts = wq.points_table(col['data'].get('waypoints'))
        for name in ('DOCKED', 'docked', 'DOCK', 'dock'):
            if name in pts:
                col['dock'] = {'name': name, 'x': pts[name][0], 'y': pts[name][1], 'yaw': None}
                break
    return col


# ---------------------------------------------------------------------------
# geometrie (shapely, když je; jinak jednodušší náhrada)
# ---------------------------------------------------------------------------
def _shapely():
    try:
        from shapely.geometry import Point, Polygon, LineString   # noqa: F401
        return True
    except Exception:                                   # noqa: BLE001
        return False


def _ring(ring):
    pts = [(float(a), float(b)) for a, b in ring or []]
    if len(pts) > 3 and pts[0] == pts[-1]:
        pts = pts[:-1]
    return pts


def polygon_area(ring):
    pts = _ring(ring)
    s = 0.0
    for i in range(len(pts)):
        x1, y1 = pts[i]
        x2, y2 = pts[(i + 1) % len(pts)]
        s += x1 * y2 - x2 * y1
    return 0.5 * s


def centroid(ring):
    """Těžiště plochy (ne průměr vrcholů — ten táhne k hustě kresleným hranám)."""
    pts = _ring(ring)
    a = polygon_area(pts)
    if abs(a) < 1e-9:
        n = max(1, len(pts))
        return (sum(p[0] for p in pts) / n, sum(p[1] for p in pts) / n)
    cx = cy = 0.0
    for i in range(len(pts)):
        x1, y1 = pts[i]
        x2, y2 = pts[(i + 1) % len(pts)]
        cross = x1 * y2 - x2 * y1
        cx += (x1 + x2) * cross
        cy += (y1 + y2) * cross
    return (cx / (6.0 * a), cy / (6.0 * a))


def perimeter(ring, closed=True):
    pts = _ring(ring)
    n = len(pts) if closed else len(pts) - 1
    return sum(math.hypot(pts[(i + 1) % len(pts)][0] - pts[i][0],
                          pts[(i + 1) % len(pts)][1] - pts[i][1]) for i in range(max(0, n)))


def inside(xy, ring):
    return wq.point_in_polygon(float(xy[0]), float(xy[1]), _ring(ring))


def distance_to_edge(xy, ring):
    return wq.dist_to_ring(float(xy[0]), float(xy[1]), _ring(ring))


def _poly(ring):
    from shapely.geometry import Polygon
    poly = Polygon(_ring(ring))
    if not poly.is_valid:
        poly = poly.buffer(0)
    return poly


def _largest(geom):
    """Největší díl (buffer dovnitř může zónu roztrhnout na kusy)."""
    if geom is None or geom.is_empty:
        return None, 0
    parts = list(getattr(geom, 'geoms', [geom]))
    parts = [p for p in parts if p.geom_type == 'Polygon' and not p.is_empty]
    if not parts:
        return None, 0
    return max(parts, key=lambda p: p.area), len(parts)


def offset_ring(ring, offset_m):
    """Obvod zóny posunutý o `offset_m` (kladně ven, záporně dovnitř).

    {ok, ring[[x,y]...], length_m, parts, offset_m} — pro „objeď zónu" (ven)
    nebo „jeď kolem okraje uvnitř" (dovnitř).  Rohy ven se zaoblí (robot
    roh stejně objíždí obloukem), dovnitř zůstanou ostré.
    """
    if not _shapely():
        return {'ok': False, 'error': 'shapely chybí — odstup obvodu neumím spočítat'}
    try:
        poly = _poly(ring)
        off = float(offset_m)
        grown = poly.buffer(off, join_style=1 if off > 0 else 2, resolution=4)
        part, parts = _largest(grown)
    except Exception as exc:                            # noqa: BLE001
        return {'ok': False, 'error': 'odstup obvodu selhal: %s' % exc}
    if part is None:
        return {'ok': False, 'error': ('zóna je na odstup %.2f m moc úzká — dovnitř se '
                                       'nevejde' % off)}
    part = part.simplify(0.03)
    coords = [[_r(x), _r(y)] for x, y in list(part.exterior.coords)[:-1]]
    out = {'ok': True, 'offset_m': off, 'ring': coords,
           'length_m': _r(part.exterior.length), 'parts': parts}
    if parts > 1:
        out['note'] = ('odstup zónu rozdělil na %d kusy — vrácen největší' % parts)
    return out


def interior_point(ring, margin_m=DEFAULT_MARGIN_M, near=None):
    """Bod uvnitř zóny aspoň `margin_m` od hrany.

    `near` dáno → nejbližší takový bod k `near` (např. kam „dojet do zóny");
    jinak nejvnitřnější bod (pól nedostupnosti, ne těžiště: těžiště L-zóny
    leží mimo ni).  {ok, x, y, edge_m}.
    """
    pts = _ring(ring)
    if len(pts) < 3:
        return {'ok': False, 'error': 'zóna nemá aspoň 3 vrcholy'}
    if not _shapely():
        c = centroid(pts)
        if inside(c, pts) and distance_to_edge(c, pts) >= margin_m:
            return {'ok': True, 'x': _r(c[0]), 'y': _r(c[1]),
                    'edge_m': _r(distance_to_edge(c, pts)), 'note': 'bez shapely: těžiště'}
        return {'ok': False, 'error': 'shapely chybí a těžiště nesplní odstup'}
    from shapely.geometry import Point
    from shapely.ops import nearest_points, polylabel
    poly = _poly(pts)
    core, _parts = _largest(poly.buffer(-float(margin_m), join_style=2))
    if core is None:
        return {'ok': False, 'error': ('zóna je na odstup %.2f m od hrany moc úzká'
                                       % float(margin_m))}
    near_xy = _xy(near)
    if near_xy is not None:
        target = Point(near_xy)
        if core.contains(target):
            best = target
        else:
            best = nearest_points(core, target)[0]
            # nearest_points leží NA hraně jádra — o kousek dovnitř, ať je odstup jistý
            best = nearest_points(core.buffer(-0.02), target)[0] if not core.buffer(
                -0.02).is_empty else best
    else:
        best = polylabel(core, tolerance=0.02)
    x, y = best.x, best.y
    return {'ok': True, 'x': _r(x), 'y': _r(y), 'edge_m': _r(distance_to_edge((x, y), pts))}


def farthest_point(ring, from_xy, margin_m=DEFAULT_MARGIN_M):
    """Nejvzdálenější bod zóny od `from_xy`, aspoň `margin_m` od hrany.

    Vzdálenost od bodu je konvexní funkce, takže maximum přes mnohoúhelník
    leží ve vrcholu — stačí projít vrcholy zóny zúžené o odstup.
    {ok, x, y, dist_m, edge_m}.
    """
    src = _xy(from_xy)
    pts = _ring(ring)
    if src is None or len(pts) < 3:
        return {'ok': False, 'error': 'chybí výchozí bod nebo zóna nemá 3 vrcholy'}
    cands = pts
    if margin_m and _shapely():
        core, _parts = _largest(_poly(pts).buffer(-float(margin_m), join_style=2))
        if core is None:
            return {'ok': False, 'error': ('zóna je na odstup %.2f m od hrany moc úzká'
                                           % float(margin_m))}
        cands = list(core.exterior.coords)[:-1]
    best = max(cands, key=lambda p: math.hypot(p[0] - src[0], p[1] - src[1]))
    return {'ok': True, 'x': _r(best[0]), 'y': _r(best[1]),
            'dist_m': _r(math.hypot(best[0] - src[0], best[1] - src[1])),
            'edge_m': _r(distance_to_edge(best, pts))}


def nearest_point(ring, from_xy):
    """Nejbližší bod zóny (hrana nebo uvnitř) k `from_xy`: {x, y, dist_m, inside}."""
    src = _xy(from_xy)
    pts = _ring(ring)
    if src is None or len(pts) < 3:
        return None
    if inside(src, pts):
        return {'x': _r(src[0]), 'y': _r(src[1]), 'dist_m': 0.0, 'inside': True}
    best = None
    for i in range(len(pts)):
        ax, ay = pts[i]
        bx, by = pts[(i + 1) % len(pts)]
        dx, dy = bx - ax, by - ay
        t = 0.0 if dx == dy == 0 else max(0.0, min(1.0, ((src[0] - ax) * dx + (src[1] - ay) * dy)
                                                   / (dx * dx + dy * dy)))
        px, py = ax + t * dx, ay + t * dy
        d = math.hypot(px - src[0], py - src[1])
        if best is None or d < best[2]:
            best = (px, py, d)
    return {'x': _r(best[0]), 'y': _r(best[1]), 'dist_m': _r(best[2]), 'inside': False}


def bbox(points):
    xs = [float(p[0]) for p in points]
    ys = [float(p[1]) for p in points]
    if not xs:
        return None
    return {'west': _r(min(xs)), 'south': _r(min(ys)), 'east': _r(max(xs)),
            'north': _r(max(ys))}


# ---------------------------------------------------------------------------
# jména: body, zóny, trasy, dok, robot
# ---------------------------------------------------------------------------
def places(col):
    """{'points': {jméno: (x, y)}, 'zones': {jméno: ring}, 'paths': {jméno: line}}."""
    wp = col['data'].get('waypoints') or {}
    return {'points': wq.points_table(wp),
            'zones': dict((n, r) for n, r in wq._zones_of(col)),
            'paths': dict((str(n), [(float(a), float(b)) for a, b in line])
                          for n, line in wp.get('paths') or [])}


def _find(table, name):
    if name in table:
        return name
    low = str(name).strip().lower().replace(' ', '')
    for key in table:
        if key.lower().replace(' ', '') == low:
            return key
    # „zóna B", „zone b", „B" -> ZoneB; „trasa zóny B" se řeší u tras
    for prefix in ('zóna', 'zona', 'zone'):
        if low.startswith(prefix):
            low = low[len(prefix):].lstrip('_-')
    for key in table:
        k = key.lower()
        for prefix in ('zone', 'zona', 'zóna'):
            if k.startswith(prefix):
                k = k[len(prefix):].lstrip('_-')
        if k == low and low:
            return key
    return None


def resolve(target, col):
    """Cokoli, na co jde ukázat -> {kind, name, x, y, ring?, line?} nebo {kind: None, error}.

    Pořadí: souřadnice, 'robot', 'dock', bod, zóna, trasa.  Jména bez ohledu na
    velikost písmen; zóna i jako „zóna B" / „B".
    """
    xy = _xy(target)
    if xy is not None:
        name = target.get('name') if isinstance(target, dict) else None
        return {'kind': 'xy', 'name': name, 'x': xy[0], 'y': xy[1]}
    text = str(target or '').strip()
    low = text.lower()
    if low in ('robot', 'me', 'ja', 'já', ''):
        robot = col['data'].get('robot')
        if robot:
            return {'kind': 'robot', 'name': 'robot', 'x': float(robot['x']),
                    'y': float(robot['y']), 'yaw': float(robot.get('yaw') or 0.0)}
        return {'kind': None, 'error': 'poloha robota není známá: %s'
                % col['missing'].get('robot', 'bez důvodu')}
    if low in ('dock', 'dok', 'docked', 'nabijecka', 'nabíječka'):
        dock = col.get('dock')
        if dock:
            return {'kind': 'dock', 'name': dock['name'], 'x': dock['x'], 'y': dock['y'],
                    'yaw': dock.get('yaw')}
    pl = places(col)
    key = _find(pl['points'], text)
    if key:
        x, y = pl['points'][key]
        return {'kind': 'point', 'name': key, 'x': x, 'y': y}
    key = _find(pl['zones'], text)
    if key:
        ring = pl['zones'][key]
        c = centroid(ring)
        return {'kind': 'zone', 'name': key, 'x': c[0], 'y': c[1], 'ring': ring}
    key = _find(pl['paths'], text)
    if key:
        line = pl['paths'][key]
        mid = line[len(line) // 2]
        return {'kind': 'path', 'name': key, 'x': mid[0], 'y': mid[1], 'line': line}
    return {'kind': None, 'error': 'neznám %r (body: %s; zóny: %s; trasy: %s)'
            % (text, ', '.join(sorted(pl['points'])) or '—',
               ', '.join(sorted(pl['zones'])) or '—', ', '.join(sorted(pl['paths'])) or '—')}


def _extent(ref):
    """Body, které musí být ve výřezu, aby byl `ref` celý vidět."""
    if ref.get('ring'):
        return list(ref['ring'])
    if ref.get('line'):
        return list(ref['line'])
    return [(ref['x'], ref['y'])]


def _square(cx, cy, span):
    half = 0.5 * max(MIN_SPAN_M, float(span))
    return (cx - half, cy - half, cx + half, cy + half)


def _fit(points, pad=FIT_PAD_M):
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    west, east = min(xs) - pad, max(xs) + pad
    south, north = min(ys) - pad, max(ys) + pad
    if east - west < MIN_SPAN_M:
        mid = 0.5 * (west + east)
        west, east = mid - 0.5 * MIN_SPAN_M, mid + 0.5 * MIN_SPAN_M
    if north - south < MIN_SPAN_M:
        mid = 0.5 * (south + north)
        south, north = mid - 0.5 * MIN_SPAN_M, mid + 0.5 * MIN_SPAN_M
    return (west, south, east, north)


def normalise_layers(layers):
    """(vrstvy plátna, stížnosti) — jako `map_compose.normalise_layers` + dock/sketch."""
    if layers is None:
        return list(DEFAULT_LAYERS), []
    if isinstance(layers, str):
        layers = [p.strip() for p in layers.replace(';', ',').split(',')]
    wanted, complaints, rest = [], [], []
    for name in layers:
        name = str(name or '').strip().lower()
        if not name:
            continue
        if name == 'all':
            wanted = list(LAYERS)
            continue
        name = ALIASES.get(name, name)
        if name in ('dock', 'sketch'):
            if name not in wanted:
                wanted.append(name)
        else:
            rest.append(name)
    if rest:
        got, complaints = mc.normalise_layers(rest)
        wanted.extend(n for n in got if n not in wanted)
    if not wanted:
        wanted = list(DEFAULT_LAYERS)
    return [n for n in LAYERS if n in wanted], complaints


def _prefetched(col, layers):
    """{vrstva map_compose: (payload, chyba)} pro VŠECHNY kreslené vrstvy.

    Každá vrstva musí být předaná — i ta chybějící, s důvodem — jinak by si ji
    `map_compose` šel číst z ROSu, a render z bundlu by na robotu bez ROSu visel.
    """
    names = {'grid': 'map', 'lidar': 'scan'}
    pre = {}
    for layer in list(layers) + ['robot']:
        if layer in ('dock', 'sketch'):
            continue
        ours = names.get(layer, layer)
        raw = (col.get('raw') or {}).get(ours)
        if ours in col['data']:
            pre[layer] = (col['data'][ours], None)
        elif ours in col['empty']:
            pre[layer] = ({'empty': col['empty'][ours]}, None)
        elif ours in col['missing']:
            pre[layer] = (None, col['missing'][ours])
        elif col.get('source') == 'bundle':
            pre[layer] = (None, 'uložená data: vrstva %s v bundlu není' % layer)
        elif raw is not None:
            pre[layer] = raw
    return pre


def _dock_overlay(dock, bounds=None):
    if not dock:
        return []
    if bounds and not (bounds[0] <= dock['x'] <= bounds[2]
                       and bounds[1] <= dock['y'] <= bounds[3]):
        return []
    return [{'x': dock['x'], 'y': dock['y'], 'kind': 'dock', 'label': 'DOK'}]


def _merge_overlay(*parts):
    out = {}
    for part in parts:
        for key, value in (part or {}).items():
            if isinstance(value, list):
                out.setdefault(key, []).extend(value)
    return out


def _objects(col, meta, drawn, center_ref=None):
    """Seznam vykreslených objektů s map souřadnicemi i pixely (jen ty ve výřezu)."""
    b = meta['bbox']

    def within(x, y, pad=0.0):
        return (b['west'] - pad <= x <= b['east'] + pad
                and b['south'] - pad <= y <= b['north'] + pad)

    def px(x, y):
        return to_px(meta, (x, y))

    objs = []
    robot = col['data'].get('robot')
    if robot and 'robot' in drawn:
        az, yaw_deg, facing = mc.heading(float(robot['yaw']))
        objs.append({'kind': 'robot', 'name': 'robot', 'x': _r(robot['x']),
                     'y': _r(robot['y']), 'yaw_deg': _r(yaw_deg, 1), 'facing': facing,
                     'px': px(robot['x'], robot['y'])})
    dock = col.get('dock')
    if dock and 'dock' in drawn and within(dock['x'], dock['y']):
        objs.append({'kind': 'dock', 'name': dock['name'], 'x': _r(dock['x']),
                     'y': _r(dock['y']), 'px': px(dock['x'], dock['y'])})
    pl = places(col)
    if 'waypoints' in drawn:
        for name, (x, y) in sorted(pl['points'].items()):
            if within(x, y):
                objs.append({'kind': 'point', 'name': name, 'x': _r(x), 'y': _r(y),
                             'px': px(x, y)})
        for name, line in sorted(pl['paths'].items()):
            if any(within(x, y) for x, y in line):
                objs.append({'kind': 'path', 'name': name,
                             'start': [_r(line[0][0]), _r(line[0][1])],
                             'end': [_r(line[-1][0]), _r(line[-1][1])],
                             'length_m': _r(perimeter(line, closed=False)),
                             'points': [[_r(x), _r(y)] for x, y in line][:60]})
    if 'zones' in drawn:
        for name, ring in sorted(pl['zones'].items()):
            if any(within(x, y, 1.0) for x, y in ring):
                c = centroid(ring)
                objs.append({'kind': 'zone', 'name': name, 'centroid': [_r(c[0]), _r(c[1])],
                             'centroid_px': px(*c), 'area_m2': _r(abs(polygon_area(ring)), 1),
                             'bbox': bbox(ring), 'ring': [[_r(x), _r(y)] for x, y in ring]})
    return objs


def view(center='robot', span_m=None, layers=None, size=DEFAULT_SIZE, fit=None,
         source='auto', overlay=None, out=None, site=None, collected=None,
         timeout_s=2.0, keep=mc.DEFAULT_KEEP, axes=True):
    """Výřez mapy se souřadnicovou mřížkou + metadata.  Vždy dict, nikdy nevyhazuje.

        {ok: True, path, meta: {frame, site, size, bbox, transform, grid, center,
         layers, empty, missing, warnings, notes, objects[], legend}}

    `center`  'robot' | 'dock' | jméno bodu/zóny/trasy | [x, y] | 'x,y' | {'x','y'}
              | 'map' (celá mapa).  Zóna a trasa bez `span_m` se vejdou celé.
    `span_m`  strana čtvercového výřezu v metrech (min. 6 m).
    `fit`     seznam věcí (jména, [x, y], prstence), které se mají vejít všechny
              — přebije `center`/`span_m`.
    `layers`  vrstvy (`LAYERS`, aliasy jako map_compose; 'obstacles' = costmap),
              None = `DEFAULT_LAYERS`.
    `overlay` překryv ke kreslení (skica): tvar `map_compose` overlay —
              paths/markers/polygons/arrows/texts/notes.
    `collected` data už jednou sebraná (`collect()`), ať se nečtou dvakrát.
    """
    try:
        return _view(center, span_m, layers, size, fit, source, overlay, out, site,
                     collected, timeout_s, keep, axes)
    except Exception as exc:                            # noqa: BLE001
        return {'ok': False, 'error': 'Neočekávaná chyba plátna: %s: %s'
                % (type(exc).__name__, exc)}


def _view(center, span_m, layers, size, fit, source, overlay, out, site, collected,
          timeout_s, keep, axes):
    wanted, complaints = normalise_layers(layers)
    col = collected or collect(source, timeout_s, site)
    warnings = list(complaints)
    ref = None
    if fit:
        pts = []
        for item in (fit if isinstance(fit, (list, tuple)) and not _xy(fit) else [fit]):
            if isinstance(item, (list, tuple)) and item and isinstance(item[0], (list, tuple)):
                pts.extend((float(a), float(b)) for a, b in item)
                continue
            got = resolve(item, col)
            if got.get('kind'):
                pts.extend(_extent(got))
            else:
                warnings.append(got.get('error'))
        if not pts:
            return {'ok': False, 'error': 'nic z `fit` neznám: %s' % '; '.join(
                w for w in warnings if w)}
        bounds = _fit(pts)
        ref = {'kind': 'fit', 'name': None, 'x': 0.5 * (bounds[0] + bounds[2]),
               'y': 0.5 * (bounds[1] + bounds[3])}
    elif str(center).strip().lower() in ('map', 'site', 'all', 'mapa', 'celá'):
        bounds = None
        ref = {'kind': 'map', 'name': None}
    else:
        ref = resolve(center, col)
        if not ref.get('kind') and str(center).strip().lower() in ('robot', ''):
            dock = col.get('dock')
            if dock:
                warnings.append('%s — výřez je kolem doku' % ref.get('error'))
                ref = {'kind': 'dock', 'name': dock['name'], 'x': dock['x'], 'y': dock['y']}
        if not ref.get('kind'):
            return {'ok': False, 'error': ref.get('error'), 'missing': col['missing']}
        if span_m:
            bounds = _square(ref['x'], ref['y'], span_m)
        elif ref['kind'] in ('zone', 'path'):
            bounds = _fit(_extent(ref))
        else:
            bounds = _square(ref['x'], ref['y'], DEFAULT_SPAN_M.get(ref['kind'], 12.0))
    compose_layers = [n for n in wanted if n not in ('dock', 'sketch')]
    extra = {'markers': _dock_overlay(col.get('dock'), bounds)} if 'dock' in wanted else {}
    drawn_overlay = _merge_overlay(extra, overlay if ('sketch' in wanted or overlay) else None)
    for note in col.get('notes') or []:
        drawn_overlay.setdefault('notes', []).append(note)
    result = mc.compose(layers=compose_layers, size=size, center='map', radius=None,
                        out=out, timeout_s=timeout_s, keep=keep, site=col.get('site'),
                        overlay=drawn_overlay, prefetched=_prefetched(col, compose_layers),
                        bounds=bounds, axes=axes)
    if not result.get('ok'):
        return {'ok': False, 'error': result.get('error'),
                'warnings': warnings + list(result.get('warnings') or [])}
    grid = col['data'].get('map') or {}
    meta = {
        'frame': FRAME, 'site': result.get('site'), 'png': result['path'],
        'size': result['size'], 'bbox': result['bbox'], 'transform': result['transform'],
        'm_per_px': result['transform']['m_per_px'],
        'grid': ({'origin': [_r(grid['ox'], 3), _r(grid['oy'], 3)], 'res': grid['res'],
                  'w': grid['w'], 'h': grid['h'], 'source': grid.get('topic')}
                 if grid else None),
        'center': dict((k, (_r(v) if isinstance(v, float) else v)) for k, v in ref.items()
                       if k in ('kind', 'name', 'x', 'y')),
        'source': col.get('source'),
        'layers': list(result.get('layers') or []) + [n for n in ('dock',) if n in wanted
                                                       and col.get('dock')],
        'empty': result.get('empty'), 'missing': result.get('missing'),
        'warnings': warnings + list(result.get('warnings') or []),
        'notes': list(col.get('notes') or []),
        'stamp': result.get('stamp'),
        'legend': result.get('legend'),
    }
    meta['objects'] = _objects(col, meta, set(meta['layers']), ref)
    return {'ok': True, 'path': result['path'], 'meta': meta,
            'summary': describe(meta)}


def describe(meta):
    """Jedna věta k obrázku: co je kde, v číslech — ne popis obsahu."""
    b = meta.get('bbox') or {}
    bits = ['výřez x %.1f…%.1f m, y %.1f…%.1f m (rám map, mřížka po %g m)'
            % (b.get('west', 0), b.get('east', 0), b.get('south', 0), b.get('north', 0),
               (meta.get('transform') or {}).get('axes_step_m') or 0)]
    for obj in meta.get('objects') or []:
        if obj['kind'] == 'robot':
            bits.append('robot %.2f, %.2f míří %s' % (obj['x'], obj['y'], obj['facing']))
        elif obj['kind'] == 'dock':
            bits.append('dok %.2f, %.2f' % (obj['x'], obj['y']))
        elif obj['kind'] == 'zone':
            bits.append('zóna %s těžiště %.2f, %.2f' % (obj['name'], obj['centroid'][0],
                                                     obj['centroid'][1]))
    if meta.get('missing'):
        bits.append('NEDOSTUPNÉ: %s' % ', '.join(meta['missing']))
    return ' · '.join(bits)


# ---------------------------------------------------------------------------
# převody pixel <-> mapa (přes metadata konkrétního renderu)
# ---------------------------------------------------------------------------
def _transform(meta):
    t = (meta or {}).get('transform') or meta or {}
    return (float(t['x0']), float(t['y0']), float(t['w']), float(t['h']),
            float(t['cx']), float(t['cy']), float(t['m_per_px']))


def _many(value):
    return (isinstance(value, (list, tuple)) and value
            and isinstance(value[0], (list, tuple, dict)))


def to_map(meta, px):
    """Pixel celého PNG -> [x, y] v rámu map.  Bere [px, py] nebo jejich seznam."""
    if _many(px):
        return [to_map(meta, p) for p in px]
    x0, y0, w, h, cx, cy, s = _transform(meta)
    p = _xy(px)
    return [round(cx + (p[0] - x0 - 0.5 * w) * s, 3),
            round(cy - (p[1] - y0 - 0.5 * h) * s, 3)]


def to_px(meta, xy):
    """[x, y] v rámu map -> pixel celého PNG.  Bere [x, y] nebo jejich seznam."""
    if _many(xy):
        return [to_px(meta, p) for p in xy]
    x0, y0, w, h, cx, cy, s = _transform(meta)
    p = _xy(xy)
    return [round(x0 + 0.5 * w + (p[0] - cx) / s, 1),
            round(y0 + 0.5 * h - (p[1] - cy) / s, 1)]


def in_view(meta, xy):
    b = meta.get('bbox') or {}
    p = _xy(xy)
    return bool(p and b and b['west'] <= p[0] <= b['east'] and b['south'] <= p[1] <= b['north'])


# ---------------------------------------------------------------------------
# kontrola bodů: volno / zóna / geofence / dosažitelnost / nejbližší volné místo
# ---------------------------------------------------------------------------
def cell_state(costgrid, xy):
    """'free' | 'inflated' | 'lethal' | 'unknown' | 'outside' + hodnota buňky."""
    if costgrid is None:
        return None, None
    v = costgrid.value(xy[0], xy[1])
    if v is None:
        return 'outside', None
    if v < 0:
        return 'unknown', v
    if v >= wq.LETHAL:
        return 'lethal', v
    if v >= wq.INSCRIBED:
        return 'inflated', v
    return 'free', v


def nearest_free(costgrid, xy, radius_m=NEAR_FREE_RADIUS_M):
    """Nejbližší volná buňka (cost < inscribed, známá) do `radius_m`: {x, y, dist_m} | None."""
    if costgrid is None or np is None:
        return None
    res = costgrid.res
    c0 = int(math.floor((xy[0] - costgrid.ox) / res))
    r0 = int(math.floor((xy[1] - costgrid.oy) / res))
    k = int(math.ceil(radius_m / res))
    rlo, rhi = max(0, r0 - k), min(costgrid.h, r0 + k + 1)
    clo, chi = max(0, c0 - k), min(costgrid.w, c0 + k + 1)
    if rlo >= rhi or clo >= chi:
        return None
    sub = costgrid.a[rlo:rhi, clo:chi]
    rr, cc = np.nonzero((sub >= 0) & (sub < wq.INSCRIBED))
    if not len(rr):
        return None
    xs = costgrid.ox + (cc + clo + 0.5) * res
    ys = costgrid.oy + (rr + rlo + 0.5) * res
    d = np.hypot(xs - xy[0], ys - xy[1])
    i = int(np.argmin(d))
    if d[i] > radius_m:
        return None
    return {'x': _r(xs[i]), 'y': _r(ys[i]), 'dist_m': _r(d[i])}


def check(points, collected=None, start=None, source='auto', site=None,
          timeout_s=2.0, reach=True, radius_free_m=NEAR_FREE_RADIUS_M):
    """Kontrola bodů plánu.  Vždy dict, nikdy nevyhazuje.

    `points`  seznam: [x, y] | {'x','y','label'} | jméno bodu/zóny ('zóna' = její
              nejvnitřnější bod) | 'dock' | 'robot'.
    `start`   odkud se počítá dosažitelnost: None = robot (bez polohy robota dok),
              jinak cokoli, co bere `resolve`.

        {ok, start{kind,name,x,y}, costmap_source, points: [{label, x, y, cell,
         cost, free, in_zones[], in_geofence, dist_m, reach{reachable, length_m,
         reason}, nearest_free{x,y,dist_m}, ok, summary}], summary, notes}
    """
    try:
        return _check(points, collected, start, source, site, timeout_s, reach,
                      radius_free_m)
    except Exception as exc:                            # noqa: BLE001
        return {'ok': False, 'error': 'Neočekávaná chyba kontroly: %s: %s'
                % (type(exc).__name__, exc)}


def _check(points, col, start, source, site, timeout_s, do_reach, radius_free_m):
    col = col or collect(source, timeout_s, site)
    costgrid = wq._costgrid(col)
    clearance = wq._clearance(costgrid) if (costgrid is not None and do_reach) else None
    pl = places(col)
    fence = (col['data'].get('geofence') or {}).get('ring')
    if start is None:
        sref = resolve('robot', col)
        if not sref.get('kind'):
            sref = resolve('dock', col) if col.get('dock') else sref
    else:
        sref = resolve(start, col)
    sxy = (sref['x'], sref['y']) if sref.get('kind') else None
    rows = []
    for raw in points if isinstance(points, (list, tuple)) and not _xy(points) else [points]:
        label = raw.get('label') if isinstance(raw, dict) else None
        ref = resolve(raw, col)
        if not ref.get('kind'):
            rows.append({'label': label or str(raw), 'ok': False, 'error': ref.get('error')})
            continue
        if ref['kind'] == 'zone':
            ip = interior_point(ref['ring'], DEFAULT_MARGIN_M)
            if ip.get('ok'):
                ref = dict(ref, x=ip['x'], y=ip['y'])
        xy = (float(ref['x']), float(ref['y']))
        row = {'label': label or ref.get('name') or '%.2f, %.2f' % xy,
               'x': _r(xy[0]), 'y': _r(xy[1])}
        state, value = cell_state(costgrid, xy)
        row['cell'] = state
        row['cost'] = value
        row['free'] = state == 'free'
        row['in_zones'] = [n for n, ring in sorted(pl['zones'].items()) if inside(xy, ring)]
        row['in_geofence'] = None if not fence else bool(inside(xy, fence))
        if sxy is not None:
            row['dist_m'] = _r(math.hypot(xy[0] - sxy[0], xy[1] - sxy[1]))
        if not row['free'] and costgrid is not None:
            row['nearest_free'] = nearest_free(costgrid, xy, radius_free_m)
        if do_reach and sxy is not None and costgrid is not None:
            got = wq.plan(costgrid, sxy, xy, clearance)
            row['reach'] = {'reachable': got.get('reachable'),
                            'length_m': got.get('length_m'),
                            'reason': got.get('reason'),
                            'min_clearance_m': got.get('min_clearance_m'),
                            'goal_adjusted_m': got.get('goal_adjusted_m')}
            if got.get('points'):
                row['reach']['path'] = got['points']
        row['ok'] = bool(row['free'] and (row.get('in_geofence') is not False)
                         and (not do_reach or (row.get('reach') or {}).get('reachable')))
        row['summary'] = _point_sentence(row, sref)
        rows.append(row)
    notes = list(col.get('notes') or [])
    out = {'ok': costgrid is not None, 'points': rows,
           'start': dict((k, (_r(v) if isinstance(v, float) else v))
                         for k, v in sref.items() if k in ('kind', 'name', 'x', 'y'))
           if sref.get('kind') else {'error': sref.get('error')},
           'costmap_source': None if costgrid is None else costgrid.topic,
           'notes': notes, 'missing': col.get('missing')}
    if costgrid is None:
        out['error'] = 'nemám costmapu ani mapu — volno ani cestu nejde ověřit'
    bad = [r['label'] for r in rows if not r.get('ok')]
    out['summary'] = ('všechny body (%d) jsou volné a dosažitelné' % len(rows) if not bad
                      else 'neprošly: %s' % ', '.join(str(b) for b in bad))
    return out


def _point_sentence(row, sref):
    where = '%.2f, %.2f' % (row['x'], row['y'])
    head = where if row['label'] == where else '%s (%s)' % (row['label'], where)
    bits = ['%s: %s' % (head, CELL_WORDS.get(row['cell'], 'stav buňky neznám'))]
    if row['in_zones']:
        bits.append('v zóně %s' % ', '.join(row['in_zones']))
    else:
        bits.append('mimo zóny')
    if row.get('in_geofence') is False:
        bits.append('MIMO geofence')
    reach = row.get('reach') or {}
    if reach.get('reachable'):
        bits.append('dosažitelné z %s, cesta %.1f m' % (sref.get('name') or sref.get('kind'),
                                                       reach.get('length_m') or 0.0))
    elif reach:
        bits.append('NEDOSAŽITELNÉ: %s' % (reach.get('reason') or '?'))
    nf = row.get('nearest_free')
    if nf:
        bits.append('nejbližší volno %.2f, %.2f (%.2f m)' % (nf['x'], nf['y'], nf['dist_m']))
    return ', '.join(bits)


# ---------------------------------------------------------------------------
# geometrie zóny jedním voláním
# ---------------------------------------------------------------------------
def zone_geometry(name, from_xy=None, margin_m=DEFAULT_MARGIN_M, offset_m=None,
                  collected=None, source='auto', site=None, timeout_s=2.0):
    """Všechno, co plánovač potřebuje vědět o zóně.  Vždy dict.

        {ok, name, ring, area_m2, perimeter_m, centroid, bbox, interior{x,y,edge_m},
         from{...}, farthest{x,y,dist_m,edge_m}, nearest{x,y,dist_m,inside},
         offset{ring,length_m,offset_m}}

    `from_xy` 'robot' (výchozí), 'dock', jméno nebo [x, y] — k němu se počítá
    nejvzdálenější a nejbližší bod.  `offset_m` kladně = obvod zóny o tolik
    VEN (objetí zóny), záporně = dovnitř.  `name` může být i přímo prstenec.
    """
    try:
        col = collected or collect(source, timeout_s, site)
        if isinstance(name, (list, tuple)) and name and isinstance(name[0], (list, tuple)):
            ref = {'kind': 'zone', 'name': None, 'ring': _ring(name)}
        else:
            ref = resolve(name, col)
            if ref.get('kind') != 'zone':
                return {'ok': False, 'error': ref.get('error') or '%r není zóna' % (name,)}
        ring = _ring(ref['ring'])
        c = centroid(ring)
        out = {'ok': True, 'name': ref.get('name'),
               'ring': [[_r(x), _r(y)] for x, y in ring],
               'area_m2': _r(abs(polygon_area(ring)), 2), 'perimeter_m': _r(perimeter(ring)),
               'centroid': [_r(c[0]), _r(c[1])], 'centroid_inside': bool(inside(c, ring)),
               'bbox': bbox(ring), 'margin_m': margin_m,
               'interior': interior_point(ring, margin_m)}
        src = resolve(from_xy if from_xy is not None else 'robot', col)
        if not src.get('kind') and from_xy is None and col.get('dock'):
            src = resolve('dock', col)
        if src.get('kind'):
            sxy = (src['x'], src['y'])
            out['from'] = {'kind': src['kind'], 'name': src.get('name'),
                           'x': _r(sxy[0]), 'y': _r(sxy[1])}
            out['farthest'] = farthest_point(ring, sxy, margin_m)
            out['nearest'] = nearest_point(ring, sxy)
            out['interior_near'] = interior_point(ring, margin_m, near=sxy)
        else:
            out['from'] = {'error': src.get('error')}
        if offset_m is not None:
            out['offset'] = offset_ring(ring, offset_m)
        return out
    except Exception as exc:                            # noqa: BLE001
        return {'ok': False, 'error': 'geometrie zóny selhala: %s: %s'
                % (type(exc).__name__, exc)}


# ---------------------------------------------------------------------------
# CLI (rosrun vitulus_navi map_canvas view|check|zone)
# ---------------------------------------------------------------------------
def main(argv=None):
    import argparse
    parser = argparse.ArgumentParser(description='Mapa jako plátno: výřez se souřadnicemi, '
                                                 'kontrola bodů, geometrie zóny.')
    parser.add_argument('cmd', choices=('view', 'check', 'zone'))
    parser.add_argument('--center', default='robot')
    parser.add_argument('--span', type=float, default=None)
    parser.add_argument('--fit', default=None, help='jména/x,y oddělená středníkem')
    parser.add_argument('--layers', default=None)
    parser.add_argument('--size', type=int, default=DEFAULT_SIZE)
    parser.add_argument('--source', default='auto', choices=SOURCES)
    parser.add_argument('--site', default=None)
    parser.add_argument('--out', default=None)
    parser.add_argument('--points', default=None, help='check: "x,y;TEST;ZoneB"')
    parser.add_argument('--start', default=None)
    parser.add_argument('--name', default=None, help='zone: jméno zóny')
    parser.add_argument('--from', dest='from_xy', default=None)
    parser.add_argument('--margin', type=float, default=DEFAULT_MARGIN_M)
    parser.add_argument('--offset', type=float, default=None)
    args = parser.parse_args(argv)
    if args.source != 'bundle':
        mc.ensure_node('vitulus_map_canvas')
    if args.cmd == 'view':
        fit = args.fit.split(';') if args.fit else None
        result = view(center=args.center, span_m=args.span, layers=args.layers,
                      size=args.size, fit=fit, source=args.source, out=args.out,
                      site=args.site)
    elif args.cmd == 'check':
        pts = (args.points or '').split(';')
        result = check(pts, start=args.start, source=args.source, site=args.site)
        for row in result.get('points') or []:
            row.get('reach', {}).pop('path', None)
    else:
        result = zone_geometry(args.name, from_xy=args.from_xy, margin_m=args.margin,
                               offset_m=args.offset, source=args.source, site=args.site)
    print(json.dumps(result, ensure_ascii=False, indent=1))
    return 0 if result.get('ok') else 1


if __name__ == '__main__':
    raise SystemExit(main())
