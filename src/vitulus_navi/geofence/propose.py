"""Geofence navržený klikáním do mapy — převod a kontrola (BEZ podpisu).

Robot-native zrcadlo
--------------------
Tenhle modul je robotí (public) kopie návrhové části agentího
`vitulus_claude/geofence_propose.py`.  Žije v `vitulus_navi` (bezpečnostně
příbuzná nav vrstva), aby ho mohl importovat webnode (:7779, public balík)
BEZ jakékoli závislosti na agentovi.  `fence.py` a `sitebundle.py` vedle něj
jsou verbatim kopie `geofence.py` / `sitebundle.py` z agenta — digest i formát
souboru proto vycházejí bit-identicky s agentí cestou.

Záměrně **propose-only**: `sign_proposal`/`_human_check`/audit tu NEJSOU.
Aktivní `geofence.geojson` vzniká jedině lidským podpisem přes agentí CLI
`tools/geofence_propose sign` (nebo `/geofence podepsat <digest>`) — tenhle
modul (a tím pádem ani žádný HTTP request na webnode) fyzicky nemá funkci,
která by plot podepsala.  Podepisovací brána (AGENT.md §11.1, §11.5) tím
zůstává beze změny.

Proč to existuje
----------------
`tools/geofence_record` nechá člověka **projet** obvod a z dráhy `base_link`u
udělá prstenec.  To je poctivé, ale drahé: vyžaduje jízdu, čas a to, že robot
vůbec jede — a k 2026-08-23 navíc nefunguje, protože rámec TF `utm` na živém
systému neexistuje (`navsat_transform` neběží), takže `utmpose.UtmPoseReader`
nemá co číst.

Tenhle modul je druhá cesta ke stejnému souboru: člověk **naklikaĺ** vrcholy
v mapovém pohledu web UI, my je převedeme z rámce `map` do UTM přes datum
daného site a uložíme **návrh**.  Nic víc.  Aktivní `geofence.geojson` z toho
udělá teprve podpis člověka — a to jen v agentovi, ne tady.

Čtyři pravidla, ze kterých plyne všechno ostatní
------------------------------------------------
1. **Geometrii nevymýšlíme znovu.**  Minimální počet vrcholů, minimální plocha
   a test sebeprotnutí jsou v `geofence.Fence` a supervisor je používá; návrh
   projde *tou samou* třídou, jinak by šlo podepsat prstenec, který supervisor
   při armování odmítne.

2. **Převod map -> UTM je na jednom místě** (`map_to_utm`).  Datum je
   `~/.vitulus/mapping_v3/<site>/datum.yaml`; vzorec je otočení o `yaw_rad`
   a posun o `(utm_e, utm_n)`.  Ověřeno na živém robotu 2026-08-23: pozice
   `map <- base_link` (2.483, 0.126) vyjde na UTM (493076.183, 5540714.800),
   což je waypoint `docked` z `waypoints.geojson` na 0.000 m.  Ta dvojice je
   fixture v `test/test_geofence_propose.py`.

3. **Neznámo není volno.**  Každý vrchol *a každá hrana* musí ležet ve
   **známých** buňkách occupancy gridu.  Hrany se vzorkují, ne jen konce:
   plot vedený přes nezmapovaný pruh vypadá v datech stejně dobře jako plot
   vedený po trávníku, a přesně tam robot nemá co dělat.  Když to neprojde,
   modul **odmítne** a řekne který úsek a kde.

4. **Podpis je lidský akt** (AGENT.md §11.1, §11.5).  Navrhnout smí i robot,
   nainstalovat nikdy.  Tenhle robotí modul podpis **vůbec neumí** — funkce
   `sign_proposal()` žije jen v agentím `geofence_propose.py` a je jediná, co
   zapisuje `geofence.geojson`; pouští k tomu jen jméno člověka z lidského
   kanálu (`agent.capabilities._actor_is_human`).  Tím, že tu ta funkce není,
   nemůže žádný HTTP request na webnode plot podepsat.

Rodokmen v souboru
------------------
Zapsaný dokument nese `properties.source` a `properties.provenance`.  Návrh
z kliknutí má `source = "map_click"`; prstenec projetý robotem (výstup
`geofence_record`) žádný `source` nemá, takže **chybějící pole se čte jako
„projetý / neznámý původ"** a nikdy ne jako „naklikaný".  Komu na tom záleží
(supervisor, runner, reporty), pozná rozdíl bez hádání.
"""
import hashlib
import json
import math
import os
import re
import time

from . import sitebundle
from .fence import Fence, GeofenceError, dump_geofence, load_geofence

# Jak se pozná naklikaný plot od projetého.
SOURCE_CLICK = 'map_click'
SOURCE_DRIVEN = 'perimeter_drive'

STATUS_PROPOSAL = 'proposal'
STATUS_ACTIVE = 'active'

PROPOSAL_NAME = 'geofence.proposed.geojson'

# Krok vzorkování hran při kontrole známosti.  0,10 m je pod polovinou
# rozlišení rastru (0,05 m -> Nyquist 0,025 m by byl paranoidní; 0,10 m mine
# nejvýš jednu buňku v řadě a všechny reálné díry v mapě jsou o řád větší).
# Kontrola je navíc doplněná Bresenhamovým průchodem buňkami (`_cells_on`),
# takže vzorkovací krok neurčuje, jestli se díra najde — jen kde se ohlásí.
EDGE_STEP_M = 0.10

# Nejvýš tolik nálezů se vypisuje; zbytek se sečte.  Odmítnutí má být čitelné.
MAX_REPORTED_HITS = 12


class ProposalError(ValueError):
    """Návrh nelze přijmout — vždy s českým jednovětým důvodem."""


# ---- datum ----------------------------------------------------------------
def datum_path(site):
    return os.path.join(sitebundle.BUNDLE_ROOT, site, 'datum.yaml')


def load_datum(site):
    """Datum site bundlu, nebo `ProposalError` když chybí či je neúplné."""
    try:
        datum = sitebundle.load_datum(site)
    except sitebundle.SiteError as exc:
        raise ProposalError(str(exc))
    if not isinstance(datum, dict):
        raise ProposalError('site %s nemá datum.yaml — bez něj se mapové '
                            'souřadnice na UTM převést nedají' % site)
    return _zone_guard(datum, site)


def _zone_guard(datum, site):
    """Datum je použitelné jen když je úplné a v zóně 33 (EPSG:32633).

    Jiná zóna není detail: `geofence.py` má CRS napevno a prstenec z jiné zóny
    by ležel o stovky kilometrů jinde, aniž by na něm cokoli vypadalo divně.
    """
    for key in ('utm_e', 'utm_n'):
        if not isinstance(datum.get(key), (int, float)):
            raise ProposalError('datum site %s nemá číselné %s' % (site, key))
    zone = datum.get('utm_zone', 33)
    if int(zone) != 33:
        raise ProposalError('datum site %s je v UTM zóně %s; geofence je '
                            'definovaná v EPSG:32633 (zóna 33)' % (site, zone))
    return datum


def map_to_utm(x, y, datum):
    """(easting, northing) pro bod v rámci `map`.

    Jediné místo, kde tenhle vzorec je.  Ověřeno proti živému robotu — viz
    hlavička modulu a fixture v testu.
    """
    yaw = float(datum.get('yaw_rad') or 0.0)
    cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
    east = float(datum['utm_e']) + x * cos_yaw - y * sin_yaw
    north = float(datum['utm_n']) + x * sin_yaw + y * cos_yaw
    return east, north


def utm_to_map(east, north, datum):
    """Inverze `map_to_utm`; potřebná pro kontrolu UTM bodů proti gridu."""
    yaw = float(datum.get('yaw_rad') or 0.0)
    cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
    dx = float(east) - float(datum['utm_e'])
    dy = float(north) - float(datum['utm_n'])
    return dx * cos_yaw + dy * sin_yaw, -dx * sin_yaw + dy * cos_yaw


def ring_map_to_utm(points, datum):
    return [map_to_utm(float(x), float(y), datum) for x, y in points]


def datum_provenance(site, datum):
    """Původ datumu tak, jak putuje do zapsaného souboru.

    `captured` a `n_samples` jdou dovnitř schválně: plot postavený na datu ze
    734 vzorků a plot postavený na datu ze tří vzorků jsou dva různě důvěryhodné
    plotv a ze souboru to musí být poznat bez dohledávání.
    """
    path = datum_path(site)
    return {
        'file': path,
        'sha256': _file_sha256(path),
        'utm_e': datum.get('utm_e'),
        'utm_n': datum.get('utm_n'),
        'yaw_rad': datum.get('yaw_rad'),
        'utm_zone': datum.get('utm_zone', 33),
        'captured': datum.get('captured'),
        'n_samples': datum.get('n_samples'),
        'version': datum.get('version'),
    }


def _file_sha256(path):
    try:
        with open(path, 'rb') as stream:
            return hashlib.sha256(stream.read()).hexdigest()[:16]
    except (IOError, OSError):
        return None


# ---- occupancy grid -------------------------------------------------------
UNKNOWN = -1


class Grid(object):
    """Occupancy grid v rámci `map`, jen na jednu otázku: je tahle buňka známá?

    Hodnoty jsou konvence ROS `nav_msgs/OccupancyGrid`: -1 neznámo, 0..100
    pravděpodobnost obsazení.  Mimo grid je také **neznámo** — plot nesmí vést
    za okraj mapy o nic víc než přes díru uprostřed.
    """

    def __init__(self, cells, width, height, resolution, origin_x, origin_y,
                 name=None, source=None):
        if width <= 0 or height <= 0:
            raise ProposalError('occupancy grid má nulový rozměr')
        if len(cells) != width * height:
            raise ProposalError('occupancy grid: %d buněk, čekáno %d'
                                % (len(cells), width * height))
        self.cells = cells
        self.width = int(width)
        self.height = int(height)
        self.resolution = float(resolution)
        self.origin_x = float(origin_x)
        self.origin_y = float(origin_y)
        self.name = name
        self.source = source

    # -- geometrie --------------------------------------------------------
    def cell_of(self, x, y):
        """(col, row) buňky pod bodem, nebo None mimo grid. row 0 = nejnižší y."""
        col = int(math.floor((float(x) - self.origin_x) / self.resolution))
        row = int(math.floor((float(y) - self.origin_y) / self.resolution))
        if col < 0 or row < 0 or col >= self.width or row >= self.height:
            return None
        return col, row

    def value(self, x, y):
        cell = self.cell_of(x, y)
        if cell is None:
            return UNKNOWN
        col, row = cell
        return self.cells[row * self.width + col]

    def is_known(self, x, y):
        return self.value(x, y) != UNKNOWN

    def known_fraction(self):
        known = sum(1 for value in self.cells if value != UNKNOWN)
        return known / float(len(self.cells)) if self.cells else 0.0

    def bounds(self):
        return (self.origin_x, self.origin_y,
                self.origin_x + self.width * self.resolution,
                self.origin_y + self.height * self.resolution)

    # -- konstruktory -----------------------------------------------------
    @classmethod
    def from_occupancy_msg(cls, message, name=None):
        """Z `nav_msgs/OccupancyGrid` (dict z rosbridge / rosio.peek)."""
        info = (message or {}).get('info') or {}
        origin = (info.get('origin') or {}).get('position') or {}
        orientation = (info.get('origin') or {}).get('orientation') or {}
        # Rotovaný grid by znamenal jinou aritmetiku buněk; na tomhle robotu
        # je orientace vždy identita (ověřeno, viz agent/mapview.py), takže
        # radši odmítnout než tiše počítat špatně.
        if abs(float(orientation.get('z') or 0.0)) > 1e-6:
            raise ProposalError('occupancy grid je pootočený; kontrola '
                                'známosti počítá jen s osově zarovnaným gridem')
        return cls(list(message.get('data') or []),
                   int(info.get('width') or 0), int(info.get('height') or 0),
                   float(info.get('resolution') or 0.0),
                   float(origin.get('x') or 0.0), float(origin.get('y') or 0.0),
                   name=name, source='topic')

    @classmethod
    def from_map_yaml(cls, yaml_path):
        """Z dvojice map_server `*.yaml` + `*.pgm` (rastr v site bundlu)."""
        import yaml as yaml_module
        try:
            with open(yaml_path, 'r') as stream:
                meta = yaml_module.safe_load(stream) or {}
        except (IOError, OSError) as exc:
            raise ProposalError('rastr mapy nelze číst: %s' % exc)
        image = meta.get('image')
        if not image:
            raise ProposalError('%s nemá pole image' % yaml_path)
        image_path = image if os.path.isabs(image) else \
            os.path.join(os.path.dirname(yaml_path), image)
        resolution = float(meta.get('resolution') or 0.0)
        if resolution <= 0:
            raise ProposalError('%s nemá kladné resolution' % yaml_path)
        origin = meta.get('origin') or [0.0, 0.0, 0.0]
        occupied_thresh = float(meta.get('occupied_thresh', 0.65))
        free_thresh = float(meta.get('free_thresh', 0.196))
        negate = int(meta.get('negate', 0))
        pixels, width, height, maxval = _read_pgm(image_path)
        cells = _pixels_to_cells(pixels, width, height, maxval, negate,
                                 occupied_thresh, free_thresh)
        return cls(cells, width, height, resolution,
                   float(origin[0]), float(origin[1]),
                   name=os.path.splitext(os.path.basename(yaml_path))[0],
                   source=yaml_path)


def _read_pgm(path):
    """(pixels_top_down, width, height, maxval) pro binární PGM (P5)."""
    try:
        with open(path, 'rb') as stream:
            blob = stream.read()
    except (IOError, OSError) as exc:
        raise ProposalError('PGM mapy nelze číst: %s' % exc)
    if not blob.startswith(b'P5'):
        raise ProposalError('%s není binární PGM (P5)' % path)
    offset = 2
    fields = []
    while len(fields) < 3:
        while offset < len(blob) and blob[offset:offset + 1].isspace():
            offset += 1
        if blob[offset:offset + 1] == b'#':
            while offset < len(blob) and blob[offset:offset + 1] not in (b'\n', b'\r'):
                offset += 1
            continue
        start = offset
        while offset < len(blob) and not blob[offset:offset + 1].isspace():
            offset += 1
        token = blob[start:offset]
        if not token.isdigit():
            raise ProposalError('%s má poškozenou hlavičku PGM' % path)
        fields.append(int(token))
    offset += 1  # jeden bílý znak za maxval
    width, height, maxval = fields
    if maxval > 255:
        raise ProposalError('%s má 16bitové PGM; podporováno je 8bitové' % path)
    expected = width * height
    data = blob[offset:offset + expected]
    if len(data) != expected:
        raise ProposalError('%s: %d bajtů obrazu, čekáno %d'
                            % (path, len(data), expected))
    return data, width, height, maxval


def _pixels_to_cells(pixels, width, height, maxval, negate,
                     occupied_thresh, free_thresh):
    """map_server pravidlo: p -> obsazenost -> {volno, obsazeno, neznámo}.

    Řádek 0 obrázku je **horní** okraj = nejvyšší y, řádek 0 gridu je nejnižší
    y, takže se obrázek cestou dovnitř převrací.  Kdyby se to zapomnělo, plot
    by se zrcadlil kolem osy x a nic dál by si toho nevšimlo.
    """
    scale = float(maxval) if maxval else 255.0
    cells = [UNKNOWN] * (width * height)
    for image_row in range(height):
        grid_row = height - 1 - image_row
        base_in = image_row * width
        base_out = grid_row * width
        for col in range(width):
            pixel = pixels[base_in + col]
            occupancy = (pixel / scale) if negate else ((scale - pixel) / scale)
            if occupancy > occupied_thresh:
                cells[base_out + col] = 100
            elif occupancy < free_thresh:
                cells[base_out + col] = 0
            else:
                cells[base_out + col] = UNKNOWN
    return cells


_RASTER_STAMP = re.compile(r'(\d{8}_\d{6})')


def site_grid(site):
    """Nejnovější rastr site bundlu jako `Grid`; `ProposalError` když žádný není."""
    root = os.path.join(sitebundle.BUNDLE_ROOT, site, 'rasters')
    if not os.path.isdir(root):
        raise ProposalError('site %s nemá adresář rasters/ — bez mapy nelze '
                            'ověřit, že plot nevede přes neznámo' % site)
    candidates = []
    for dirpath, _dirnames, filenames in os.walk(root):
        for filename in filenames:
            if filename.endswith('.yaml'):
                candidates.append(os.path.join(dirpath, filename))
    if not candidates:
        raise ProposalError('site %s nemá v rasters/ žádný *.yaml' % site)

    def sort_key(path):
        match = _RASTER_STAMP.search(os.path.basename(path))
        return (match.group(1) if match else '', os.path.getmtime(path))

    return Grid.from_map_yaml(sorted(candidates, key=sort_key)[-1])


# ---- kontrola: vrchol i hrana v známém prostoru ---------------------------
def unknown_hits(points_map, grid, step_m=EDGE_STEP_M, closed=True):
    """Seznam míst, kde prstenec opouští známý prostor.  Prázdný = v pořádku.

    Prochází nejdřív vrcholy, pak hrany.  Hrany se řeší dvakrát a schválně:
    Bresenhamovým průchodem *všech* buněk pod úsečkou (nic nelze přeskočit)
    a zároveň vzorkováním po `step_m` (dá čitelnou souřadnici, kde to selhalo).
    Průchod buňkami je ta záruka; vzorkování je ta zpráva pro člověka.
    """
    hits = []
    count = len(points_map)
    bad_vertices = set()
    for index, (x, y) in enumerate(points_map):
        if not grid.is_known(x, y):
            bad_vertices.add(index)
            hits.append({'kind': 'vertex', 'index': index, 'segment': None,
                         'map': (round(x, 3), round(y, 3)),
                         'why': _why(grid, x, y)})
    limit = count if closed else count - 1
    for index in range(limit):
        start = points_map[index]
        end_index = (index + 1) % count
        end = points_map[end_index]
        hit = _segment_unknown(start, end, grid, step_m)
        if not hit:
            continue
        # Hrana, která selhala hned ve svém začátku, jenom opakuje už ohlášený
        # vrchol; hlásit obojí dělá z odmítnutí seznam, ve kterém se skutečná
        # díra uprostřed hrany ztratí.
        if hit.get('at_m') == 0.0 and index in bad_vertices:
            continue
        hit['kind'] = 'edge'
        hit['segment'] = index
        hit['to_index'] = end_index
        hit['from'] = (round(start[0], 3), round(start[1], 3))
        hit['to'] = (round(end[0], 3), round(end[1], 3))
        hits.append(hit)
    return hits


def _why(grid, x, y):
    return 'mimo mapu' if grid.cell_of(x, y) is None else 'nezmapovaná buňka'


def _segment_unknown(start, end, grid, step_m):
    """První neznámé místo na úsečce, nebo None."""
    bad_cell = _first_unknown_cell(start, end, grid)
    if bad_cell is None:
        return None
    # Vzorkováním najdi bod na úsečce, který v té buňce leží — kvůli hlášení.
    length = math.hypot(end[0] - start[0], end[1] - start[1])
    steps = max(1, int(math.ceil(length / max(step_m, 1e-6))))
    for index in range(steps + 1):
        t = index / float(steps)
        x = start[0] + t * (end[0] - start[0])
        y = start[1] + t * (end[1] - start[1])
        if not grid.is_known(x, y):
            return {'at_m': round(t * length, 2),
                    'map': (round(x, 3), round(y, 3)),
                    'why': _why(grid, x, y)}
    # Buňka je neznámá, ale žádný vzorek do ní nepadl (úsečka ji jen protne
    # rohem).  Ohlásíme střed té buňky — odmítnutí platí, jen souřadnice je
    # střed buňky místo bodu na úsečce.
    col, row = bad_cell
    return {'at_m': None,
            'map': (round(grid.origin_x + (col + 0.5) * grid.resolution, 3),
                    round(grid.origin_y + (row + 0.5) * grid.resolution, 3)),
            'why': 'nezmapovaná buňka'}


def _first_unknown_cell(start, end, grid):
    """První buňka pod úsečkou, která je neznámá (nebo mimo grid)."""
    for col, row in _cells_on(start, end, grid):
        if col < 0 or row < 0 or col >= grid.width or row >= grid.height:
            return (col, row)
        if grid.cells[row * grid.width + col] == UNKNOWN:
            return (col, row)
    return None


def _cells_on(start, end, grid):
    """Buňky protnuté úsečkou (amanatides-woo / DDA po mřížce)."""
    res = grid.resolution
    x0 = (start[0] - grid.origin_x) / res
    y0 = (start[1] - grid.origin_y) / res
    x1 = (end[0] - grid.origin_x) / res
    y1 = (end[1] - grid.origin_y) / res
    col, row = int(math.floor(x0)), int(math.floor(y0))
    end_col, end_row = int(math.floor(x1)), int(math.floor(y1))
    dx, dy = x1 - x0, y1 - y0
    step_x = 1 if dx > 0 else (-1 if dx < 0 else 0)
    step_y = 1 if dy > 0 else (-1 if dy < 0 else 0)
    t_max_x = ((col + (1 if step_x > 0 else 0)) - x0) / dx if step_x else float('inf')
    t_max_y = ((row + (1 if step_y > 0 else 0)) - y0) / dy if step_y else float('inf')
    t_delta_x = abs(1.0 / dx) if step_x else float('inf')
    t_delta_y = abs(1.0 / dy) if step_y else float('inf')
    cells = [(col, row)]
    # Strop: délka v buňkách plus rezerva; nekonečná smyčka je horší než hrubý
    # limit, a ten se v praxi nikdy nevyčerpá.
    guard = int(abs(dx) + abs(dy)) + 4
    while (col, row) != (end_col, end_row) and guard > 0:
        guard -= 1
        if t_max_x < t_max_y:
            col += step_x
            t_max_x += t_delta_x
        else:
            row += step_y
            t_max_y += t_delta_y
        cells.append((col, row))
    return cells


def describe_hits(hits, datum=None):
    """Odmítnutí v lidské řeči — který úsek, kde a proč."""
    lines = []
    for hit in hits[:MAX_REPORTED_HITS]:
        x, y = hit['map']
        where = 'map (%.2f, %.2f)' % (x, y)
        if datum:
            east, north = map_to_utm(x, y, datum)
            where += ' = UTM (%.2f, %.2f)' % (east, north)
        if hit['kind'] == 'vertex':
            lines.append('vrchol %d: %s — %s' % (hit['index'] + 1, where, hit['why']))
        else:
            at = ('%.2f m od začátku' % hit['at_m']) if hit.get('at_m') is not None \
                else 'na okraji buňky'
            lines.append('úsek %d->%d (%s): %s — %s'
                         % (hit['segment'] + 1, hit.get('to_index', hit['segment'] + 1) + 1,
                            at, where, hit['why']))
    if len(hits) > MAX_REPORTED_HITS:
        lines.append('... a dalších %d míst' % (len(hits) - MAX_REPORTED_HITS))
    return lines


# ---- návrh ----------------------------------------------------------------
def proposal_path(site):
    return os.path.join(sitebundle.BUNDLE_ROOT, site, PROPOSAL_NAME)


def active_path(site):
    return sitebundle.geofence_path(site)


def build_proposal(points_map, site, datum=None, grid=None, clicked_by=None,
                   note=None, step_m=EDGE_STEP_M, now=None):
    """Ověřený návrh plotu z naklikaných bodů v rámci `map`.

    Vrací `(fence, document)`.  Zvedne `ProposalError`/`GeofenceError`, když
    prstenec neprojde geometrií (`Fence`) nebo kontrolou známosti — a to
    **před** tím, než se cokoli zapíše.
    """
    now = time.time() if now is None else float(now)
    points_map = [(float(x), float(y)) for x, y in points_map]
    if len(points_map) >= 2 and _close_enough(points_map[0], points_map[-1]):
        points_map = points_map[:-1]
    if len(points_map) < 3:
        raise ProposalError('plot potřebuje aspoň 3 různé vrcholy, dostal %d'
                            % len(points_map))
    datum = datum or load_datum(site)
    grid = grid or site_grid(site)

    hits = unknown_hits(points_map, grid, step_m=step_m)
    if hits:
        raise ProposalError(
            'plot vede přes nezmapovaný prostor (%d míst) — neznámo není volno, '
            'návrh neukládám:\n  %s'
            % (len(hits), '\n  '.join(describe_hits(hits, datum))))

    ring_utm = ring_map_to_utm(points_map, datum)
    # Tady prochází návrh přesně tou validací, kterou pak použije supervisor:
    # minimální počet vrcholů, minimální plocha, sebeprotnutí.
    fence = Fence(ring_utm, site=site)

    samples = sum(max(1, int(math.ceil(
        math.hypot(points_map[(i + 1) % len(points_map)][0] - points_map[i][0],
                   points_map[(i + 1) % len(points_map)][1] - points_map[i][1])
        / max(step_m, 1e-6)))) + 1 for i in range(len(points_map)))
    provenance = {
        'method': SOURCE_CLICK,
        'tool': 'tools/geofence_propose',
        'created': time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(now)),
        'created_ts': round(now, 3),
        'clicked_by': clicked_by or None,
        'click_frame': 'map',
        'points_clicked': len(points_map),
        'ring_map': [[round(x, 3), round(y, 3)] for x, y in points_map],
        'datum': datum_provenance(site, datum),
        'known_check': {
            'grid': grid.name,
            'grid_source': grid.source,
            'resolution_m': grid.resolution,
            'step_m': step_m,
            'edge_samples': samples,
            'unknown_hits': 0,
            'grid_known_fraction': round(grid.known_fraction(), 4),
        },
    }
    document = dump_geofence(fence, site=site)
    document['note'] = (document['note'] +
                        ' Navrženo klikáním do mapy (source=%s) — do ostrého '
                        'provozu jen s podpisem člověka.' % SOURCE_CLICK)
    properties = document['features'][0]['properties']
    properties['source'] = SOURCE_CLICK
    properties['status'] = STATUS_PROPOSAL
    properties['signature'] = None
    properties['provenance'] = provenance
    if note:
        properties['note'] = str(note)[:400]
    return fence, document


def _close_enough(a, b, tolerance=1e-6):
    return abs(a[0] - b[0]) < tolerance and abs(a[1] - b[1]) < tolerance


def write_proposal(document, site, path=None):
    """Zapíše návrh vedle site bundlu.  **Nikdy** ne na `geofence.geojson`."""
    path = path or proposal_path(site)
    if os.path.basename(path) == sitebundle.GEOFENCE_NAME:
        raise ProposalError('návrh se nesmí zapsat jako aktivní %s — aktivní '
                            'plot vzniká jen podpisem' % sitebundle.GEOFENCE_NAME)
    _write_json(path, document)
    return path


def read_document(path):
    try:
        with open(path, 'r') as stream:
            return json.load(stream)
    except (IOError, OSError) as exc:
        raise ProposalError('soubor %s nelze číst: %s' % (path, exc))
    except ValueError as exc:
        raise ProposalError('soubor %s není platný JSON: %s' % (path, exc))


def document_properties(document):
    features = (document or {}).get('features') or []
    if not features:
        return {}
    return features[0].get('properties') or {}


def pedigree(document):
    """(source, status) dokumentu; chybějící `source` = projetý obvod."""
    properties = document_properties(document)
    return (properties.get('source') or SOURCE_DRIVEN,
            properties.get('status') or STATUS_ACTIVE)


def _write_json(path, document):
    directory = os.path.dirname(path)
    if directory and not os.path.isdir(directory):
        raise ProposalError('adresář %s neexistuje' % directory)
    tmp = path + '.tmp'
    with open(tmp, 'w') as stream:
        json.dump(document, stream, indent=2, ensure_ascii=False)
        stream.write('\n')
    os.replace(tmp, path)


def summarize(site):
    """Co pro tenhle site leží na disku — návrh, aktivní plot, nebo nic."""
    out = {'site': site, 'proposal': None, 'active': None}
    for key, path in (('proposal', proposal_path(site)),
                      ('active', active_path(site))):
        if not os.path.exists(path):
            continue
        entry = {'path': path}
        try:
            document = read_document(path)
            fence = load_geofence(path, site=site)
            properties = document_properties(document)
            entry.update({
                'digest': fence.digest,
                'summary': fence.summary(),
                'source': properties.get('source') or SOURCE_DRIVEN,
                'status': properties.get('status') or STATUS_ACTIVE,
                'signature': properties.get('signature'),
                'provenance': properties.get('provenance'),
            })
        except (ProposalError, GeofenceError) as exc:
            entry['error'] = str(exc)
        out[key] = entry
    return out
