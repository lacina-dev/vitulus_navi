"""Skládání mapy robota do jednoho PNG — robotí nástroj, bez agenta.

Proč to tady je
---------------
Čísla („x=1.2, y=-0.4, yaw=87°") neřeknou „stojím v úzké mezeře mezi dvěma
překážkami" ani „plán řeže roh živého plotu".  Prostorový vhled je v obrázku,
ne v JSONu, a vision model obrázek přečte.  Tenhle modul proto skládá svět
robota — occupancy grid, costmapy, lidar, plán, waypointy, zóny, geofence,
detekované objekty a robota se šipkou natočení — do jednoho PNG.

Historicky uměl totéž `vitulus_claude/agent/mapview.py`, tedy modul uvnitř
runtime agenta, a volalo ho jen web UI.  Podle pravidla „robotí vlastnost
patří do robotího balíčku, runtime agenta nesmí být závislostí robotí funkce"
je renderer tady, v `vitulus_navi`, a agent (i UI, i Hermes) ho volá zvenku
přes CLI `scripts/map_compose` nebo přes službu `/navi/compose_map`.

Domácí pravidla
---------------
* **Jen čtení.**  Každý zdroj je odběr topicu, TF lookup nebo soubor v site
  bundle.  Není tu jediný publisher ani service call, který by robotem hnul.
* **Nikdy neblokuje.**  Každé čtení ROSu má vlastní timeout a vrstvy se
  sbírají paralelně, takže celková doba je jeden timeout, ne devět.
* **Nikdy nevyhazuje.**  `compose()` vždy vrátí dict.  Chybějící ROS, chybějící
  data, chybějící Pillow -> `{ok: False, error: '<jednořádkový český důvod>'}`.
* **Vrstva, která se nepřečetla, se řekne v legendě.**  Model čte obrázek; kdyby
  vrstva zmizela potichu, přečte prázdné místo jako „nic tam není".  Pro stroj,
  který se podle toho rozhoduje, je „nevím" a „prázdno" opačný fakt.

Geometrie a orientace — jediná věc, která nesmí být potichu špatně
------------------------------------------------------------------
Kreslí se vždy v rámu **`map`, sever nahoru (+Y), východ vpravo (+X)** — tak,
jak to dělá web UI.  Řádek 0 occupancy gridu je *nejnižší* y, ale řádek 0
obrázku je *horní*, takže se každá mřížka na vstupu překlápí (`_grid_image`).
Orientace je i slovy v legendě: zrcadlená mapa by naučila agenta zrcadlený
svět a nic dál po proudu by to nechytilo.
"""
import json
import math
import os
import re
import threading
import time

# ---- zdroje (ověřeno živě na tomhle robotu) -------------------------------
FRAME = 'map'                 # všechno se kreslí v tomhle rámu
BASE_FRAME = 'base_link'

MAP_TOPIC = '/navi_manager/map'                            # nav_msgs/OccupancyGrid
COSTMAP_TOPIC = '/move_base_flex/global_costmap/costmap'   # nav_msgs/OccupancyGrid
LOCAL_COSTMAP_TOPIC = '/local_costmap_map_framed'          # už v rámu map
SCAN_TOPIC = '/scan'                                       # sensor_msgs/LaserScan
OBJECTS_TOPIC = '/navi_manager/map_objects'                # std_msgs/String (JSON)
ZONES_TOPIC = '/web_plan/zone_list'                        # vitulus_msgs/MapEditZoneList
NAVPVT_TOPIC = '/gnss/navpvt'                              # kvalita RTK do legendy
LIDAR_OBJECTS_TOPIC = '/safety/lidar_objects'              # std_msgs/String (JSON)

# Aktuální plán, v pořadí priority: kreslí se ten, který odpoví první.
GLOBAL_PLAN_TOPICS = ('/move_base_flex/GlobalPlanner/plan',
                      '/move_base_flex/TebLocalPlannerROS/global_plan',
                      '/navi_manager/map_path')
LOCAL_PLAN_TOPIC = '/move_base_flex/TebLocalPlannerROS/local_plan'

# ---- vrstvy ---------------------------------------------------------------
# Pořadí kreslení je významné: pozdější vrstvy leží na dřívějších.
LAYERS = ('grid', 'costmap', 'local_costmap', 'zones', 'geofence',
          'waypoints', 'plan', 'lidar', 'lidar_objects', 'robot')

# Rozumný přehled: jak site vypadá, kde robot je, co lidar vidí právě teď,
# kam smí a kam jede.  Costmapy jsou opt-in — nafouknuté překážky přikryjí
# základní mapu a obrázek se hůř, ne líp, čte, když si o ně nikdo neřekl.
DEFAULT_LAYERS = ('grid', 'robot', 'lidar', 'plan', 'waypoints',
                  'geofence', 'zones')

# Jména, kterými totéž pojmenovává web UI, agent a majitel.  Radši přijmout
# jeho slovník než odpovědět „neznámá vrstva" na jeho vlastní vokabulář.
ALIASES = {
    'map': 'grid', 'base': 'grid', 'saved': 'grid', 'mapa': 'grid',
    'occupancy': 'grid',
    'scan': 'lidar', 'laser': 'lidar',
    'path': 'plan', 'plan_global': 'plan', 'trasa': 'plan',
    'point': 'waypoints', 'points': 'waypoints', 'paths': 'waypoints',
    'body': 'waypoints',
    'costmap_global': 'costmap', 'costmap_local': 'local_costmap',
    'zone': 'zones', 'robot_marker': 'robot',
    'objects': 'lidar_objects', 'detections': 'lidar_objects',
    'prekazky': 'lidar_objects',
}
REFUSED = {
    'aerial': 'satelitní/fotomapa se do tohohle pohledu záměrně nekreslí',
    'rain': 'dešťový radar není mapová vrstva v rámu map',
    'terrain': 'terénní DEM (Terrain) zatím neumím georeferencovat',
    'direct': 'živý direct raster se v rámu map nevysílá',
    'preview': 'náhledový raster se v rámu map nevysílá',
}

DEFAULT_KEEP = 30
DEFAULT_WIDTH = 1024            # šířka MAPOVÉ části; legenda se přidá pod ni
DEFAULT_TIMEOUT_S = 3.0
DEFAULT_RADIUS_M = 15.0         # poloměr výřezu kolem robota (center=robot)
MIN_SPAN_M = 6.0                # blíž se nezoomuje, lhalo by to o detailu
PAD_M = 1.5                     # vzduch kolem kresleného obsahu

_NAME_SUFFIX = '.png'

# ---- barvy ----------------------------------------------------------------
BG = (24, 26, 31)
PANEL = (17, 18, 22)
INK = (232, 234, 238)
DIM = (150, 156, 166)
LINE = (58, 62, 72)

C_FREE = (170, 205, 150, 255)
C_OCC = (230, 40, 40, 255)
C_OCC_PROB = (230, 40, 40, 150)
C_UNKNOWN = (56, 60, 70, 255)

C_COST_LETHAL = (255, 120, 0, 115)
C_COST_INSCRIBED = (255, 170, 40, 70)
C_COST_SOFT = (255, 215, 90, 32)

C_LOCAL_LETHAL = (255, 60, 235, 115)
C_LOCAL_SOFT = (255, 60, 235, 30)

C_ZONE = (170, 110, 255)
C_GEOFENCE = (0, 225, 255)
C_WAYPOINT = (255, 255, 255)
C_SITE_PATH = (80, 150, 255)
C_PLAN = (120, 255, 120)
C_LOCAL_PLAN = (255, 160, 40)
C_SCAN = (255, 235, 90)
C_OBJECT = (255, 70, 70)
C_ROBOT = (255, 40, 40)

FONT_PATHS = ('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',
              '/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf')

_tf = {'buffer': None, 'listener': None}
_node = {'inited': False}


# ---------------------------------------------------------------------------
# instalatérství: minimalistické čtení ROSu, vlastní, bez agenta
# ---------------------------------------------------------------------------
def ensure_node(name='vitulus_map_compose', anonymous=True):
    """Inicializuj uzel jednou za proces.  Když už uzel běží, nedělej nic."""
    if _node['inited']:
        return True
    try:
        import rospy
        if not rospy.core.is_initialized():
            rospy.init_node(name, anonymous=anonymous, disable_signals=True)
        _node['inited'] = True
        return True
    except Exception:                                   # noqa: BLE001
        return False


def _peek(topic, timeout):
    """Jedna čerstvá zpráva z topicu, nebo None.  Nikdy nevyhazuje.

    Typ se zjišťuje z masteru, ne z hint tabulky: seznam topiců se mění
    rychleji než jakákoli tabulka v kódu.
    """
    if not ensure_node():
        return None
    try:
        import rospy
        import rostopic
        import roslib.message
    except Exception:                                   # noqa: BLE001
        return None
    try:
        typ, real, _fn = rostopic.get_topic_type(topic, blocking=False)
    except Exception:                                   # noqa: BLE001
        return None
    if not typ:
        return None
    cls = roslib.message.get_message_class(typ)
    if cls is None:
        return None
    try:
        return rospy.wait_for_message(real or topic, cls, timeout=timeout)
    except Exception:                                   # noqa: BLE001
        return None


def _pil():
    """Pillow, nebo None.  Import líný, ať se modul načte i bez ní."""
    try:
        from PIL import Image, ImageDraw, ImageFont
        return Image, ImageDraw, ImageFont
    except Exception:                                   # noqa: BLE001
        return None


def output_dir():
    """Kam padají obrázky, když volající neřekne `--out`."""
    override = os.environ.get('VITULUS_MAP_COMPOSE_DIR')
    if override:
        return os.path.expanduser(override)
    return os.path.expanduser(os.path.join('~', '.vitulus', 'map_compose'))


def _filename(now=None):
    """Jméno s UTC razítkem na milisekundy, ať se dva rendery za sebou nesrazí."""
    now = time.time() if now is None else now
    base = time.strftime('%Y%m%dT%H%M%S', time.gmtime(now))
    millis = int((now - int(now)) * 1000)
    return '%s%03dZ%s' % (base, millis, _NAME_SUFFIX)


#: Tvar, který dělá `_filename()`.  Úklid maže JEN vlastní jména: kvóta
#: spočítaná přes cizí soubory není kvóta a mazat cizí soubory není práce
#: tohohle modulu.
_OURS = re.compile(r'^[0-9T]+Z' + re.escape(_NAME_SUFFIX) + r'$')


def _prune(dir_path, keep):
    if keep is None or keep <= 0:
        return
    try:
        names = sorted(f for f in os.listdir(dir_path) if _OURS.match(f))
    except OSError:
        return
    for name in names[:-keep]:
        try:
            os.remove(os.path.join(dir_path, name))
        except OSError:
            pass


def _yaw(quaternion):
    """Yaw v radiánech z geometry_msgs/Quaternion."""
    x = float(getattr(quaternion, 'x', 0.0) or 0.0)
    y = float(getattr(quaternion, 'y', 0.0) or 0.0)
    z = float(getattr(quaternion, 'z', 0.0) or 0.0)
    w = float(getattr(quaternion, 'w', 1.0) or 0.0)
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _prime_tf(timeout_s):
    """Postav TF buffer/listener jednou, na VOLAJÍCÍM vlákně.

    `rospy.init_node` je prostý check-then-set a `TransformListener` si v
    konstruktoru staví odběry — ani jedno nepřežije několik `_gather` vláken
    naráz a selhání je tiché (robot a lidar prostě z obrázku zmizí).  Čerstvý
    listener má navíc prázdný buffer, takže dostane chvíli na naplnění.
    """
    if _tf['buffer'] is not None:
        return
    try:
        import tf2_ros
    except Exception:                                   # noqa: BLE001
        return
    if not ensure_node():
        return
    try:
        buffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(buffer)
    except Exception:                                   # noqa: BLE001
        return
    _tf['buffer'] = buffer
    _tf['listener'] = listener
    time.sleep(min(1.5, max(0.2, float(timeout_s))))


#: Česká 16dílná růžice, po směru hodin od severu.  Slovo vedle čísel proto,
#: že slovo nejde přečíst ve špatné konvenci.
_COMPASS = ('S', 'SSV', 'SV', 'VSV', 'V', 'VJV', 'JV', 'JJV',
            'J', 'JJZ', 'JZ', 'ZJZ', 'Z', 'ZSZ', 'SZ', 'SSZ')


def heading(yaw_rad):
    """(azimut_deg, yaw_deg, světová strana) pro yaw v rámu `map`.

    Potkávají se tu dvě konvence a splést je je horší než špatný obrázek,
    protože popisek je to, co čtenář cituje: ROS yaw se měří proti směru
    hodin od +X (východ), zatímco azimut na mapě se sever nahoru čte po
    směru hodin od severu.  Liší se zrcadlením, ne posunem:
    `azimut = (90 - yaw) mod 360`.
    """
    yaw_deg = math.degrees(float(yaw_rad))
    signed = ((yaw_deg + 180.0) % 360.0) - 180.0        # (-180, 180]
    azimuth = (90.0 - yaw_deg) % 360.0
    point = _COMPASS[int(((azimuth + 11.25) % 360.0) / 22.5)]
    return azimuth, signed, point


def _tf_lookup(target, source, timeout_s=1.0):
    """(x, y, yaw, stáří_s) pro `target` <- `source`, nebo None.  Nevyhazuje."""
    try:
        import rospy
    except Exception:                                   # noqa: BLE001
        return None
    if _tf['buffer'] is None:
        _prime_tf(timeout_s)
    if _tf['buffer'] is None:
        return None
    try:
        transform = _tf['buffer'].lookup_transform(
            target, source, rospy.Time(0),
            rospy.Duration(max(0.05, float(timeout_s))))
    except Exception:                                   # noqa: BLE001
        return None
    try:
        translation = transform.transform.translation
        stamp = transform.header.stamp
        age = (rospy.Time.now() - stamp).to_sec() if stamp != rospy.Time() else 0.0
        return (float(translation.x), float(translation.y),
                _yaw(transform.transform.rotation), float(age))
    except Exception:                                   # noqa: BLE001
        return None


def _gather(jobs, timeout_s):
    """Každý sběrač ve vlastním vlákně; nedodaná odpověď zůstane None.

    Sondy jsou nezávislá omezená čtení, takže je nic nestojí překrýt — a ze
    součtu timeoutů se stane jeden timeout.  Démon vlákna + deadline znamená,
    že sonda, která svůj vlastní limit ignoruje, nemůže zaseknout volajícího.
    """
    results = {}
    threads = []

    def worker(key, job):
        try:
            results[key] = job()
        except Exception as exc:                        # noqa: BLE001
            results[key] = (None, '%s: %s' % (type(exc).__name__, exc))

    for key, job in jobs.items():
        thread = threading.Thread(target=worker, args=(key, job),
                                  name='mapcompose-%s' % key)
        thread.daemon = True
        thread.start()
        threads.append(thread)

    deadline = time.time() + max(0.5, float(timeout_s)) + 3.0
    for thread in threads:
        thread.join(timeout=max(0.0, deadline - time.time()))
    return results


# ---------------------------------------------------------------------------
# site bundle (waypointy, trasy, geofence) a datum UTM <-> map
# ---------------------------------------------------------------------------
def _sitebundle():
    """Modul `geofence.sitebundle` tohohle balíčku, nebo None.

    Zkusí se normální import; když balíček `vitulus_navi` není nainstalovaný
    (setup.py ho zatím nevyjmenovává), načte se sourozenec podle cesty k
    tomuhle souboru.  Renderer nesmí padnout jen kvůli tomu, jak je zabalený.
    """
    return _sibling('geofence.sitebundle', ('geofence', 'sitebundle.py'))


def _fencemod():
    return _sibling('geofence.fence', ('geofence', 'fence.py'))


_SIBLINGS = {}


def _sibling(dotted, relpath):
    if dotted in _SIBLINGS:
        return _SIBLINGS[dotted]
    module = None
    try:
        import importlib
        module = importlib.import_module('vitulus_navi.' + dotted)
    except Exception:                                   # noqa: BLE001
        try:
            import importlib.util as _util
            path = os.path.join(os.path.dirname(os.path.abspath(__file__)), *relpath)
            spec = _util.spec_from_file_location(
                'vitulus_navi_local_' + dotted.replace('.', '_'), path)
            module = _util.module_from_spec(spec)
            spec.loader.exec_module(module)
        except Exception:                               # noqa: BLE001
            module = None
    _SIBLINGS[dotted] = module
    return module


def active_site():
    """Site bundle, který robot právě obsluhuje, nebo None.

    Pořadí: co si mapping naposled odbavil (`.last_served`), jinak první
    bundle na disku.  Robot bez jediného bundle je legitimní stav (nic ještě
    nebylo zmapované), ne chyba.
    """
    bundle = _sitebundle()
    if bundle is None:
        return None
    try:
        with open(os.path.join(bundle.BUNDLE_ROOT, '.last_served'), 'r') as stream:
            data = json.load(stream)
        if isinstance(data, dict) and data.get('site'):
            return str(data['site'])
    except (IOError, OSError, ValueError):
        pass
    try:
        sites = bundle.list_sites()
    except Exception:                                   # noqa: BLE001
        return None
    return sites[0] if sites else None


def _datum(site):
    """(utm_e, utm_n, yaw_rad) pro site, nebo None.

    Jediný most mezi dvěma soustavami: bundle drží geofence a waypointy v
    UTM metrech, všechno na drátě (a všechno kreslené) je v rámu `map`.
    """
    if not site:
        return None
    bundle = _sitebundle()
    if bundle is None:
        return None
    try:
        data = bundle.load_datum(site)
    except Exception:                                   # noqa: BLE001
        return None
    if not isinstance(data, dict):
        return None
    try:
        return (float(data['utm_e']), float(data['utm_n']),
                float(data.get('yaw_rad') or 0.0))
    except (KeyError, TypeError, ValueError):
        return None


def utm_to_map(east, north, datum):
    """UTM metry -> metry v rámu `map`; inverze data bundlu."""
    e0, n0, yaw = datum
    dx, dy = float(east) - e0, float(north) - n0
    cos, sin = math.cos(yaw), math.sin(yaw)
    return (dx * cos + dy * sin, -dx * sin + dy * cos)


#: „Geofence prostě ještě není."  Normální stav, dokud majitel neobjede
#: perimetr — tedy znalost, ne porucha.  Legenda to musí říct nahlas, protože
#: nenakreslená hranice se nikdy nesmí číst jako „žádná hranice neplatí".
NOT_RECORDED = object()


def _load_geofence_ring(site):
    """(prstenec_v_UTM, None) | (NOT_RECORDED, proč) | (None, '<český důvod>')."""
    if not site:
        return None, 'není aktivní site bundle'
    bundle, fence_mod = _sitebundle(), _fencemod()
    if bundle is None or fence_mod is None:
        return None, 'geofence modul nejde načíst'
    path = bundle.geofence_path(site)
    if not os.path.isfile(path):
        return NOT_RECORDED, ('geofence pro site %s ještě neexistuje — nikdo '
                              'zatím neobjel perimetr' % site)
    try:
        fence = fence_mod.load_geofence(path, site=site)
    except Exception as exc:                            # noqa: BLE001
        return None, 'geofence je vadná: %s' % exc
    return list(fence.ring), None


# ---------------------------------------------------------------------------
# sběrače: (payload, chyba) na vrstvu, nikdy nevyhazují
# ---------------------------------------------------------------------------
def _empty(why):
    """Vrstva, která odpověděla, a poctivá odpověď zní „nic tam není".

    Drží se odděleně od vrstvy, která se nepřečetla vůbec: pro robota jsou to
    opačné fakty.  „Žádné zóny nejsou uložené" je znalost, „zóny se nepodařilo
    přečíst" je její absence.
    """
    return {'empty': str(why)}


def _drawable(entry):
    payload = (entry or (None, None))[0]
    if not isinstance(payload, dict) or payload.get('empty'):
        return None
    return payload


def _collect_grid(topic, timeout):
    msg = _peek(topic, timeout)
    if msg is None:
        return None, 'topic %s mlčí' % topic
    info = getattr(msg, 'info', None)
    data = getattr(msg, 'data', None)
    if info is None or data is None:
        return None, 'zpráva z %s nemá očekávaný tvar' % topic
    try:
        origin = info.origin.position
        grid = {'w': int(info.width), 'h': int(info.height),
                'res': float(info.resolution),
                'ox': float(origin.x), 'oy': float(origin.y),
                'oyaw': _yaw(info.origin.orientation),
                'data': data, 'topic': topic,
                'frame': getattr(getattr(msg, 'header', None), 'frame_id', '') or ''}
    except Exception as exc:                            # noqa: BLE001
        return None, 'mřížku z %s nejde přečíst: %s' % (topic, exc)
    if grid['w'] <= 0 or grid['h'] <= 0 or grid['res'] <= 0:
        return None, 'mřížka z %s je prázdná' % topic
    if grid['frame'] and grid['frame'] != FRAME:
        return None, ('mřížka z %s je v rámu %s, ne %s'
                      % (topic, grid['frame'], FRAME))
    if abs(grid['oyaw']) > 1e-3:
        # Radši nepodporováno než potichu špatně: osově zarovnané vložení
        # pootočené mřížky je přesně ta třída chyb, kvůli které tenhle modul
        # opakuje orientaci i slovy.
        return None, ('mřížka z %s je pootočená o %.2f°, to zatím neumím vykreslit'
                      % (topic, math.degrees(grid['oyaw'])))
    return grid, None


def _collect_scan(timeout):
    msg = _peek(SCAN_TOPIC, timeout)
    if msg is None:
        return None, 'topic %s mlčí (lidar neběží?)' % SCAN_TOPIC
    frame = getattr(getattr(msg, 'header', None), 'frame_id', '') or 'base_scan'
    pose = _tf_lookup(FRAME, frame, timeout)
    if pose is None:
        return None, 'transformace %s <- %s není dostupná' % (FRAME, frame)
    px, py, pyaw, _age = pose
    try:
        ranges = list(msg.ranges)
        amin = float(msg.angle_min)
        ainc = float(msg.angle_increment)
        rmin = float(getattr(msg, 'range_min', 0.0) or 0.0)
        rmax = float(getattr(msg, 'range_max', 0.0) or 0.0)
    except Exception as exc:                            # noqa: BLE001
        return None, 'scan z %s nejde přečíst: %s' % (SCAN_TOPIC, exc)
    points = []
    cos, sin = math.cos(pyaw), math.sin(pyaw)
    for index, distance in enumerate(ranges):
        try:
            distance = float(distance)
        except (TypeError, ValueError):
            continue
        if distance != distance or distance <= rmin or (rmax and distance >= rmax):
            continue                                    # NaN / inf / mimo pásmo
        angle = amin + index * ainc
        lx, ly = distance * math.cos(angle), distance * math.sin(angle)
        points.append((px + lx * cos - ly * sin, py + lx * sin + ly * cos))
    if not points:
        return None, 'lidar posílá scan, ale bez platných paprsků'
    return {'points': points, 'frame': frame, 'total': len(ranges)}, None


def _collect_robot(timeout):
    """Poloha robota v rámu `map` plus jak moc se jí dá věřit.

    Dva zdroje v pořadí, a odpověď říká, který to byl — rozdíl je podstatný:
    TF `map <- base_link` je to, z čeho kreslí robota web UI, ale na tomhle
    stroji ho živí odometrie zaseknutá u doku, ne GNSS.  Může tedy být zcela
    v pořádku *relativně*, zatímco absolutní fix chybí úplně.
    """
    pose = _tf_lookup(FRAME, BASE_FRAME, timeout)
    source = 'TF %s ← %s' % (FRAME, BASE_FRAME)
    if pose is None:
        odom = _peek('/odometry/odom', timeout)
        link = _tf_lookup(FRAME, 'odom', timeout)
        if odom is None or link is None:
            return None, ('poloha nedostupná: %s ani /odometry/odom neodpovídá'
                          % source)
        try:
            position = odom.pose.pose.position
            yaw = _yaw(odom.pose.pose.orientation)
        except Exception as exc:                        # noqa: BLE001
            return None, 'odometrii nejde přečíst: %s' % exc
        ox, oy, oyaw, _ = link
        cos, sin = math.cos(oyaw), math.sin(oyaw)
        pose = (ox + position.x * cos - position.y * sin,
                oy + position.x * sin + position.y * cos,
                oyaw + yaw, 0.0)
        source = 'odometrie /odometry/odom přes TF %s ← odom' % FRAME
    x, y, yaw, age = pose
    return {'x': x, 'y': y, 'yaw': yaw, 'age_s': age, 'source': source,
            'rtk': _rtk_quality(timeout)}, None


def _rtk_quality(timeout):
    """'fixed' / 'float' / 'none' / None — vždycky jen popisek, nikdy poloha."""
    msg = _peek(NAVPVT_TOPIC, timeout)
    flags = getattr(msg, 'flags', None) if msg is not None else None
    if flags is None:
        return None
    try:
        carr = int(flags) & 0xC0
    except (TypeError, ValueError):
        return None
    return 'fixed' if carr == 0x80 else 'float' if carr == 0x40 else 'none'


def _path_points(msg):
    """[(x, y), ...] z nav_msgs/Path, nebo []."""
    out = []
    for stamped in getattr(msg, 'poses', None) or []:
        try:
            position = stamped.pose.position
            out.append((float(position.x), float(position.y)))
        except Exception:                               # noqa: BLE001
            continue
    return out


def _collect_plan(timeout):
    """Plán, který robot právě sleduje — globální i lokální.

    Zkoušejí se čtyři topicy, takže každý dostane *díl* rozpočtu vrstvy, ne
    celý: existující plán se publikuje několikrát za sekundu, kdežto stojící
    robot mlčí na všech — a čtyři plné timeouty za sebou by rozbily deadline
    celého renderu kvůli vrstvě, jejíž poctivá odpověď zní „nikam nejedu".
    """
    slot = max(0.4, min(float(timeout), 0.9))
    result = {'global': [], 'local': [], 'topic': None}
    for topic in GLOBAL_PLAN_TOPICS:
        msg = _peek(topic, slot)
        points = _path_points(msg) if msg is not None else []
        if points:
            result['global'] = points
            result['topic'] = topic
            break
    local = _peek(LOCAL_PLAN_TOPIC, slot)
    result['local'] = _path_points(local) if local is not None else []
    if not result['global'] and not result['local']:
        return _empty('robot právě nikam nejede — žádný plán se nevysílá'), None
    return result, None


def _collect_waypoints(timeout, site, datum):
    """Uložené waypointy a trasy site bundlu, v rámu `map`.

    `/navi_manager/map_objects` má přednost: je už v rámu `map` *a* je to
    přesně to, co vypisuje web UI, takže jména, která přečte model, jsou
    jména, která vidí majitel.  Bundle je záloha pro případ, že navi_manager
    neběží; je v UTM, proto to datum.
    """
    msg = _peek(OBJECTS_TOPIC, timeout)
    raw = getattr(msg, 'data', None) if msg is not None else None
    if raw:
        try:
            payload = json.loads(raw)
            points = [(str(w.get('name') or '?'), float(w['x']), float(w['y']))
                      for w in payload.get('waypoints') or [] if 'x' in w and 'y' in w]
            paths = [(str(p.get('name') or '?'),
                      [(float(a), float(b)) for a, b in (p.get('points') or [])])
                     for p in payload.get('paths') or []]
            if points or paths:
                return {'points': points, 'paths': paths,
                        'source': OBJECTS_TOPIC}, None
        except Exception:                               # noqa: BLE001
            pass
    if not site:
        return None, 'topic %s mlčí a není aktivní site bundle' % OBJECTS_TOPIC
    if datum is None:
        return None, ('topic %s mlčí a site %s nemá datum.yaml, takže UTM '
                      'body nejde přepočítat do rámu map' % (OBJECTS_TOPIC, site))
    bundle = _sitebundle()
    if bundle is None:
        return None, 'site bundle modul nejde načíst'
    try:
        points = [(name, ) + utm_to_map(e, n, datum)
                  for name, (e, n) in sorted(bundle.load_waypoints(site).items())]
        paths = [(name, [utm_to_map(e, n, datum) for e, n in line])
                 for name, line in sorted(bundle.load_paths(site).items())]
    except Exception as exc:                            # noqa: BLE001
        return None, 'site bundle %s nejde přečíst: %s' % (site, exc)
    if not points and not paths:
        return None, 'site bundle %s nemá uložené body ani trasy' % site
    return {'points': points, 'paths': paths,
            'source': 'site bundle %s (přes datum)' % site}, None


def _collect_zones(timeout):
    msg = _peek(ZONES_TOPIC, timeout)
    if msg is None:
        return None, 'topic %s mlčí' % ZONES_TOPIC
    zones = []
    for zone in getattr(msg, 'zone_list', None) or []:
        try:
            ring = [(float(p.x), float(p.y)) for p in zone.polygon.polygon.points]
        except Exception:                               # noqa: BLE001
            continue
        if len(ring) >= 3:
            zones.append((str(getattr(zone, 'name', '') or '?'), ring))
    if not zones:
        return _empty('žádné zóny nejsou uložené (seznam je prázdný)'), None
    return {'zones': zones}, None


def _collect_geofence(site, datum):
    ring, error = _load_geofence_ring(site)
    if ring is NOT_RECORDED:
        return _empty(error), None
    if ring is None:
        return None, error
    if datum is None:
        return None, ('geofence existuje, ale site %s nemá datum.yaml — UTM '
                      'hranici nejde přepočítat do rámu map' % site)
    return {'ring': [utm_to_map(e, n, datum) for e, n in ring],
            'site': site}, None


def _obj_geometry(item, timeout):
    """(prstenec_v_map, střed_v_map) pro jeden detekovaný objekt, nebo (None, None).

    Tvar zprávy z `/safety/lidar_objects` vzniká paralelně, takže se tu čte
    tolerantně: přijme se polygon (`points`/`polygon`/`hull`), střed
    (`x`/`y` nebo `cx`/`cy`) s poloměrem (`r`/`radius`) nebo s rozměry
    (`w`/`h`).  Cizí rám (`frame`/`frame_id`) se dotáhne přes TF.
    """
    ring = None
    for key in ('points', 'polygon', 'hull', 'contour'):
        raw = item.get(key)
        if isinstance(raw, (list, tuple)) and len(raw) >= 3:
            try:
                ring = [(float(p[0]), float(p[1])) for p in raw]
            except (TypeError, ValueError, IndexError):
                ring = None
            if ring:
                break
    cx = item.get('x', item.get('cx'))
    cy = item.get('y', item.get('cy'))
    centre = None
    try:
        if cx is not None and cy is not None:
            centre = (float(cx), float(cy))
    except (TypeError, ValueError):
        centre = None
    if ring is None and centre is not None:
        radius = item.get('r', item.get('radius'))
        try:
            radius = float(radius) if radius is not None else None
        except (TypeError, ValueError):
            radius = None
        if radius is None:
            try:
                radius = 0.5 * max(float(item.get('w') or 0.4),
                                   float(item.get('h') or 0.4))
            except (TypeError, ValueError):
                radius = 0.25
        radius = max(0.1, min(5.0, radius))
        ring = [(centre[0] + radius * math.cos(a), centre[1] + radius * math.sin(a))
                for a in [i * math.pi / 8.0 for i in range(16)]]
    if ring is None:
        return None, None
    if centre is None:
        centre = (sum(p[0] for p in ring) / len(ring),
                  sum(p[1] for p in ring) / len(ring))
    frame = str(item.get('frame') or item.get('frame_id') or FRAME)
    if frame and frame != FRAME:
        pose = _tf_lookup(FRAME, frame, timeout)
        if pose is None:
            return None, None
        px, py, pyaw, _age = pose
        cos, sin = math.cos(pyaw), math.sin(pyaw)
        ring = [(px + x * cos - y * sin, py + x * sin + y * cos) for x, y in ring]
        centre = (px + centre[0] * cos - centre[1] * sin,
                  py + centre[0] * sin + centre[1] * cos)
    return ring, centre


def _collect_lidar_objects(timeout):
    """Objekty z `/safety/lidar_objects` (std_msgs/String s JSONem).

    Topic vzniká paralelně v `vitulus_safety`.  Když ještě není, je to
    *chybějící* vrstva, ne prázdná — v legendě se to řekne rozdílně.
    """
    msg = _peek(LIDAR_OBJECTS_TOPIC, timeout)
    raw = getattr(msg, 'data', None) if msg is not None else None
    if not raw:
        return None, ('topic %s mlčí (detekce objektů neběží)'
                      % LIDAR_OBJECTS_TOPIC)
    try:
        payload = json.loads(raw)
    except Exception as exc:                            # noqa: BLE001
        return None, '%s nevysílá platný JSON: %s' % (LIDAR_OBJECTS_TOPIC, exc)
    items = payload
    if isinstance(payload, dict):
        for key in ('objects', 'items', 'detections', 'obstacles'):
            if isinstance(payload.get(key), list):
                items = payload[key]
                break
    if not isinstance(items, list):
        return None, '%s má neznámý tvar JSONu' % LIDAR_OBJECTS_TOPIC
    objects = []
    for item in items:
        if not isinstance(item, dict):
            continue
        ring, centre = _obj_geometry(item, timeout)
        if ring is None:
            continue
        label = str(item.get('label') or item.get('class') or item.get('name')
                    or item.get('type') or 'objekt')
        objects.append({'ring': ring, 'centre': centre, 'label': label})
    if not objects:
        return _empty('detekce běží, ale právě teď nevidí žádný objekt'), None
    return {'objects': objects}, None


# ---------------------------------------------------------------------------
# projekce: metry v rámu `map` -> pixely, sever nahoru, východ vpravo
# ---------------------------------------------------------------------------
class _Proj(object):
    """+x (východ) doprava, +y (sever) NAHORU — proto se člen y odečítá:
    řádky obrázku rostou dolů, řádky světa ne."""

    def __init__(self, west, south, east, north, area):
        x0, y0, width, height = area
        span_x = max(1e-6, east - west)
        span_y = max(1e-6, north - south)
        self.scale = max(span_x / width, span_y / height)   # metrů na pixel
        self.cx = 0.5 * (west + east)
        self.cy = 0.5 * (south + north)
        self.x0, self.y0, self.width, self.height = x0, y0, width, height
        half_x = 0.5 * width * self.scale
        half_y = 0.5 * height * self.scale
        self.bbox = (self.cx - half_x, self.cy - half_y,
                     self.cx + half_x, self.cy + half_y)

    def px(self, x, y):
        return (self.x0 + 0.5 * self.width + (x - self.cx) / self.scale,
                self.y0 + 0.5 * self.height - (y - self.cy) / self.scale)

    def length(self, metres):
        return metres / self.scale


def _content_bounds(data):
    """(west, south, east, north) přes všechno, co se bude kreslit."""
    xs, ys = [], []

    def add(x, y):
        xs.append(float(x))
        ys.append(float(y))

    for key in ('grid', 'costmap', 'local_costmap'):
        grid = _drawable(data.get(key))
        if grid:
            add(grid['ox'], grid['oy'])
            add(grid['ox'] + grid['w'] * grid['res'],
                grid['oy'] + grid['h'] * grid['res'])
    fence = _drawable(data.get('geofence'))
    if fence:
        for x, y in fence['ring']:
            add(x, y)
    waypoints = _drawable(data.get('waypoints'))
    if waypoints:
        for _name, x, y in waypoints['points']:
            add(x, y)
        for _name, line in waypoints['paths']:
            for x, y in line:
                add(x, y)
    zones = _drawable(data.get('zones'))
    if zones:
        for _name, ring in zones['zones']:
            for x, y in ring:
                add(x, y)
    plan = _drawable(data.get('plan'))
    if plan:
        for x, y in plan['global'] + plan['local']:
            add(x, y)
    robot = _drawable(data.get('robot'))
    if robot:
        add(robot['x'] - 3.0, robot['y'] - 3.0)
        add(robot['x'] + 3.0, robot['y'] + 3.0)
    if not xs:
        # Teprve když není nic jiného: pohled jen z lidaru má rámovat odrazy,
        # ale nikdy nesmí strhnout jinak dobře zarámovanou mapu.
        for key in ('lidar', 'lidar_objects'):
            payload = _drawable(data.get(key))
            if not payload:
                continue
            for x, y in payload.get('points') or []:
                add(x, y)
            for obj in payload.get('objects') or []:
                for x, y in obj['ring']:
                    add(x, y)
    if not xs:
        return None
    west, east = min(xs) - PAD_M, max(xs) + PAD_M
    south, north = min(ys) - PAD_M, max(ys) + PAD_M
    if east - west < MIN_SPAN_M:
        mid = 0.5 * (west + east)
        west, east = mid - 0.5 * MIN_SPAN_M, mid + 0.5 * MIN_SPAN_M
    if north - south < MIN_SPAN_M:
        mid = 0.5 * (south + north)
        south, north = mid - 0.5 * MIN_SPAN_M, mid + 0.5 * MIN_SPAN_M
    return west, south, east, north


# ---------------------------------------------------------------------------
# kreslení
# ---------------------------------------------------------------------------
def _font(ImageFont, size, bold=False):
    for path in (FONT_PATHS[1], FONT_PATHS[0]) if bold else FONT_PATHS:
        try:
            return ImageFont.truetype(path, size)
        except Exception:                               # noqa: BLE001
            continue
    try:
        return ImageFont.load_default()
    except Exception:                                   # noqa: BLE001
        return None


def _grid_image(Image, grid, lut):
    """RGBA obrázek jedné mřížky, už překlopený do pořadí řádků obrázku.

    `lut` mapuje syrový bajt (hodnota & 0xFF, takže -1 se stane 255) na čtyři
    bajty RGBA.  Jeden join místo numpy — pro ~170k buněk téhle site to stojí
    desítky milisekund a nepřidává závislost.
    """
    blank = b'\x00\x00\x00\x00'
    table = [blank] * 256
    for value, colour in lut.items():
        table[value & 0xFF] = bytes(colour)
    try:
        buffer = b''.join([table[v & 0xFF] for v in grid['data']])
    except Exception:                                   # noqa: BLE001
        return None
    if len(buffer) != grid['w'] * grid['h'] * 4:
        return None
    image = Image.frombytes('RGBA', (grid['w'], grid['h']), buffer)
    # Řádek 0 occupancy gridu je NEJNIŽŠÍ y; řádek 0 obrázku je HORNÍ. Překlop.
    return image.transpose(Image.FLIP_TOP_BOTTOM)


def _paste_grid(Image, canvas, grid, image, proj):
    """Naškáluj mřížku na místo a ořízni, co spadne mimo plátno."""
    left, top = proj.px(grid['ox'], grid['oy'] + grid['h'] * grid['res'])
    right, bottom = proj.px(grid['ox'] + grid['w'] * grid['res'], grid['oy'])
    width = int(round(right - left))
    height = int(round(bottom - top))
    if width < 1 or height < 1 or width > 12000 or height > 12000:
        return False
    scaled = image.resize((width, height), Image.NEAREST)
    x, y = int(round(left)), int(round(top))
    cw, ch = canvas.size
    cx0, cy0 = max(0, x), max(0, y)
    cx1, cy1 = min(cw, x + width), min(ch, y + height)
    if cx1 <= cx0 or cy1 <= cy0:
        return False
    if (cx0, cy0, cx1, cy1) != (x, y, x + width, y + height):
        scaled = scaled.crop((cx0 - x, cy0 - y, cx1 - x, cy1 - y))
    canvas.paste(scaled, (cx0, cy0), scaled)
    return True


def _polyline(draw, proj, points, colour, width=2, closed=False):
    pixels = [proj.px(x, y) for x, y in points]
    if len(pixels) < 2:
        return
    if closed:
        pixels = pixels + [pixels[0]]
    draw.line(pixels, fill=colour, width=width, joint='curve')


def _dot(draw, x, y, radius, fill, outline=None):
    draw.ellipse([x - radius, y - radius, x + radius, y + radius],
                 fill=fill, outline=outline)


def _draw_robot(draw, proj, robot):
    """Poloha A natočení — samotná tečka by schovala půlku toho podstatného.

    Pořadí nese význam: tělo dolů první, šipka na něj, protože obráceně tělo
    dost velké na to, aby se dalo najít pohledem, šipku spolkne a obrázek
    potichu přijde o natočení.
    """
    x, y = proj.px(robot['x'], robot['y'])
    body = max(5.0, proj.length(0.22))
    arrow = max(26.0, proj.length(1.4))
    yaw = robot['yaw']
    tip = (x + arrow * math.cos(yaw), y - arrow * math.sin(yaw))
    barb = 0.34 * arrow
    left = (tip[0] - barb * math.cos(yaw - 0.5), tip[1] + barb * math.sin(yaw - 0.5))
    right = (tip[0] - barb * math.cos(yaw + 0.5), tip[1] + barb * math.sin(yaw + 0.5))

    _dot(draw, x, y, body + 2, (0, 0, 0))
    _dot(draw, x, y, body, C_ROBOT)
    draw.line([(x, y), tip], fill=(0, 0, 0), width=8)
    draw.line([(x, y), tip], fill=C_ROBOT, width=4)
    draw.polygon([tip, left, right], fill=C_ROBOT, outline=(0, 0, 0))


def _nice_scale_length(metres):
    """Největší 1/2/5 × 10^n, které se ještě vejde do `metres`."""
    if metres <= 0:
        return 1.0
    exponent = math.floor(math.log10(metres))
    for step in (5.0, 2.0, 1.0):
        candidate = step * (10 ** exponent)
        if candidate <= metres:
            return candidate
    return 10 ** exponent


def _draw_scale_bar(draw, proj, font):
    """Měřítko v metrech — bez něj pixely čtenáři nic neříkají."""
    max_px = 0.30 * proj.width
    metres = _nice_scale_length(max_px * proj.scale)
    length = proj.length(metres)
    x = proj.x0 + 22
    y = proj.y0 + proj.height - 26
    draw.rectangle([x - 8, y - 22, x + length + 8, y + 12], fill=(0, 0, 0))
    draw.line([(x, y), (x + length, y)], fill=INK, width=3)
    for end in (x, x + length):
        draw.line([(end, y - 7), (end, y + 7)], fill=INK, width=3)
    draw.text((x, y - 20), '%g m' % metres, fill=INK, font=font)


def _draw_north(draw, proj, font):
    """Střelka k severu.  Pevná, protože celý render je pevný: sever nahoru."""
    x = proj.x0 + proj.width - 58
    y = proj.y0 + 30
    draw.rectangle([x - 40, y - 18, x + 40, y + 48], fill=(0, 0, 0))
    draw.line([(x, y + 34), (x, y - 10)], fill=INK, width=3)
    draw.polygon([(x, y - 16), (x - 8, y - 2), (x + 8, y - 2)], fill=INK)
    draw.text((x - 22, y + 32), 'SEVER (N)', fill=INK, font=font)


def _measure(draw, text, font):
    try:
        box = draw.textbbox((0, 0), text, font=font)
        return box[2] - box[0], box[3] - box[1]
    except Exception:                                   # noqa: BLE001
        return (len(text) * 7, 12)


def _wrap(draw, text, font, max_px):
    """Zalom řádek legendy, ať nemůže utéct z plátna.

    Oříznutá legenda je horší než dlouhá: čtenář nepozná, že se něco uřízlo,
    a useknuté „vrstva geofence NEDOSTUPNÁ — …" se čte jako celá věta.
    """
    words = (text or '').split()
    if not words:
        return ['']
    lines, current = [], words[0]
    for word in words[1:]:
        candidate = current + ' ' + word
        if _measure(draw, candidate, font)[0] <= max_px:
            current = candidate
        else:
            lines.append(current)
            current = word
    lines.append(current)
    return lines


# ---------------------------------------------------------------------------
# legenda — všechno, co obrázek nedokáže říct sám za sebe
# ---------------------------------------------------------------------------
def _legend_lines(status, requested, proj, stamp, site, robot):
    """Řádky (barva_vzorku nebo None, text) v pořadí čtení."""
    rows = [(None, 'Rám: %s · sever nahoru (+Y), východ vpravo (+X) · '
                   'měřítko %.3f m/px · výřez %.1f × %.1f m'
             % (FRAME, proj.scale,
                proj.bbox[2] - proj.bbox[0], proj.bbox[3] - proj.bbox[1]))]
    rows.append((None, 'Site: %s · vykresleno %s UTC · bez satelitní/fotomapy '
                       '(záměrně, jen vlastní data robota)' % (site or 'neznámý', stamp)))
    if robot:
        rtk = robot.get('rtk')
        quality = {'fixed': 'RTK fixed', 'float': 'RTK float',
                   'none': 'bez RTK fixu'}.get(rtk, 'GNSS kvalita neznámá')
        rows.append((None, 'Robot: %s, stáří %.1f s · %s (poloha je relativní '
                           'vůči mapě, ne absolutní GNSS)'
                     % (robot['source'], robot.get('age_s') or 0.0, quality)))
    rows.append((None, '—'))

    swatches = {
        'grid': [(C_FREE[:3], 'volná projetá plocha (= vrstva Base/Saved ve web UI)'),
                 (C_OCC[:3], 'překážka'),
                 (C_UNKNOWN[:3], 'NEZMAPOVÁNO (není to volno!)')],
        'costmap': [(C_COST_LETHAL[:3], 'globální costmap — nafouknutá překážka')],
        'local_costmap': [(C_LOCAL_LETHAL[:3], 'lokální costmap — okamžitá překážka')],
        'zones': [(C_ZONE, 'zóna')],
        'geofence': [(C_GEOFENCE, 'geofence (hranice, kam smím)')],
        'waypoints': [(C_WAYPOINT, 'uložený waypoint'),
                      (C_SITE_PATH, 'uložená trasa')],
        'plan': [(C_PLAN, 'globální plán'), (C_LOCAL_PLAN, 'lokální plán')],
        'lidar': [(C_SCAN, 'co vidí lidar právě teď')],
        'lidar_objects': [(C_OBJECT, 'detekovaný objekt (obrys + jméno)')],
        'robot': [(C_ROBOT, 'ČERVENÁ ŠIPKA = robot, hrot ukazuje, kam je otočený')],
    }

    for name in requested:
        state = status.get(name) or {}
        kind = state.get('kind')
        note = state.get('note') or 'bez důvodu'
        if kind == 'ok':
            detail = state.get('note') or ''
            for colour, label in swatches.get(name, [(DIM, name)]):
                rows.append((colour, '%s — %s%s'
                             % (name, label, (' · ' + detail) if detail else '')))
                detail = ''
        elif kind == 'empty':
            rows.append(((110, 116, 128),
                         'vrstva %s je PRÁZDNÁ (přečteno, nic tam není) — %s'
                         % (name, note)))
        else:
            rows.append(((90, 94, 104),
                         'vrstva %s NEDOSTUPNÁ (nepřečteno, nevím) — %s'
                         % (name, note)))
    rows.append((None, 'Co není nakreslené, není prázdné — je to nezměřené. '
                       'Šedá plocha = nikdo tam nebyl.'))
    return rows


# ---------------------------------------------------------------------------
# veřejné API
# ---------------------------------------------------------------------------
UNKNOWN_SEEN = []   # neznámá jména z posledního `normalise_layers` (pro `unknown` ve výsledku)


def normalise_layers(layers):
    """(vrstvy, stížnosti).  Bere seznam, n-tici i 'a,b,c'.

    Neznámé jméno render nikdy neshodí — stane se z něj varování, protože
    překlep v chatu nesmí stát obrázek.
    """
    del UNKNOWN_SEEN[:]
    if layers is None:
        return list(DEFAULT_LAYERS), []
    if isinstance(layers, str):
        layers = [part.strip() for part in layers.replace(';', ',').split(',')]
    wanted, complaints = [], []
    for name in layers:
        name = str(name or '').strip().lower()
        if not name:
            continue
        if name == 'all':
            wanted = list(LAYERS)
            continue
        name = ALIASES.get(name, name)
        if name in REFUSED:
            complaints.append('vrstva %r se nekreslí — %s' % (name, REFUSED[name]))
        elif name not in LAYERS:
            complaints.append('neznámá vrstva %r — ignoruji ji (platné: %s)'
                              % (name, ', '.join(LAYERS)))
            UNKNOWN_SEEN.append(name)
        elif name not in wanted:
            wanted.append(name)
    if not wanted:
        wanted = list(DEFAULT_LAYERS)
    return [name for name in LAYERS if name in wanted], complaints


def compose(layers=None, size=DEFAULT_WIDTH, center='robot',
            radius=DEFAULT_RADIUS_M, out=None, timeout_s=DEFAULT_TIMEOUT_S,
            keep=DEFAULT_KEEP, site=None):
    """Slož požadované vrstvy do jednoho PNG.  Vždy vrací dict, nikdy nevyhazuje.

        {ok: True,  path, layers, empty, missing, bbox, scale_m_per_px, size,
         frame, site, stamp, robot, legend, warnings[], bytes}
        {ok: False, stamp, warnings[], error: '<jednořádkový český důvod>'}

    `layers`  seznam nebo 'a,b,c'; 'all' = všechno.
    `size`    šířka MAPOVÉ části v px (výška se dopočte z poměru obsahu);
              lze předat i (šířka, výška).
    `center`  'robot' = čtverec o straně 2·`radius` kolem robota,
              'map'   = celý obsah všech vrstev.
    `out`     cílový soubor; None = `output_dir()` s razítkem a úklidem.
    """
    stamp = time.strftime('%Y-%m-%d %H:%M:%S', time.gmtime())
    try:
        return _compose(layers, size, center, radius, out, timeout_s,
                        keep, site, stamp)
    except Exception as exc:                            # noqa: BLE001
        return {'ok': False, 'stamp': stamp, 'warnings': [],
                'error': 'Neočekávaná chyba při skládání mapy: %s' % exc}


def _span_bounds(bounds, data, span_m):
    """Výřez zúžený na čtverec `span_m` kolem robota, nebo beze změny.

    Nikdy nerozšiřuje: chtít 40 m z 20metrové site by obrázek jen vycpalo
    šedou.  A nikdy pod `MIN_SPAN_M` — pod tím začne obrázek lhát o detailu.
    """
    if not span_m:
        return bounds, None
    try:
        span = float(span_m)
    except (TypeError, ValueError):
        return bounds, 'rozsah %r není číslo — kreslím celý výřez' % (span_m,)
    if span <= 0:
        return bounds, None
    span = max(MIN_SPAN_M, span)
    robot = _drawable(data.get('robot'))
    if not robot:
        return bounds, ('výřez %.0f m kolem robota se neuplatnil: bez polohy '
                        'robota není kolem čeho ořezávat' % span)
    west, south, east, north = bounds
    if (east - west) <= span and (north - south) <= span:
        return bounds, None
    half = 0.5 * span
    cx, cy = float(robot['x']), float(robot['y'])
    return (cx - half, cy - half, cx + half, cy + half), None


def _compose(layers, size, center, radius, out, timeout_s, keep, site, stamp):
    requested, warnings = normalise_layers(layers)
    warnings = list(warnings)

    pil = _pil()
    if pil is None:
        return {'ok': False, 'stamp': stamp, 'warnings': warnings,
                'error': 'Knihovna Pillow není nainstalovaná, mapu nemám čím vykreslit.'}
    Image, ImageDraw, ImageFont = pil

    try:
        if isinstance(size, (list, tuple)):
            map_w = max(300, min(3000, int(size[0])))
            map_h = max(300, min(3000, int(size[1])))
        else:
            map_w = max(300, min(3000, int(size)))
            map_h = map_w
    except Exception:                                   # noqa: BLE001
        map_w = map_h = DEFAULT_WIDTH

    site = site or active_site()
    datum = _datum(site)
    timeout = max(0.2, float(timeout_s))
    needs_tf = {'robot', 'lidar', 'lidar_objects'} & set(requested)
    if needs_tf:
        _prime_tf(timeout)
    # Robota potřebuje i ořez kolem robota, i když si o vrstvu nikdo neřekl:
    # jinak by `--center robot` tiše spadl na celý výřez.
    center = str(center or 'robot').lower()
    want_robot = 'robot' in requested or center == 'robot'
    if want_robot and 'robot' not in requested:
        _prime_tf(timeout)

    # ---- sběr (paralelně; každý sběrač odpoví (payload, chyba)) ----------
    jobs = {}
    if 'grid' in requested:
        jobs['grid'] = lambda: _collect_grid(MAP_TOPIC, timeout)
    if 'costmap' in requested:
        jobs['costmap'] = lambda: _collect_grid(COSTMAP_TOPIC, timeout)
    if 'local_costmap' in requested:
        jobs['local_costmap'] = lambda: _collect_grid(LOCAL_COSTMAP_TOPIC, timeout)
    if 'lidar' in requested:
        jobs['lidar'] = lambda: _collect_scan(timeout)
    if 'lidar_objects' in requested:
        jobs['lidar_objects'] = lambda: _collect_lidar_objects(timeout)
    if want_robot:
        jobs['robot'] = lambda: _collect_robot(timeout)
    if 'plan' in requested:
        jobs['plan'] = lambda: _collect_plan(timeout)
    if 'waypoints' in requested:
        jobs['waypoints'] = lambda: _collect_waypoints(timeout, site, datum)
    if 'zones' in requested:
        jobs['zones'] = lambda: _collect_zones(timeout)
    if 'geofence' in requested:
        jobs['geofence'] = lambda: _collect_geofence(site, datum)

    data = _gather(jobs, timeout) if jobs else {}
    for name in requested:
        if name not in data:
            data[name] = (None, 'vrstva neodpověděla ve svém limitu')

    status = {}
    for name in requested:
        payload, error = data.get(name) or (None, 'nic se nenačetlo')
        if isinstance(payload, dict) and payload.get('empty'):
            status[name] = {'kind': 'empty', 'note': payload['empty']}
            warnings.append('%s (prázdná): %s' % (name, payload['empty']))
        elif payload is None:
            status[name] = {'kind': 'missing', 'note': error or 'bez důvodu'}
            warnings.append('%s (nedostupná): %s' % (name, error))
        else:
            status[name] = {'kind': 'ok', 'note': ''}

    bounds = _content_bounds(data)
    if bounds is None:
        return {'ok': False, 'stamp': stamp, 'warnings': warnings,
                'error': ('Není z čeho mapu složit — žádná z vrstev (%s) '
                          'nedodala data.' % ', '.join(requested))}
    span_m = None
    if center == 'robot':
        try:
            span_m = 2.0 * float(radius if radius is not None else DEFAULT_RADIUS_M)
        except (TypeError, ValueError):
            span_m = 2.0 * DEFAULT_RADIUS_M
    bounds, complaint = _span_bounds(bounds, data, span_m)
    if complaint:
        warnings.append(complaint)

    # ---- plátno ---------------------------------------------------------
    # Mapová část si drží šířku od volajícího, ale bere jen tu výšku, kterou
    # poměr obsahu opravdu potřebuje: nacpat vysokou site do pevného obdélníku
    # znamená zahodit rozlišení na té ose, která ho neměla nazbyt.
    span_x = max(1e-6, bounds[2] - bounds[0])
    span_y = max(1e-6, bounds[3] - bounds[1])
    map_h = max(300, min(map_h * 2, int(round(map_w * span_y / span_x))))
    proj = _Proj(bounds[0], bounds[1], bounds[2], bounds[3], (0, 0, map_w, map_h))
    small = _font(ImageFont, 15)
    tiny = _font(ImageFont, 13)
    bold = _font(ImageFont, 17, bold=True)

    canvas = Image.new('RGB', (map_w, map_h), BG)
    draw = ImageDraw.Draw(canvas)

    def payload_of(name):
        return _drawable(data.get(name))

    # ---- vrstvy v pořadí kreslení ---------------------------------------
    grid = payload_of('grid') if 'grid' in requested else None
    if grid:
        lut = {0: C_FREE, 100: C_OCC, 255: C_UNKNOWN, 205: C_UNKNOWN}
        for value in range(1, 100):
            lut[value] = C_OCC_PROB
        image = _grid_image(Image, grid, lut)
        if image is None or not _paste_grid(Image, canvas, grid, image, proj):
            status['grid'] = {'kind': 'missing', 'note': 'mřížku se nepodařilo vykreslit'}
            warnings.append('grid (nedostupná): mřížku se nepodařilo vykreslit')
        else:
            status['grid']['note'] = '%dx%d buněk po %.2f m (%s)' % (
                grid['w'], grid['h'], grid['res'], grid['topic'])

    for name, lethal, soft in (('costmap', C_COST_LETHAL, C_COST_SOFT),
                               ('local_costmap', C_LOCAL_LETHAL, C_LOCAL_SOFT)):
        grid = payload_of(name) if name in requested else None
        if not grid:
            continue
        lut = {100: lethal, 99: lethal}
        if name == 'costmap':
            for value in range(90, 99):
                lut[value] = C_COST_INSCRIBED
        for value in range(1, 90):
            lut[value] = soft
        image = _grid_image(Image, grid, lut)
        if image is None or not _paste_grid(Image, canvas, grid, image, proj):
            status[name] = {'kind': 'missing', 'note': 'mřížku se nepodařilo vykreslit'}
            warnings.append('%s (nedostupná): mřížku se nepodařilo vykreslit' % name)
        else:
            status[name]['note'] = grid['topic']

    zones = payload_of('zones') if 'zones' in requested else None
    if zones:
        for zone_name, ring in zones['zones']:
            _polyline(draw, proj, ring, C_ZONE, width=3, closed=True)
            cx = sum(p[0] for p in ring) / len(ring)
            cy = sum(p[1] for p in ring) / len(ring)
            px, py = proj.px(cx, cy)
            draw.text((px, py), zone_name, fill=C_ZONE, font=tiny,
                      stroke_width=2, stroke_fill=(0, 0, 0))
        status['zones']['note'] = 'zón: %d' % len(zones['zones'])

    fence = payload_of('geofence') if 'geofence' in requested else None
    if fence:
        _polyline(draw, proj, fence['ring'], (0, 0, 0), width=7, closed=True)
        _polyline(draw, proj, fence['ring'], C_GEOFENCE, width=4, closed=True)
        status['geofence']['note'] = 'vrcholů: %d' % len(fence['ring'])

    waypoints = payload_of('waypoints') if 'waypoints' in requested else None
    if waypoints:
        for path_name, line in waypoints['paths']:
            _polyline(draw, proj, line, (0, 0, 0), width=6)
            _polyline(draw, proj, line, C_SITE_PATH, width=3)
            if line:
                px, py = proj.px(*line[0])
                draw.text((px + 6, py - 16), path_name, fill=C_SITE_PATH,
                          font=tiny, stroke_width=2, stroke_fill=(0, 0, 0))
        for point_name, x, y in waypoints['points']:
            px, py = proj.px(x, y)
            _dot(draw, px, py, 6, C_WAYPOINT, outline=(0, 0, 0))
            draw.text((px + 9, py - 8), point_name, fill=C_WAYPOINT, font=tiny,
                      stroke_width=2, stroke_fill=(0, 0, 0))
        status['waypoints']['note'] = 'bodů: %d, tras: %d (%s)' % (
            len(waypoints['points']), len(waypoints['paths']), waypoints['source'])

    plan = payload_of('plan') if 'plan' in requested else None
    if plan:
        if plan['global']:
            _polyline(draw, proj, plan['global'], (0, 0, 0), width=7)
            _polyline(draw, proj, plan['global'], C_PLAN, width=4)
        if plan['local']:
            _polyline(draw, proj, plan['local'], (0, 0, 0), width=6)
            _polyline(draw, proj, plan['local'], C_LOCAL_PLAN, width=3)
        status['plan']['note'] = 'bodů globálně: %d, lokálně: %d (%s)' % (
            len(plan['global']), len(plan['local']), plan['topic'] or '—')

    scan = payload_of('lidar') if 'lidar' in requested else None
    if scan:
        radius_px = 1.6 if len(scan['points']) > 400 else 2.4
        for x, y in scan['points']:
            px, py = proj.px(x, y)
            if (proj.x0 <= px < proj.x0 + proj.width
                    and proj.y0 <= py < proj.y0 + proj.height):
                _dot(draw, px, py, radius_px, C_SCAN)
        status['lidar']['note'] = 'platných paprsků: %d z %d (rám %s)' % (
            len(scan['points']), scan['total'], scan['frame'])

    objects = payload_of('lidar_objects') if 'lidar_objects' in requested else None
    if objects:
        for obj in objects['objects']:
            _polyline(draw, proj, obj['ring'], (0, 0, 0), width=5, closed=True)
            _polyline(draw, proj, obj['ring'], C_OBJECT, width=3, closed=True)
            px, py = proj.px(*obj['centre'])
            draw.text((px + 7, py - 8), obj['label'], fill=C_OBJECT, font=tiny,
                      stroke_width=2, stroke_fill=(0, 0, 0))
        status['lidar_objects']['note'] = 'objektů: %d' % len(objects['objects'])

    robot = payload_of('robot')
    if robot and 'robot' in requested:
        _draw_robot(draw, proj, robot)
        azimuth, yaw_deg, point = heading(robot['yaw'])
        status['robot']['note'] = (
            'x=%.2f y=%.2f · míří na %s — azimut %.0f° od severu po směru '
            'hodin (= yaw %+.0f° v rámu map, 0°=východ, proti směru hodin)'
            % (robot['x'], robot['y'], point, azimuth, yaw_deg))

    _draw_scale_bar(draw, proj, small)
    _draw_north(draw, proj, tiny)

    # ---- legenda --------------------------------------------------------
    rows = _legend_lines(status, requested, proj, stamp, site, robot)
    row_h = max(19, _measure(draw, 'Ag', small)[1] + 8)
    wrapped = []
    for colour, text in rows:
        if text == '—':
            wrapped.append((colour, text, False))
            continue
        indent = 38 if colour is not None else 16
        for index, line in enumerate(_wrap(draw, text, small, map_w - indent - 16)):
            wrapped.append((colour if index == 0 else None, line, index > 0))
    legend_h = int(38 + row_h * len(wrapped) + 16)

    full = Image.new('RGB', (map_w, map_h + legend_h), BG)
    full.paste(canvas, (0, 0))
    canvas = full
    draw = ImageDraw.Draw(canvas)

    top = map_h
    draw.rectangle([0, top, map_w, map_h + legend_h], fill=PANEL)
    draw.line([(0, top), (map_w, top)], fill=LINE, width=2)
    draw.text((16, top + 10), 'Mapa robota Vitulus — legenda', fill=INK, font=bold)
    y = top + 36
    for colour, text, continuation in wrapped:
        if colour is not None:
            draw.rectangle([16, y + 3, 30, y + 15], fill=colour, outline=LINE)
            draw.text((38, y), text, fill=INK, font=small)
        elif text == '—':
            draw.line([(16, y + 9), (map_w - 16, y + 9)], fill=LINE, width=1)
        elif continuation:
            draw.text((38, y), text, fill=INK, font=small)
        else:
            draw.text((16, y), text, fill=DIM, font=small)
        y += row_h

    # ---- uložení --------------------------------------------------------
    if out:
        path = os.path.expanduser(str(out))
        directory = os.path.dirname(path) or '.'
        keep = 0                                        # cizí adresář neuklízíme
    else:
        directory = output_dir()
        path = os.path.join(directory, _filename())
    try:
        os.makedirs(directory, exist_ok=True)
        tmp = path + '.tmp'
        canvas.save(tmp, 'PNG', optimize=True)
        os.replace(tmp, path)
    except Exception as exc:                            # noqa: BLE001
        return {'ok': False, 'stamp': stamp, 'warnings': warnings,
                'error': 'Zápis mapy na disk selhal: %s' % exc}
    _prune(directory, keep)

    drawn = [n for n in requested if status.get(n, {}).get('kind') == 'ok']
    empty = [n for n in requested if status.get(n, {}).get('kind') == 'empty']
    missing = [n for n in requested if status.get(n, {}).get('kind') == 'missing']
    return {
        'ok': True,
        'path': path,
        'frame': FRAME,
        'site': site,
        'layers': drawn,
        'requested': requested,
        'unknown': list(UNKNOWN_SEEN),
        'empty': empty,
        'missing': missing,
        'center': center,
        'radius_m': (0.5 * span_m) if span_m else None,
        'bbox': {'west': round(proj.bbox[0], 2), 'south': round(proj.bbox[1], 2),
                 'east': round(proj.bbox[2], 2), 'north': round(proj.bbox[3], 2)},
        'scale_m_per_px': round(proj.scale, 4),
        'size': [map_w, map_h + legend_h],
        'stamp': stamp,
        'robot': ({'x': round(robot['x'], 2), 'y': round(robot['y'], 2),
                   'yaw': round(robot['yaw'], 3), 'rtk': robot.get('rtk'),
                   'source': robot.get('source')} if robot else None),
        'warnings': warnings,
        'legend': [text for _colour, text in rows if text != '—'],
        'bytes': os.path.getsize(path) if os.path.isfile(path) else 0,
    }


def summary(result):
    """Jedna věta, která jde do promptu VEDLE obrázku, ne místo něj.

    Říká, co na obrázku je a čemu se nedá věřit — ne co je na něm vidět.
    Popis obsahu je práce modelu; kdyby ho psal tenhle kód, nebyl by to
    pohled, jen další text.
    """
    if not result or not result.get('ok'):
        return (result or {}).get('error') or 'mapa se nesložila'
    bits = []
    robot = result.get('robot') or {}
    if robot:
        bits.append('robot x=%.2f y=%.2f v rámu map (červená šipka)'
                    % (robot['x'], robot['y']))
        if robot.get('rtk'):
            bits.append('RTK %s' % robot['rtk'])
    bbox = result.get('bbox') or {}
    if bbox:
        bits.append('výřez %.1f × %.1f m'
                    % (bbox['east'] - bbox['west'], bbox['north'] - bbox['south']))
    if result.get('layers'):
        bits.append('vrstvy: %s' % ', '.join(result['layers']))
    if result.get('empty'):
        bits.append('PRÁZDNÉ (přečteno, nic tam není): %s' % ', '.join(result['empty']))
    if result.get('missing'):
        bits.append('NEDOSTUPNÉ (nevíme): %s' % ', '.join(result['missing']))
    bits.append('sever nahoru, legenda je součástí obrázku pod mapou')
    return ' · '.join(bits)
