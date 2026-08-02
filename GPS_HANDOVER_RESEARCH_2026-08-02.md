# Rešerše: přechod dobré GPS → GPS-denied → zpět (RTK sekačka)
2026-08-02, deep-research (104 agentů, 15 ověřených zjištění, 1 vyvrácený claim)

## Hlavní zpráva: náš design je potvrzený literaturou — a nic z něj není overkill

Všech šest našich mechanismů má přímou oporu v dokumentované praxi:
- **FIXED-only + kovarianční gating** — standard.
- **Lag-buffer s okenní konzistencí** (heading anchor v2, 2–2,5 s) — dokumentovaný
  „persistence" vzor (kritéria musí platit několik kroků po sobě); konkrétní N
  literatura neudává, naše 2,5 s je empirické a legitimní.
- **Motion-verified anchoring** (GPS delta vs. wheel delta) — dokumentovaný vzor
  „dead-reckoning jako reference pro gating GNSS" (Li 2017, Harr 2018).
- **Probace při re-akvizici** (1 m shody / 10 s klidu) — obdoba vendorské
  konzervativní akceptace (NovAtel ARTK default Q4 = 99,9 % konfidence).
- **Hystereze korekcí** — obdoba NovAtel „on engage only" (re-fixy jen mimo misi,
  výslovně proti mid-mission skokům pózy).
- **Arbiter architektura** (ownership switching) — validována ETH Graph-MSF
  (ICRA 2022): dual-graph s okamžitým switch-backem PŘEKONÁVÁ kontinuální EKF
  blendování, do kterého výpadek+návrat GNSS zanáší drift; hladké přes reálné
  výpadky 105–380 s. Aplikace korekce do map→odom transformace (ne skokem do
  lokální pózy) = přesně co děláme.
- „První fix po výpadku může být špatný" je **vendorem přiznaný failure mode**
  (wrong ambiguity fix) — naše nedůvěra je na místě.

## Co nám chybí (seřazeno podle poměru přínos/náklad)

1. **Protection Level z F9P (UBX-NAV-PL)** — přijímač umí hlásit garantovaný
   bound chyby (validovaný proti reálným scénářům, silnější než kovariance/pAcc);
   u-blox manuál obsahuje přímo use-case robotické sekačky (PL řídí rychlost/
   překryv/stop). Čistá konfigurace + parsování zprávy v gnss driveru →
   gate `plPosValid && PL < práh` vedle stávajících cov gates. Nejlepší „early
   warning" nástupu degradace, který máme zadarmo v HW.
2. **RTK_CAR mód (CFG-NAVHPG-DGNSSMODE, HPG 1.50+)** — konzervativní ambiguity
   fix: delší time-to-fix a víc FLOAT epoch výměnou za téměř jistý FIXED.
   Jedna konfigurační hodnota; receiver-side řešení wrong-fix pasti při výjezdu
   ze dvora. (Ověřit verzi FW našich F9P.)
3. **Formální NIS/chi-square gate na inovacích** — analytický práh z chi-kvadrát
   rozdělení (místo ručních prahů); robot_localization to už umí
   (`mahalanobis_threshold` na pose vstupech). Levné, zachytí pomalu narůstající
   multipath, který snapshot gaty propustí.
4. **C/N0 monitoring (UBX-NAV-SIG)** — dokumentovaný leading indicator multipathu
   před vjezdem do dvora; dnes nesledujeme vůbec. Minimálně logovat do telemetrie,
   případně přidat do fix_usable váhy.
5. **FIXED→FLOAT přechod jako leading indicator** — dokumentováno; my dnes FLOAT
   jen zahazujeme. Levné: FLOAT epocha = signál „degradace začíná" → preemptivně
   zmrazit heading anchor (dnes to řeší až variance band).
6. **Explicitní ZUPT/NHC pseudo-měření** (nulová rychlost při stání, žádný boční
   skluz) — dokumentovaná brzda driftu (−91 % chyby přes 60 s výpadek s ML-NHC);
   u nás roli hraje kolová odometrie + tracker, takže přínos menší — nice-to-have.
7. **Inflace R místo tvrdého zahození** u degradovaných-ale-přítomných měření —
   dokumentovaná alternativa; náš binární gating je ale v arbiter architektuře
   legitimní (arbiter > blendování, viz Graph-MSF). Neměnit bez důvodu.

## Otevřené otázky (literatura nedala odpověď)

- Konkrétní hodnota N pro lag-buffer — nikde publikována, naše 2,5 s zůstává empirie.
- Četnost wrong fixů na F9P v poli (default vs. RTK_CAR) — žádná studie; kandidát
  na vlastní A/B test (RTK_CAR na jednom přijímači, default na druhém, náš dvůr).
- Pre-mapované GNSS trust zóny (deny-list dvora) vs. reaktivní gating — přímé
  srovnání neexistuje; reaktivní metody jsou dokumentované jako soběstačné.
  Kandidát na experiment: logovat PL/C-N0/fix-type podle polohy → mapa kvality
  zdarma z běžného provozu, pak teprve rozhodnout o geofence.
- Jump vs. slew vs. backward smoothing pro aplikaci korekce — ověřen jen map→odom
  update vzor (který máme); kvantitativní srovnání chybí.

## Doporučený akční plán

Hned (konfigurace + malý kód): (1) PL gate, (2) RTK_CAR, (3) mahalanobis_threshold
na GPS vstupu nav EKF. Do telemetrie: (4) C/N0 + (5) FLOAT-as-warning → preemptivní
heading freeze. Experiment při validačních jízdách: logovat PL/C-N0 podle polohy
(základ případné trust mapy dvora). Nedělat: přechod na kontinuální blendování
či FGO (arbiter je literaturou potvrzen jako robustnější pro náš případ).

## Klíčové zdroje
u-blox F9P Interface/Integration manual (UBX-NAV-PL, CFG-NAVHPG-DGNSSMODE);
NovAtel ARTK appnote (Q4, on-engage-only); ETH Graph-MSF (Nubert et al., ICRA 2022,
github.com/leggedrobotics/graph_msf); DLR/EPFL ION GNSS+ 2017 (innovation vs
residual testing); Frontiers in Physics 2025 RAIM survey; IEEE 2018 C/N0-weighted
EKF integrity; Li 2017 (PMC5621381), Harr 2018 (arxiv 1801.02058) DR-as-reference;
Wen et al. NAVIGATION 68(2) FGO vs EKF; Sünderhauf switchable constraints pro GNSS.
