# Machine enclosure plan

**Goal.** One modular enclosure kit and generator that every machine uses: bench mill, hobber,
lathe, skiver, router, printer, press brake. From a machine's own data, it generates an
enclosure that:
- clears every axis position;
- contains chips, coolant, mist or dust for its process;
- keeps people safe through interlocked doors;
- gives loading and service access;
- can be made in-house (router-cut panels, press-brake folds, welded plastic) or from finished
  panels we buy.

1. Each panel has a **zone** (wet chip zone, splash, impact, cladding, thermal envelope) and a
   **construction** from a panel catalogue (compact laminate, GRP, welded PP, FR ACM, steel, Al
   honeycomb, insulated sandwich, polycarbonate). Zone rules check the construction fits.
2. The generator places the frame, panels, windows, doors, service panels and pass-throughs
   around the machine's swept envelope, loading corridor and service zones.
3. Doors use one shared signal interface (open, close, closed-and-locked,
   standstill-confirmed unlock), reused by the tending cell and every machine.
4. Outputs: BOM, router and press-brake jobs, lamination and welding steps, sealant and
   fastener lists, and a build document.

The work spans:
- `machinekit` (`machinekit/haxe/src/machinekit/enclosure/`): parts, panel catalogue, zone
  rules, generator, checks;
- `camkit`: router jobs for laminate and ACM, including V-groove rout-and-fold;
- the sheet-metal and press-brake work (`cadkit/plans/SHEET_METAL.md`,
  `examples/press-brake/`) for steel trays, trims and production panels;
- `processkit`: the shared door and guard signal interface;
- `projectkit` `MaterialLibrary`: panel materials and coatings;
- examples: bench mill first, then hobber, lathe, skiver, router, printer.

## What exists

Surveyed 2026-10-19 on local main (shared checkout) and branch `flexspline-hobber`.

- **Bench mill:** `examples/bench-mill/EnclosedBenchMill.hx` builds convex enclosure panels from
  `MillPanel` (2 mm sheet), a chip tray, a stand service panel, and a front opening with a
  sliding door on a rail (`doorStroke`, door ids). The machine-tending cell drives the door
  (`examples/machine-tending/TendingCyclePlan.hx`: open, request close, closed; no cut before
  door-closed confirmation). Bench-mill FEM (`FEM_PLAN.md`) and mould-tooling work also exist on
  main from another session.
- **Hobber:** an enclosure recipe placeholder (envelope and loading opening, panels omitted for
  inspection), chip tray, sloped table guard, coolant-manifold placeholder, rail guards; front
  loading and the X retract corridor check; plinth with internal chip and coolant volume
  (HB-D11).
- **Press brake:** interlock and light-curtain signals (`processkit/pressbrake/...`).
- **Structural parts:** `machinekit.structural` (`TSlotExtrusion`, `FrameAssembly`,
  `ProfileMember`, `Angle`, `Channel`, `SeatedPlate`, …).
- **Making panels:** sheet-metal design and the press brake; CNC router CAM (CamKit profile and
  drill, `CamSheetProfiles` / `SheetCutPlan`).
- **Checks to reuse:** `AssemblyIntegrity` (interference, sampled sweeps), and the machines'
  travel-grid layout checks (which give a swept envelope).

**Missing:**
- a shared enclosure model: zones, panel catalogue, coatings, rules, generator;
- the shared door and guard interface (each machine has its own logic);
- impact, coolant and fire properties of panels;
- ACM V-fold and laminate/weld fabrication routes;
- mist and dust extraction, pass-throughs, lighting, status light;
- coupon-test records.

## Decisions

**EN-D1. Zones decide construction.** Every panel face gets a zone:

| Zone | Meaning | Default construction (first choice / alternative) |
|---|---|---|
| **Wet chip zone** | Direct chip and coolant stream: around the tool, table, trough walls | Chemical-grade **compact laminate** / welded PP liner (hobbing) / stainless liner tray |
| **Splash** | Wetted but not in the chip stream: side walls, roof inside, door inner skins | Compact laminate / **GRP-laminated or GRP-faced FR ACM** / stainless-faced ACM |
| **Chute / tray** | Surfaces where chips slide to the trough | **Welded PP or HDPE**, UHMW-PE for sliding / stainless |
| **Impact** | Line of possible ejection (doors and walls facing the cut) | **Steel sheet** / a panel with an impact **test record** |
| **Window** | Viewing | **Rated polycarbonate** with a replaceable inner pane (never acrylic) |
| **Cladding** | Outside, covers, light doors | **FR ACM** (router V-fold) / painted aluminium / Al honeycomb for large doors |
| **Thermal envelope** | Precision machines that need stable temperature | **Insulated sandwich panels**, or insulation behind cladding |

- Rules:
  - plain or painted ACM is allowed only in cladding (painted ACM also in splash);
  - impact zones need steel, rated polycarbonate, or a test record;
  - every ACM edge and fastener in a wet or splash zone is marked sealed;
  - oil-mist zones use FR-grade materials;
  - heat limits are checked against the process (hot lathe chips vs welded PP or UHMW).
- A violation names the panel, zone and rule.

**EN-D2. A panel catalogue with real properties.**
- Each construction records:
  - layers (for laminates: skin, adhesive, core, skin);
  - density and stiffness;
  - coolant and chemical rating;
  - heat limit;
  - fire class;
  - impact status (none, rated, tested with a record);
  - fabrication route: router-cut, V-fold, press-brake fold, hot-gas weld, wet lay-up, bought
    finished;
  - corner and joint method (profile, fold, weld, trim);
  - assumed cost per m².
- `LaminatedPanel` covers stainless-faced ACM, GRP-on-ACM and similar.
- Coatings are finish attributes: epoxy and PU machine paint, polyurea + PU topcoat,
  ceramic-filled epoxy patches. Powder coat is **disallowed on ACM** (oven cure vs core).
- All values are marked assumed until a datasheet or a coupon test (EN-D7) replaces them.

**EN-D3. Frame plus infill, generated from the machine.**
- An extrusion frame (T-slot, 40/45 series) with infill panels by default. A folded sheet-metal
  or welded production enclosure uses the same panel layout later.
- Generator inputs, all from the machine:
  - **swept envelope** (layout checks' travel grid plus margin);
  - **loading direction and corridor** (e.g. hobber front loading);
  - **service zones** (hob change, spindle, swivel, table) with their access directions;
  - **process type** (wet machining, dry/dust, heated chamber, press);
  - **floor and plinth interfaces** (trough, coolant tank, feet);
  - **pass-through needs** (cables, coolant, air, mist).
- Output: frame members, panels with zones, windows, doors, service panels, pass-throughs, mist
  or dust port, lighting, status light.

**EN-D4. One door and guard interface.**
- `processkit` signals:
  - door open/close request;
  - closed;
  - locked;
  - unlock permitted only after **spindle and axis standstill** is confirmed;
  - setup mode (reduced speed, hold-to-run with door open, per machine rules);
  - E-stop chain;
  - optional light curtain at the cell boundary.
- The bench mill and tending cell port onto it first, and must behave exactly as now. Every
  generated door is wired to it.
- The machine refuses to cut while the door is not closed-and-locked (the tending rule,
  generalised).

**EN-D5. Process types set containment.**

| Process type | Containment and extras |
|---|---|
| **Wet machining** (mill, lathe, hobber, skiver) | Coolant-tight seams and drip edges, mist extraction port, impact windows, floor to trough |
| **Dry/dust** (router) | Dust extraction port and airflow, lighter windows, spark and fire considerations |
| **Heated chamber** (printer) | Insulation, chamber temperature control, ACM throughout |
| **Press** (press brake) | Open front with light curtain, side and rear guards |

**EN-D6. Fabrication uses our own machines where it can.**
- ACM: router profile plus **V-groove rout-and-fold** (a new CamKit op: groove depth and angle,
  fold allowance, minimum flange).
- Compact laminate: router cut, with corner profiles.
- Steel trays and trims: press brake.
- Welded PP: cut on the router, then hot-gas weld (a documented step).
- GRP lay-up: a documented step (cloth, resin, cure, flow coat).
- Finished panels can be bought (stainless-faced composite, Al honeycomb, insulated sandwich);
  the BOM says which.

**EN-D7. Coupon tests turn assumed properties into measured ones.**
- Samples of each construction:
  - soaked in our actual coolant and oil for weeks (swelling, adhesion, edge ingress);
  - hot-chip drops;
  - abrasion and cleaning;
  - a basic drop or projectile impact test for any panel proposed for an impact zone.
- Results are recorded as panel-catalogue values with ValueBasis Measured. No construction counts
  as impact protection without a test record (or a rated product).

**EN-D8. The enclosure helps precision too.**
- For precision machines (hobber, skiver, gear-measuring station), the generator can add a
  thermal envelope (insulated panels, no direct sun or draft openings) and report an estimated
  air-temperature time constant. This feeds HB14-style thermal models later.

**EN-D9. Generic names in code;** vendors only as sources in docs. Standards are cited as
references (ISO 16090-1 milling machines, ISO 23125 lathes, ISO 14119 interlocks). The plan
does not claim certification.

## Test machines

- **Bench mill (enclosed):** the existing panels and door, ported. The tending cell must still
  pass.
- **Hobber:** front loading door, hob-change service panel, mist port, plinth trough, thermal
  envelope option.
- **Lathe** (`examples/lathe/PLAN.md`): heavier impact rating, hot chips.
- **Router:** the dust variant. **Printer:** the heated-chamber variant.

## Steps

### Phase A: model

**EN0. Baseline.** Record the gate. Snapshot the enclosed bench mill and tending results
(door timing, cycle) that must not change.

**EN1. Panel catalogue and materials (EN-D2).**
- Materials and coatings in `MaterialLibrary` (FR ACM, compact laminate, GRP, PP/HDPE/UHMW, Al
  honeycomb, stainless-faced ACM, insulated sandwich, polycarbonate), with properties and
  ValueBasis. `LaminatedPanel`.
- Tests: every construction validates; powder coat on ACM is rejected; properties are carried
  into the BOM.

**EN2. Zones and rules (EN-D1, EN-D5).**
- Zone assignment, rule engine, process-type containment.
- Tests: each rule fires on a crafted panel and names it; a cladding-only ACM in a wet zone
  fails; an impact zone without steel, rated polycarbonate or a test record fails.

**EN3. Enclosure parts.**
- `EnclosureFrame`, `EnclosurePanel`, `EnclosureWindow`, `SlidingDoor` (guide, actuator, guard
  lock, closure sensor), `ServicePanel`, `PassThrough`, `MistPort` / `DustPort`, `DripTray`,
  `StatusLight`, `EnclosureLight`.
- Tests: recipes, BOM rows, geometry valid.

**EN4. Generator (EN-D3).**
- From swept envelope, loading corridor, service zones, process type, plinth and floor
  interfaces, and pass-through needs.
- Tests:
  - the generated enclosure clears the swept envelope at every travel sample (interference
    check);
  - the open door clears the loading corridor;
  - each service zone has a removable panel that can be reached;
  - seams in wet and splash zones are sealed.

**EN5. Door and guard interface (EN-D4).**
- `processkit` signals and state machine; standstill-confirmed unlock; setup mode.
- Tests:
  - no cut unless closed-and-locked;
  - unlock is refused while the spindle turns;
  - setup mode limits speed;
  - the E-stop chain opens.

### Phase B: machines

**EN6. Bench mill port.**
- `EnclosedBenchMill` builds through the kit and generator. Its door moves to the shared
  interface.
- Tests: the tending cell passes with unchanged door timing and cycle time (EN0 snapshot); zone
  rules pass.

**EN7. Hobber enclosure.**
- Front loading door, hob-change and swivel service panels, mist port, trough interface, an
  optional thermal envelope (EN-D8).
- Tests: the generator output passes all checks; the loading and X-retract corridor stays clear;
  app scene shows it (with a hide toggle for inspection).

**EN8. Fabrication outputs (EN-D6).**
- CamKit V-groove rout-and-fold op; router jobs for ACM and compact laminate; press-brake jobs
  for steel trays and trims; weld and lay-up steps; a build document.
- Tests: V-fold jobs reproduce panel outlines and fold lines in the router simulation; the
  press-brake jobs validate; the build document lists every part once.

### Phase C: proof and rollout

**EN9. Coupon tests (EN-D7).**
- A physical test programme. Results entered as Measured in the catalogue.
- Done when: every default construction in EN-D1 has coolant, heat and abrasion results; any
  impact-zone candidate has an impact record.

**EN10. Rollout.** Lathe (impact and hot chips), skiver, router (dust variant), printer
(heated-chamber variant), press brake (guard and light-curtain variant).
- Tests: each machine's generated enclosure passes its process type's rules.

### Phase D: later

- Production enclosures (welded or folded sheet metal) from the same layouts.
- Cost model per construction.
- Noise modelling.
- Fire-suppression provisions for oil-mist machines.
- Robot cell fencing with the same kit.

## Order

```
EN0 ── EN1 ── EN2 ── EN3 ── EN4 ── EN5 ── EN6 ── EN7 ── EN8 ── EN10
                                                         EN9 (physical, any time after EN1)
```

- EN6 must keep the tending cell's behaviour identical; it is the regression anchor.
- EN7 waits for the hobber's layout revision and bought C table to land.

## Working rules

- Own worktree from local `main` (the bench mill is on main). The hobber step (EN7) needs the
  hobber branch on main.
- Dependencies count only once they are on local `main`.
- A blocked step is marked `blocked: <what>`.
- Commit per step, e.g. `Enclosure: zones and rules (EN2)`, with Progress updated.
- Numbers not from a datasheet or a coupon test are marked assumed.
- No legacy compatibility.
- Heavy suites go through the heavy-run wrapper.

## Progress

| Step | State | Commits |
|---|---|---|
| EN0 | planned | |
| EN1 | planned | |
| EN2 | planned | |
| EN3 | planned | |
| EN4 | in progress: fitted front-loading generator and plinth/tray interface | |
| EN5 | in progress: shared guard controller, end switches, fitted adapter and live hobber runtime binding; hardware qualification remains | |
| EN6 | planned | |
| EN7 | in progress: current hobber geometry fitted; live guard integrated; containment and hardware qualification remain open | |
| EN8 | planned | |
| EN9 | planned | |
| EN10 | planned | |
| Phase D | later | |

### EN4 / early EN7 hobber fit — 2026-10-19

The user requested fitting the enclosure to the current hobber before the
bench-mill port. Generic layout/cut-panel/generator code remains in branch
`enclosure`; it was also integrated into the existing `flexspline-hobber`
worktree for current-CAD acceptance. Existing mechanical edits were preserved;
no merge to main or change to the open/cutting previews was required.

`EnclosureLayout` sizes a front-loading shell and right-hand door parking from
machine-frame swept bounds, a loading reservation and the plinth interface.
`GeneratedEnclosure` produces a frame, removable side service panels, roof,
rear mist opening and a steel tray ring with four upstands. `EnclosureCutPanel`
has a recipe, exact CAD through-cut, net mass/centre of mass and BOM area, plus
collision proxies preserving its aperture. Rectangular proxies are exact;
a round port proxy reserves its circumscribed square.

The hobber adapter derives X/Y/Z endpoints and swivel limits from the machine.
It uses at most one-degree swivel spacing and radius-times-half-step padding;
B/C descendants get conservative rotation-centred sphere bounds. The separate
electrical rack is excluded from sizing but included in collision acceptance.
The front loading reservation follows the existing OD140 jaw assumptions and
extends outward through the shell; it does not author a robot path.

Initial dimensions: about 1472 × 1103 × 787 mm clear shell above the plinth,
tray datum Z847 mm, 600 mm opening and 610 mm door stroke. The 75 mm margin,
100 mm mist opening, service outside space, OD140 jaw reservation and 60 C
process assessment are assumed. Reported dimensions are CAD-derived.

Open the new hobber `materia.enclosed.project.json`: **enclosed** shows the shell,
**inspection** hides enclosure occurrences. Outputs and the focused acceptance
command are documented in `examples/flexspline-hobber/ENCLOSURE.md` in that
worktree. The normal project and its cutting preview are untouched.

Acceptance covers 48 machine/door states, exact near-pair intersections,
loading and removable service openings, port cut/mass/proxy and save/reopen.
The report exposes qualification violations rather than treating geometry as
protective or fluid-containment acceptance. Seals, plinth transition, retention,
tray slope/flow, extractor hardware and guard controls remain open. EN4/EN7
therefore remain in progress; this is the fitted geometry milestone.

Final fitted acceptance passed: 248 envelope samples; 48 machine/door states;
648 exact near-pair checks; loading reservation and both removable service
openings clear; port CAD/net mass/aperture proxy and assembly save/reopen pass.
The 7 MiB scene and layout/check JSON files are exported under the hobber's
`build/enclosure/`. `ENCLOSURE_LAYOUT_REPORT.json` records the mechanical source
revision and derived layout. The report retains 52 unresolved qualification,
sealing and plant-interface requirements. No protective/containment acceptance
or unattended cutting cycle is claimed.

### Electrical rack correction — 2026-10-19

Corrected the rack's 637 mm elevated base and the horizontal device orientation.
The base is now at machine-floor Z0, with two welded triangular support webs;
one supply and six drives mount upright, flush to the backplate. Added focused
floor/contact/coverage and device-overlap acceptance to the existing enclosure
runner. Both rack checks and the renewed 48-state enclosure acceptance pass
(624 exact near-pair checks). The transparent-window desktop was reopened with
the updated scene and visually verified. The initial enclosure dimensions are
unchanged because the external rack is excluded from shell sizing.

### EN5 controller contract and fitted hobber adapter

The shared `processkit.guard.EnclosureGuard` now models closed-and-locked cutting
permission, standstill-confirmed unlock, independently confirmed lock feedback,
hold-to-run setup with a configured speed limit, E-stop/boundary interruption,
latched feedback faults/timeouts and explicit safe reset. Maintained cut requests
do not automatically restart after access or reset.

The generated sliding door exports closed/open joint switches and a 200 mm/s
kinematic rate. `HobberEnclosureControl` consumes actual assembly joint position,
requires a safety-output consumer, and preserves independent external lock and
standstill feedback. Focused tests use the fitted 610 mm joint, sensor save/reopen,
and simulated motion-permission consumption. These are simulation/control-contract
tests; physical actuator/lock sizing, safety validation, the live cutting-runtime
binding and the unchanged-timing bench-mill port remain follow-on work.

### EN5 live enclosed-hobber runtime

The enclosed artifact carries explicit door/sensor/hazardous-axis bindings and
feedback policy. The app reads native joint switches and measured velocities,
qualifies consecutive standstill samples, and runs the guard before the hobbing
player on each tick. Cutting starts only on an explicit guarded request. Door
access requests a controlled cutting stop; the independently simulated lock
releases after confirmed standstill, then a process-owned actuator moves the
leaf without changing cutting-axis plans or brake ownership. The Machine panel
exposes door, cutting and reset actions with feedback and explanatory status.

Interrupted cutting requires a full simulation reset (including stock) and a
fresh start. Physical actuator/lock selection, safety qualification, bench-mill
port and containment/fabrication requirements remain open. The live path does
not expose door-open setup jogging. See the hobber ENCLOSURE.md for assumed
rates, confirmation policy and the native integration check.
