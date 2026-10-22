# Component geometry: from envelopes to recognisable parts

Many MachineKit components still draw as their envelope: a box or a plain cylinder of the right size. That is
enough for clearance and packaging, but a catalogue reader, a drawing or an editor user cannot tell a contactor
from a power supply, or a servo motor from a proximity switch. This plan gives every component a recognisable
`Preview` geometry without changing what the rest of the stack relies on.

## Audit (2026-10-21)

Measured on the docs catalogue export (`docs/components`): triangle count of the preview mesh, and fill ratio
(solid volume over bounding-box volume; 1.0 is a solid box, about 0.79 a plain cylinder).

| Tier | Count | What it is |
|---|---|---|
| Detailed | 59 | More than 600 triangles: fasteners, gears, couplings, most machine parts, supplier imports |
| Simple but rough | 92 | 13–600 triangles: plain cylinders, rings and stepped blocks |
| Pure boxes | 44 | 12 triangles, fill 1.0 |
| No model | 9 | The recipe's defaults do not build a part |

Of the 156 rough or missing ones, 112 are generic recipes, 11 supplier parts, 16 reference envelopes and 17
parts of one machine. 137 compute their mass from the preview geometry, 10 declare it.

### Pure boxes (44)

- Electrical, DIN-rail: safety relay, contactor, circuit breaker, motor driver, power supply, three-phase
  rectifier, VFD, brake resistor, thermistor input, digital sensor.
- Controllers and cabinets: CNC controller, robot controller.
- Pneumatics: air supply, manifold, solenoid valve, vacuum control valve, vacuum generator, vacuum pressure
  sensor, Schmalz SBP vacuum generator.
- Hydraulics (press brake): hydraulic manifold, valve, supply, tool clamp, gauge finger.
- Linear motion: MGN rail, MGN carriage block, linear scale.
- Printing: fan, heated bed.
- Robotics: frame bar, parallel-gripper finger, Schmalz SXT master.
- Structural: block bracket, drawing-derived profile member, mineral casting.
- Plates by nature (keep, with holes and edge breaks where they have them): enclosure panel, mill panel,
  spoilboard, parallel key, tooling block, panel and volume, hobber bracket.

### Plain cylinders, rings and blocks (the larger set)

- Motors: servo motor, spindle motor, axis servo and brake servo envelopes; torque motor.
- Sensors: shaft encoder, rotary encoder ring, proximity switch, limit switch.
- Bearings: angular-contact and crossed-roller (plain annuli), pillow block, flange housing.
- Reducers: gearbox (its fill ratio of 2.26 shows its mass is not its shape), planetary reducer.
- Pneumatic and hydraulic cylinders and rods, suction cups, hoses, fittings.
- Tooling and spindles: ATC spindle, ER20 and ISO30 holders, ER32 collet, tool length setter, tool rack, tool
  fork, dust shoe, vacuum blower and zone valve.
- Printing: heater cartridge, thermistor, hotend, extruder.
- Robotics: arm joint, tool changers, gripper body and jaw, wrist fork.
- Hobber and bench-mill reference envelopes and machine parts (owned by those machines' plans).

### No model (9)

- Supplier references whose pinned CAD is not prepared on the exporting machine: HIWIN HG block reference,
  HGH15CA reference, Vallder BF12 and screw support, HZMotion supplier component, RBK motor mount, ZPP member.
- Recipes without defaults that build: cast-in steel insert (`role`), tooling source casting (`source`).

## Principles

1. **Two detail levels keep their meaning.** `Envelope` stays the clearance and packaging solid that
   collision, layout and integrity checks use; it does not change. `Preview` becomes the recognisable part
   and must lie inside the envelope.
2. **Mass is not a side effect of drawing.** A purchased part declares its datasheet mass, centre and
   (where known) inertia; adding preview detail must not change it. A made part keeps computing mass from its
   geometry, and more accurate geometry is then a real correction, reviewed as such.
3. **Interfaces do not move.** Connectors, ports, mounting holes and catalogue dimensions are data the
   geometry follows, never the reverse.
4. **Our own geometry from public dimensions.** Preview geometry is modelled parametrically from datasheet
   dimensions (housing, flanges, terminals, ports, shafts). Supplier STEP files stay reference inputs for
   checks; meshes derived from them are not published unless their licence allows redistribution.
5. **Shared generators, not one-off models.** Most rough parts belong to a handful of product shapes. A
   generator per shape keeps them consistent and makes a new catalogue row cheap.
6. **Fidelity is stated.** Each recipe declares how faithful its preview is, so the catalogue and drawings can
   say so and the backlog is measurable.

## Steps

| Step | What |
|---|---|
| G0 | **Fidelity and audit.** Recipes declare a geometry fidelity: `Envelope` (shape is the envelope), `Nominal` (recognisable, generic proportions), `Datasheet` (key dimensions from a cited datasheet), `Supplier` (supplier CAD, reference only unless licensed). The catalogue shows it as a badge. A MachineKit test reports the envelope-only recipes and fails if one that had a preview regresses. |
| G1 | **Mass decoupling.** A purchased part's mass is not its drawn housing. Decided 2026-10-21: it is its envelope's volume times an assumed effective density for its product class (DIN-rail device, drive or power supply, control cabinet, pneumatic body, hydraulic body), reported as such (`MassSource.EffectiveDensity`), and a catalogue row's datasheet mass overrides it. The audit found these parts computed as solid blocks of their housing material, 5–10 times too heavy (a 4.2 kg safety relay, a 26 kg VFD, 24 kg controllers), so this corrects masses rather than freezing them; example baselines that depend on them are updated with the reason. Motors, sensors, bearings and reducers follow in their geometry batch (G4). |
| G2 | **Containment check.** A test that every recipe's preview lies inside its envelope (bounding-box first, then a boolean check for a sample of rows), and that connector frames are unchanged by the preview. |
| G3 | **Shared generators.** `DinRailDevice` (width in modules, stepped front, screw-terminal rows, label face, rail clip), `ControlEnclosure` (door, hinges, handle, vents, feet, gland plate), `MotorBody` (square flange with pilot and holes, body, rear cap, connector boss, keyed shaft; servo, stepper and spindle variants), `BearingRings` (inner and outer rings, shields or seals, visible rolling elements for crossed-roller), `ProfileExtrusion` (closed section along +Z for rails, scales and drawing profiles), `PneumaticBody` (body with port bosses, solenoid coil block, fittings), `Fan` (frame, corner holes, hub, blades). |
| G4 | **Motion batch.** MGN rail and block (profile, holes, end caps, seals, grease nipple), linear scale and read head, servo/spindle/brake-servo motors via `MotorBody`, encoders, proximity and limit switches, bearings via `BearingRings`, gearbox and planetary reducer, torque motor. |
| G5 | **Electrical batch.** All DIN-rail devices via `DinRailDevice`; controllers via `ControlEnclosure`; VFD and power supply with heat-sink fins. |
| G6 | **Pneumatic and hydraulic batch.** Valves, manifolds, vacuum generators, pressure sensor, FRL air supply, cylinders (ISO 15552-style end caps and tie rods), rods (threaded ends), suction cups (lip and bellows), hydraulic supply, valve and manifold. |
| G7 | **Tooling, spindle and printing batch.** ATC spindle, ER and ISO30 holders, ER collet (slots), tool length setter, tool rack and fork, dust shoe, vacuum blower and zone valve; fan, heated bed, heater cartridge and thermistor with leads, hotend, extruder. |
| G8 | **Robotics and structural batch.** Arm joint and link, tool changers, gripper body, jaw and fingers, wrist fork; block bracket, leveling foot, slot nut; drawing profile member via `ProfileExtrusion`. |
| G9 | **Missing models.** Give the cast-in insert and source casting representative defaults; decide per supplier reference whether its prepared CAD can be used at export (licence) or a datasheet preview replaces it. |
| G10 | **Machine-owned envelopes.** Hobber and bench-mill reference envelopes and machine parts move to their machines' plans; the catalogue can hide machine parts by default once the generic set is complete. |

## Status (2026-10-22)

- G0–G3 done. Generators in use: `MotorBody`, `ProfileRailShapes`, `RoundHousing`, `DinRailDevice`, and `RingParts`
  (bearings, ring encoders, frameless motors), which replaced the planned `BearingRings`.
- G4 done. Linear rails and carriages became one recipe each: rows keyed by carriage designation (MGN, HGH, HGW),
  drawn nominal, measured or from the pinned supplier CAD, with datasheet masses and load ratings by source
  (`LinearRailSystem`). The MGN rows were checked against HIWIN's 2023 MG catalogue. Bearings, the ring encoder,
  the torque motor and the linear scale keep their annulus or strip as the mass model. The read head is not drawn yet.
- Next: G5 (electrical), then G6–G8; Vallder screw supports and HZMotion parts should follow the rail pattern
  (one recipe per family, drawing as a variant, sourced masses).

Each batch regenerates the docs catalogue and is reviewed on a contact sheet of before and after thumbnails,
alongside the editor.

## Checks

- MachineKit recipe contract tests, plus G0's fidelity report and G2's containment test.
- Mass and inertia of every recipe before and after a batch: unchanged for purchased parts, reviewed for made
  parts.
- Example machines: assembly integrity, collision and drive sizing suites, and the project-source cases that
  build them.
- Docs catalogue: regenerated models and thumbnails, contact sheet review.

## Open decisions

- Whether supplier-CAD-derived meshes may be published on the docs site (per supplier licence), or only our
  own datasheet-dimensioned geometry.
- Whether the docs catalogue lists machine parts by default.
