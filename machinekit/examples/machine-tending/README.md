
The isolated dual electric wrist can be inspected through
`materia.electric-tool.project.json` (the shared `DhDualToolPreview` in
`../dh-gripper`). It uses two supplier PGC references, CNC finger variants and a
prototype orthogonal adapter. Build/check instructions and assumptions are in
`../dh-gripper/README.md`. It is a tool-design reference; the active pneumatic
mission, electrical controls and tending reach certificate are not replaced by
opening it.


The electric access study is a separate, bounded design check:

```sh
./haxeon/scripts/haxeon run --project=machinekit/examples/machine-tending/haxeon.electric-access.json
./haxeon/scripts/haxeon run --project=machinekit/examples/machine-tending/haxeon.electric-audit.json
```

It reads the stored pneumatic layout only for the proposed pedestal/tray positions,
builds the current mill and bare-arm geometry, and writes
`build/electric-access/report.json` and static `.mtrg` snapshots. It does not update
the active tending mission or its reach certificate. The two configurations are
the original side-entry blank and the same blank turned within the existing CNC
fingers for top entry. Both must place a flat 60 x 60 x 20 mm stock mesh on the
actual station datum. The top-entry stock contact audit checks 604 mm² on each
finger with zero CAD penetration.

Four workpiece headings, both grippers, and all fourteen vise/tray targets are
screened. General clearance is the existing planner's 5 mm; selected working
members use its 0.5 mm contact clearance near their fixture. Only the active blank
may touch its named stock-support members without penetration in the last 25 mm
of insertion/withdrawal. The unused gripper, cables, neighboring stock and other
fixtures retain 5 mm. Only the active jaw pair opens; its companion stays closed
on the other blank. Active-blank occupancy is conservatively retained while
opening, so these states are clearance envelopes, not grasp simulation.

Tool geometry uses triangle meshes, except the cuboid blanks use solid convex
bodies. Fixtures/arm parts use authored convex pieces or an enclosing 26-DOP
of their display mesh with a 0.001 mm outward offset. A hull may fill holes and
concavities: its interference flag must be distinguished from an exact CAD clash.
Mesh/convex zero distance can mean either touching or overlapping surfaces;
negative mesh distances are not reliable CAD penetration depths. The independent
audit deliberately checks separated, 0.4 mm-gap, touching and 2 mm-overlap boxes
against analytic results and CAD. Solid convex stock is required for the special
contact filter to reject penetration reliably. The audit then compares a reported
top-entry cable/link pair against the actual CAD solids and exports both posed
STEP files plus `collision-audit.json`. It also writes `service-cad-audit.json`,
checking both revised cables and all six support/cap pieces against actual arm link,
joint housing and tool-flange CAD at that one solved pose. This is a pose-specific
check; it does not validate a flexible forearm harness or full wrist motion.

Vise candidates use a conservative whole-tool aperture fit and a fixed-orientation
route through the open doorway to a 100 mm approach. Tray candidates use a local
100 mm approach with the door closed. Cartesian samples are no more than 10 mm
apart. A bounded set of IK seeds screens the best geometric heading at the saved
pedestal position; arm/tool and arm/environment checks follow. IK is checked
against CAD-assembly flange position and orientation. Arm self-collision, travel
between stations, continuous joint-motion certification, electrical control,
flexible-cable behavior and grip retention are not commissioned by this study.
Failure at these finite rolls/seeds is not proof that no alternative route exists.

The cable routing is still a design assumption. The supplier drawing shows a
6.4 mm cable, a 100 mm clearance callout and R50 allowable bend radius; the chosen
R55 curve, return leg, remote-connector position and support/clamp geometry are
our prototype. In particular, the clamp spans 90–100 mm from the jacket exit;
the meaning of the drawing's 100 mm callout relative to this fixation needs
confirmation or a longer lead. The exact cable part, mating connector, pinout,
assembly access, dynamic bend/torsion ratings and wrist-motion service loop remain
unverified. A successful rigid geometric check would not establish those facts.


The two tray supports use the reusable `machinekit.structural.TSlotTable`
assembly with HFS5-4040 posts, upper and lower rails, drilled corner angles,
bolts and end-loaded slot plate nuts, threaded levelling feet with locknuts,
and a counterbored 10 mm steel top. The upright front row is inset 100 mm for
robot clearance. Tops still follow the tray layout, while independent low bases
are about 720 x 670 mm at the stored 1019 mm top height. Width extends away from
the robot's central passage; depth extends forward with the rear base edge flush
with the top. Rear diagonal straps brace both upright frames. These are geometric
layout constraints; full robot reach and structural load qualification remain pending.

The generator preserves semantic subassemblies in the scene: upper frame, lower
frame, base (when fitted), feet, worktop, and optional bracing. Each frame's
connections contain their angle and fasteners; each foot groups its adapter,
levelling foot, locknut and mounting hardware. The cell names its supports
`left-table` and `right-table`. `partPath(member)` resolves a manufacturing member
name to its grouped occurrence path; cut-list names remain manufacturing labels.
The editor displays authored assembly groups and applies visibility and selection
navigation to their descendants.

Detailing follows dimensions and the selected `TSlotTableHardware` specification:

- HFS5-2020 defaults to M4 slot fasteners and M5 feet; HFS5-4040 defaults to M5
  slot fasteners and M8 feet. A caller can supply another validated hardware spec.
- Sixteen angles have two clearance holes each. Their screws seat against the
  angle faces; each nut fits the slot undercut. Slot plates must be loaded from
  the open extrusion ends before closing the frame.
- Four top mounting holes follow the upper rails. Counterbore depth adapts to
  top thickness and an available nominal screw length, recessing the heads while
  retaining at least 2 mm of plate. Screws retain 0.2 mm of slot bottom clearance.
- Each leg has a drilled and tapped bottom bore matching the foot thread. Foot
  stems retain at least 1.5 diameters of engagement through the reported adjustment
  range; a separate hex nut locks each foot against the leg.
- `rearBrace: true` adds a drilled diagonal rear strap and its slot fasteners.
  Bracing is an explicit choice, not an automatic stiffness or load calculation.

The constructor retains its existing dimension arguments, followed by optional
hardware, rear-brace and base-footprint arguments. With no footprint the original
floor-standing post construction is retained. A footprint adds low side rails,
cross members under each post row, angle connections, and separate tapped foot
adapter plates; end cross rails are added where there is room between connections.
The footprint must contain the upper frame. Its optional offsets default to zero.
For example:

```haxe
var bench = new TSlotTable(1200, 700, 900, "HFS5-4040", 18, 0, 40, 200, null, true);
var compact = new TSlotTable(320, 240, 650, "HFS5-2020", 10, 0, 25, 150);
var wideBase = new TSlotTable(540, 330, 1020, "HFS5-4040", 10, 100, 30, 220, null, true,
    {width: 720, depth: 670, offsetX: 90, offsetY: -170});
```

`memberCuts()` exposes extrusion cuts; `cutList()` aggregates profile stock.
`topMounts()` returns the top drilling/counterbore pattern, `machining()` reports
drilling and tapping for the selected foot mount (post end or base adapter), and
`footAdjustment()` reports nominal, minimum and
maximum foot extension. `billOfMaterials()` includes every placed hardware part.
Incompatible slots, obstructed driver space at the angles, thin tops, insufficient
foot engagement, and dimensions too small for the connections are rejected.

These are nominal dimensional reference specifications, not verified supplier
SKUs or load ratings. Angles use specified machined thicknesses; slot plate nuts
have semantic threads and unmodeled thread teeth. Hardware fit uses the actual
modeled slot depth, including the existing 2020 profile's corner-web allowance,
rather than assuming the nominal catalog depth. Supplier compatibility, strength,
and complete assembly-tool paths still need qualification before fabrication.

Run the sizing, CAD hole, mating interference, BOM and collision checks:

```sh
./haxeon/scripts/haxeon run --project=machinekit/examples/machine-tending/haxeon.table-checks.json
```

Generated files under `machinekit/examples/machine-tending/build/`:

- `tslot-tables.mtrg`: both detailed tray supports.
- `tslot-table-sizing.mtrg`: six layouts, including both square profiles, asymmetric
  and centred wide bases, and optional bracing.
- `tslot-table-manufacturing.json`: upper/base dimensions, member cuts, top holes,
  foot-mount machining, adjustment ranges and hardware BOMs for those layouts.

The reach study uses separate conservative envelopes for every member and hardware
part, preserving their poses. The table revision is included in the certificate
fingerprint; the full tending reach certificate still requires regeneration.

The electric access entry now builds the selected Fairino Expert harness preview
and writes to `build/electric-expert-access/`, preserving the historical baseline
`build/electric-access/` reports. The independent Expert service CAD audit lives in
`../dh-gripper/haxeon.expert-audit.json`; it reuses historical solved joint poses
only, and recomputes service/robot geometry checks. Neither study qualifies the
external-power dress pack, wrist connector seating, adapter pinout or DH compatibility.


Reach memory diagnostics can run the already-built study without storing a new
certificate:

```sh
./haxeon/scripts/haxeon build --project=machinekit/examples/machine-tending/haxeon.json
python3 machinekit/examples/machine-tending/profile_reach.py --output=/tmp/reach-setup --census
python3 machinekit/examples/machine-tending/profile_reach.py --search --seconds=180 --output=/tmp/reach-search --census
```

To run the full study and regenerate the certificate only on success, use
`profile_reach.py --store --seconds=1800 --max-rss-mib=3072 --output=/tmp/reach-full`.
The 4 GiB available-memory reserve still applies; this command runs existing
bytecode and does not compile concurrently with the study.

The default probe builds one scene and deliberately rejects an altered proposal
before IK, exercising setup reuse. `--search` follows the real reuse/fallback
path without writing the certificate. The monitor samples process RSS, anonymous
and file-backed memory, swap and system available memory into `memory.jsonl`;
`reach.log` records phase names and VM heap/allocation counters. `allocations.json`
contains cumulative allocations and sampled call stacks, not retained live bytes.
`summary.json` identifies normal exit, program failure, or a deliberate guard stop.
The default limits are 2 GiB process RSS, 4 GiB available system RAM, and 60 seconds;
the guard interrupts only its own process group. Compiler memory is excluded.

Collision worlds retain lightweight eligible pair records and initialize Coal
collision/distance evaluators only when a pair reaches narrow phase. Pair rules,
geometry, signed distances and broadphase ordering are unchanged. When a saved
layout or candidate is rejected during `--store`, the study reuses its prepared
geometry and self-collision world for fresh search. The explicit `revalidate()`
API remains strict and reports invalid proposals instead of falling back.

### Exact CAD confirmation of electric vise access

Run the affected stations independently with:

```sh
./haxeon/scripts/haxeon run --project=machinekit/examples/machine-tending/haxeon.electric-access.json -- --cached-expert --vise-only
```

This writes `build/electric-expert-vise/` without replacing the full-cell study.
Vise flags now receive a native CAD narrow phase by default. The coarse query
includes 0.35 mm reserve for both mesh deviations; acceptance still requires
5 mm air clearance or the existing scoped 0.5 mm working clearance. Native
solid common volume distinguishes overlap from separation. Supplier gripper
housing surfaces remain surface references, not closed-volume containment proof.
`--coarse-only` disables this refinement for diagnostics.

`exact-vise-checks.json` records native gap, common volume, jaw openings and the
actual pair transforms for every queried candidate. Its `sources` lists hashes
of the local STEP geometry saved under `exact-cad/`; all lengths are in mm.
The rejected baseline is retained under `build/electric-expert-vise-baseline/`.
Exact CAD found approximately 4.90 mm at the doorway header, 4.06 mm at the cutter,
and 3.09 mm at the door actuator in the originally selected approaches. Other
quarter-turn headings have confirmed finger/jaw or connector/head overlaps.

The route now reserves 1 mm beyond the aperture margin, crosses low, aligns
laterally before advancing, raises stock to clear the table, and advances the
wrist inside before the final lift. Vise approaches use 50 mm; tray approaches
remain 100 mm. Distinct bounded endpoint postures are tried through the complete
sampled vise approach, instead of accepting the first clear endpoint posture.
These are routing-study changes; they do not rewrite the pneumatic reach
certificate or establish swept clearance, arm self-collision or moving cable
retention.

Accepted rows save `routeQ` for every Cartesian sample and save `q` from the actual
last solved sample. Independent endpoint CAD review uses the dedicated dataset:

```sh
./haxeon/scripts/haxeon run --project=machinekit/examples/dh-gripper/haxeon.expert-audit.json -- --vise-access --poses-only --approved-only
python machinekit/examples/dh-gripper/review_expert.py --vise-access
```

The vise review requires all four station/gripper cases to pass, validates joint
trace continuity and its final pose, verifies captured CAD hashes, and independently
checks service clearance against all 104 mechanical arm components at the saved
endpoints. It also verifies that reconstructed FK still reaches the reported wrist
pose. Outputs are separate from the historical/full-cell reports; success here
must not be presented as clearance for all tray stations or the full cycle.

Validated result: **4/4 vise station/gripper cases clear** (A at 0°, B at 180°),
with 334 saved joint samples across the accepted routes. The native narrow phase
recorded 85 close-candidate checks using 15 captured CAD sources. The independent
endpoint review passed all 5,824 service/robot separation checks; its nearest
reported separation is a conservative bounds lower limit of 7.78 mm. The full
tray study was not rerun as part of this focused vise check.

### Electric-tool automatic planning (VAMP deferred)

The current study uses one common layout with both tray banks and support tables
set back 50 mm. All 28 station/gripper transfers pass continuous checks and are
rechecked in their actual approach state. Of the 28 approaches, 23 are free-space
clear and five require deliberate stock/fixture contact. All 28 reverse withdrawal
routes pass outside their explicitly reported guarded-contact segments.

The electric four-part sequence now has **52 operations**, returns through the
common park for every vise exchange, and waits at park during door movement.
All 28 complete jaw strokes (17–25 mm per jaw) pass against the stock, fixture,
robot and surrounding tool hardware. The complete door stroke clears the parked
robot/tool, including both held stock envelopes and full jaw-stroke envelopes.

The discrete process simulation delivers all four parts, completes four cuts and
finishes parked with no CNC PLC alarms. It uses the real `RobotInterface` state
machine, not a mock interlock. Thirteen guarded-action checks cover successful
seating/separation, timeout, overtravel, unexpected or missing contact, numerical
penetration, force/speed limits, missing feedback and failed separation. A fault
injected during loading stops the sequence and retains the held stock. A separate
grasp-confirmation failure leaves the finished stock clamped in the vise and
retains the next blank in the companion gripper.

`modelSimulationReady=true` describes this geometric and discrete process model.
**`programReady=false`** remains deliberate: physical force/speed limits and
contact/gripper feedback bindings are unqualified. The simulation does not play
back rigid-body dynamics or drive hardware. No prototype test force value is
used as a hardware setting.

Current results:

- `build/electric-expert-vise-setback-50/planner-report.json`: four vise cases.
- `build/electric-expert-access-setback-50-trays/planner-report.json`: 24 tray cases.
- `process-report.json` alongside each planner report: complete jaw and door
  sweeps, stock separating-plane proofs and declared structural contacts.
- `build/electric-cycle-review.json`: the 52-operation simulation, PLC/event trace,
  contact limits, fault checks, source hashes and remaining hardware requirements.

The tray report uses `current-proposals.json`, a composite of the earlier full
study and the two isolated repaired cases, with a source manifest. Those inputs
are proposals only. Current limits, target poses, assembly and compiled-model FK,
collision geometry and every continuous edge are checked afresh. Tool and
placement-seed identities must match. Arm/fixture changes cannot inherit an old
clearance result. The cycle review rejects mixed geometry or changed source
reports; all transfers must share the same park configuration.

To regenerate a complete proposal study and validate it:

```sh
./haxeon/scripts/haxeon run --project=machinekit/examples/machine-tending/haxeon.electric-access.json -- --cached-expert --trays-only --tray-setback=50 --reuse-endpoints --extended-ik-seeds
./haxeon/scripts/haxeon run --project=machinekit/examples/machine-tending/haxeon.electric-access.json -- --cached-expert --trays-only --tray-setback=50 --plan-saved --check-withdrawal
./haxeon/scripts/haxeon run --project=machinekit/examples/machine-tending/haxeon.electric-access.json -- --cached-expert --vise-only --tray-setback=50 --plan-saved --check-withdrawal --planner-smoke
./haxeon/scripts/haxeon run --project=machinekit/examples/machine-tending/haxeon.electric-access.json -- --cached-expert --trays-only --tray-setback=50 --plan-saved --audit-process
./haxeon/scripts/haxeon run --project=machinekit/examples/machine-tending/haxeon.electric-access.json -- --cached-expert --vise-only --tray-setback=50 --plan-saved --audit-process
./haxeon/scripts/haxeon run --project=machinekit/examples/machine-tending/haxeon.electric-cycle-review.json
```

For the currently saved composite tray proposals, pass
`--proposal-report=/absolute/path/to/current-proposals.json` to both planning and
process-audit runs. Their geometry, proposal hashes and park must agree. The cycle
review requires all 28 cases. `--audit-process` requires `--plan-saved`, writes a
separate report and preserves the validated transfer/withdrawal report.

`TendingCyclePlan.electricBearingBlocks()` enables the new process order;
`bearingBlocks()` retains the legacy pneumatic order and its regression checks.
Electric pickup confirms grasp before unclamping. Placement releases, withdraws,
then runs an explicit `Clamp` operation while the door is still open. Door closure
is requested only afterward. Direct in-vise exchanges are replaced by validated
return-through-park paths. `TendingMission` emits this ordering when supplied the
electric production plan; existing pneumatic missions retain their behavior.
An electric production plan still needs its own matching reach/runtime profile;
the pneumatic reach certificate must not be relabelled as an electric certificate.

The jaw audit encloses each translating part's full stroke with convex sweeps.
Finger bounds come from the same four-box construction recipe as their geometry;
bores and root cuts only remove material. Fine 0.01 mm CAD vertices of both CNC
and SLS variants are covered, and exact symmetric differences between regenerated
CNC fingers and the cached tool STEP files are zero. Stock contact is qualified
by separating planes over the entire affine stroke, rather than waived after a
mesh contact flag. Supplier carriage guides and rigid finger/carriage mounts are
explicit structural contacts. Supplier jaw meshes have a 0.151 mm outward reserve
over the 0.15 mm rendering deflection. Tool-internal and working-fixture clearance
is 0.5 mm; other robot/environment clearance remains 5 mm.

`TendingGuardedAction` supervises only the approved contact path. Travel limits
come from its guarded segments with a 0.12 mm reserve for the source FK tolerance;
the action budget is 90 seconds, matching the mission motion budget. Unexpected
contact, excessive penetration/travel, timeout or configured force/speed feedback
failure latches a robot stop, retains payload/gripper state and inhibits the next
operation. There is no automatic retry or blind retreat. Numerical penetration
is limited to 0.001 mm. Physical force and speed limits remain null until a real
runtime profile supplies qualified values; the model is not a hardware driver.

Use `--case=infeedTray-1-2/b` to solve or validate one case in its own output
directory. `--extended-ik-seeds` adds 48 bounded shoulder/elbow/wrist seeds;
`--reuse-endpoints` still checks every new solution. Requested IK accuracy is
0.0001 mm / 0.000001 rad. The previous 0.00332 mm seat penetration is now a
reported guarded seating contact. The alternative infeed posture has exact CAD
clearances of 238.71 mm to joint 4 and 95.45 mm to link 4; the rejected posture
had only 4.22 mm to joint 4 against the 5 mm policy.

The collision world uses metres and carries the full tool and harness. Long arm
housings use 17 authored convex construction pieces each. The oblique taper's
hull is built in its longitudinal frame, preserving the routed empty space.
Fine 0.01 mm CAD triangles are covered by the proxies. At the formerly flagged
table poses, corrected proxy clearance is 60.77–60.90 mm and exact CAD clearance
is 64.71–64.91 mm. `haxeon.arm-audit.json` replays the captured
`table-collision-witness.json`; `--adapter-only --current-adapter` checks the
repaired pocket posture independently against exact CAD.

Structural contact rules remain explicit. Reference-pose overlaps cannot create
new exemptions. Same-carrier pairs are exempted only when a recorded bounds
calculation proves their required separation. Cable loops retain mesh surfaces.

Pickup approaches use jaws at 25 mm per jaw and stock fixed in the pocket or
clamped vise. Placement uses jaws at 17 mm per jaw and held stock. Withdrawal
switches ownership and jaw state: pickup carries the stock after unclamping;
placement leaves stock fixed and withdraws with open jaws. Companion stock is
retained conservatively. Air clearance is 5 mm, and the authored straight working
phase uses 0.5 mm. Intended seating/sliding contact rejects a free-space edge and
is reported as guarded. Up to 0.001 mm numerical penetration can be classified as
guarded, never accepted as clear; larger penetrations retain a collision error.

Transfers use deterministic RRT-Connect in a reusable conservative world for
each door state. It includes both held stock envelopes, open/closed jaw geometry,
occupied neighbouring pockets and a convex sweep of the vise jaw stroke. The separate
full-stroke process audit certifies modeled actuation geometry; the transfer
world's jaw endpoints alone do not. Proposed transfer routes are freshly
validated; unresolved continuous edges fail closed. Bounded search failure does
not prove that a target is unreachable.

Use `--approaches-only` to skip transfer search, or `--geometry-only` to export
`layout.mtrg` and screen tool clearance without approving arm motion. Flexible
cable motion, supplier mesh surfaces, real connector dimensions, grip retention
and dynamics remain unqualified. VAMP is deferred.

### Electric runtime binding

`TendingElectricRuntimeProfile` binds the hashed electric proposals to the newly
validated transfer paths, applying the recorded joint-turn offsets once. Each
incoming/outgoing edge retains its stock owner, per-jaw opening, door policy and
explicit guarded-contact policy. Both directions return through the common park;
no pneumatic reach certificate is reused.

`TendingElectricEdgeExecutor` is a tick-driven executor over the
`TendingElectricEdgeDriver` interface. The host supplies measured joints, arrival,
contacts, travel and penetration (plus force/speed when configured). Invalid entry,
feedback, contact or timeout latches stop/hold without retry. It does not command
the gripper or change stock ownership; those remain process actions that must
wait for actual grasp/release feedback.

The electric cycle review exports all 28 runtime profiles and runs both directions
through a deterministic driver, plus stop/hold fault injections. These are driver
contract tests, not physics playback. The current `SceneArtifact`/`MissionPlayer`
API has no guarded-edge action bound to this feedback driver. Guarded incoming
paths explicitly reject conversion to ordinary waypoints. Full Materia playback
still requires that host binding, the electric tool/controller scene, and physical
gripper/contact observers. Hardware readiness remains false; force/speed limits
remain unqualified.

Run the isolated feedback-driver checks against the latest review with:

```sh
./haxeon/scripts/haxeon run --project=machinekit/examples/machine-tending/haxeon.electric-runtime-checks.json -- /home/joao/dev/materia/machinekit/examples/machine-tending/build/electric-cycle-review.json
```

This checks 28 round trips (1,308 edges) and nine latched faults, including a
travel-budget overrun across consecutive guarded edges, loss of bilateral grasp
and excessive fixture penetration. The generated
`electric-runtime-checks.json` hashes its input review and explicitly reports
`physicsPlayback: false` and `hardwareReady: false`.

### Electric physics playback

The headless electric host uses Materia's SimKit/RobotKit MuJoCo backend. It loads
arm, fixture and tool meshes by the hashes in `electric-cycle-review.json`, rather
than rebuilding a potentially changed mill. The launcher recovers the reviewed
geometry from the local geometry cache and rejects a changed tool or review source.

```sh
python3 machinekit/examples/machine-tending/run_electric_simulation.py --gripper-check
python3 machinekit/examples/machine-tending/run_electric_simulation.py --servo-check
python3 machinekit/examples/machine-tending/run_electric_simulation.py --withdraw-smoke
python3 machinekit/examples/machine-tending/run_electric_simulation.py --placement-smoke
python3 machinekit/examples/machine-tending/run_electric_simulation.py --seat-settling-check
python3 machinekit/examples/machine-tending/run_electric_simulation.py
```

Build Materia's native provider first if `app/build/host/native/app` is absent.
The launcher rebuilds the frame-contact API before execution. The resulting
`build/electric-gripper-physics-checks.json` is a physical adapter test with
simplified pads and a cube, not a qualification of DH fingers. It checks bilateral
force contact and width, retained grasp on stop, confirmed release and refusal to
report an empty close as grasped. Jaw speed, effort, contact thresholds and servo
gains are explicit simulation assumptions.

`TendingElectricSimScene` adds the imported electric jaws and mechanical coupling
to the arm runtime. Finger collision pieces come from the shared construction
recipe and must enclose the imported vertices. Cable loops use measured convex
decompositions instead of one solid hull spanning each loop. Rigid fasteners use
one enclosing hull per fastener; long-link construction pieces stay separate.
Mass and supplier controller properties remain estimates. The physics proxy is
conservative and differs from the CAD narrow phase; a proxy contact needs diagnosis
before concluding that the manufactured parts collide.

Arm commands use the model's explicit servo gains. Initial position commands
would ramp from the runtime's stale zero reference, so the host submits the seeded
park directly in servo mode. Gravity feedforward uses the combined arm/tool mass
and CoG model, with its sign and DOF mapping checked independently against the
potential-energy derivative. Startup must hold the reviewed park within 0.00001
rad; admission is never relaxed to bypass drift.

The executor reads actual joints, bilateral workpiece contacts and world contacts,
including free-stock/fixture contact. Payload retention is monitored during
transport; jaw/workpiece contact exemptions apply only to the actual jaw links.
Grasp must precede vise opening, and release feedback precedes withdrawal. Stops
latch without a kinematic workpiece attachment or automatic retreat.

The host writes `build/electric-physics-playback.json`, including failed runs,
measured joints, contact diagnostics and stock poses. Fault diagnostics also retain
the active operation, route edge, failing gripper and stock states before the
controlled stop advances physics. Inspect `passed` and `failure`:
starting the host does not certify the four-part cycle. This is headless physics
playback, not `MissionPlayer` UI integration. CNC cutting remains a timed process. The host runs the real CNC PLC with measured
door/vise positions, stock presence and the authored robot-zone boxes. The zone
includes physically gripped free objects as well as robot links. The separate
discrete cycle review continues to test logical interlocks and fault ordering. Hardware readiness and full-cycle
qualification remain false; flexible cable dynamics and timed trajectory tracking
also remain unqualified.

The baseline full MuJoCo run completed 14 operations and one CNC process handshake before
stopping during the second vise load (`vise-load/b`). At the fault, the carried
blank contacted the vise end stop and lost opposite-jaw contact. The stop latched,
the arm reached rest and both blanks were gripped after settling; there was no
automatic retry. The preserved report is
`build/electric-physics-exchange-end-stop-baseline.json`. The authored loading
datum has clearance in Y and Z but none from the end stop in X. Physical loading
therefore needs a reviewed locating strategy that tolerates measured grasp error;
the nominal rigid-payload collision review does not establish that capability.
That baseline four-part physical cycle failed; focused handoff checks must not
be substituted for a complete run of its replacement strategy.

The opt-in clearance-first strategy uses the exact baseline geometry snapshots,
not a fresh assembly built from unrelated working-tree changes. Replan and review
with `--clearance-first` on the access and cycle-review commands. Its artifacts
are separate (`electric-expert-vise-setback-50-clearance-first` and
`electric-cycle-review-clearance-first.json`). The insertion is offset 0.5 mm
from the stop. A separately checked 0.65 mm command corridor permits a bounded
horizontal contact search; only the active stock/end-stop pair can overlap in
that command-envelope check. All live physical contacts retain the 0.001 mm
penetration limit. Robot/tool contact with the stop is rejected.

Both vise locating and tray placement use `TendingContactSearchAction`. Vise
locating commands 0.02 mm/s, with a 0.1 mm/s closing point-speed bound and a
0.5 mm/s total point-speed ceiling. Tray placement commands 0.05 mm/s, with
corresponding bounds of 0.5 and 1.0 mm/s. Rotation contributes to both point-speed
bounds. These are explicit, uncommissioned simulation limits.

Positive axial load above 0.05 N requests a controlled hold on the first loaded
contact sample. The closing-speed limit applies during free approach; the total
point-speed ceiling also bounds elastic settling during contact and hold. Ten
stationary samples below 0.001 mm/s admit release. Tray placement requires
support load throughout those samples. For horizontal vise locating, a measured
touch followed by an uninflated closest-hull gap of at most 0.01 mm admits
stationary elastic recoil without resuming motion. This uses observed poses and
compound physical hulls; proximity without a measured touch never qualifies.
The unused trajectory
is discarded; contact loss never resumes the search. Axial load, resultant force
and each individual contact force are limited to 5 N. Resultants add world-axis
vectors with the correct selected-body sign; opposing friction components are
not added as magnitudes. Unexpected contact, missing wrench feedback, excessive
force/speed/travel/depth or timeout latches a stop.

An intermediate full run passed both vise handoffs and reached the first tray
delivery, where a nominal endpoint drove the imperfectly grasped part into its
supports. That report is retained as
`build/electric-physics-first-tray-support-baseline.json`. Tray placement now
follows the reviewed approach to the final 3 mm of its last segment, checks at
least 1 mm of measured stock clearance, then searches slowly along the remaining
segment and stops on support load. It never extends beyond that reviewed segment.
After release, all four supports, rest and the full 0.001 rad orientation tolerance
must pass. The final-segment subsets and their return continuity are checked for
all twelve tray profiles.

A later full run reached gripper B's vise extraction with a preloaded end-stop
contact already present before motion. Its report is
`build/electric-physics-vise-b-regrasp-preload-baseline.json`. Vise pickup now
registers the open gripper's grasp centre to the observed clamped stock position,
keeping the reviewed wrist orientation. Planning and registration share
`TendingStockDatum`; no stock pose or physical attachment is altered. The part
must pass the clamped full-pose admission, and a configuration-independent
MotionReach bound limits every robot/tool/payload point's correction to 0.12 mm.
A fresh continuous query checks the correction against observed cell geometry.
After grasp and unclamp, extraction first moves outwards toward the same clear
loading datum: 0.5 mm away from the stop, and the authored Y/Z loading clearance.
The first 0.75 mm is slow, guarded separation at 0.05 mm/s, with a
1 mm/s bound on every carried point. Its 70 N loaded-separation ceiling matches
the simulated gripper effort ceiling; it is an uncommissioned simulation bound,
separate from the 5 N free-search limit. Persistent or unexpected contact stops
the operation. After separation, a fresh observed-geometry query admits the
remaining escape, a bridge to a reviewed waypoint at least 5 mm above the actual
vise hulls, and the remaining withdrawal. Both phases have continuous geometry
checks; only the selected stock/fixture pairs have command envelopes in the slow
phase, and all live contacts retain the 0.001 mm depth limit. Registration is model-only until real pose
sensing is commissioned, and cannot be exported as ordinary waypoints.

Withdrawal checks that the measured stopping posture lies on the reviewed search
corridor, reverses to its entry, and rejoins the existing open-jaw return. Before
moving, a fresh continuous collision query checks the measured released-stock pose
against every robot/tool hull and any retained payload along the return. Tool and
payload offsets use observed physical poses; the query verifies arm FK against
physical links and includes an explicit 0.12 mm point reserve. Positive and
synthetic overlapping query controls run in the yaw/clamp check without moving a
physical object. Live contact penetration remains limited to 0.001 mm.

```sh
python3 machinekit/examples/machine-tending/run_electric_simulation.py --clearance-first --locating-smoke a --locating-offset-check
python3 machinekit/examples/machine-tending/run_electric_simulation.py --clearance-first --locating-smoke b --locating-offset-check
python3 machinekit/examples/machine-tending/run_electric_simulation.py --clearance-first --vise-pickup-smoke b
python3 machinekit/examples/machine-tending/run_electric_simulation.py --clearance-first --tray-placement-smoke a
python3 machinekit/examples/machine-tending/run_electric_simulation.py --clearance-first --tray-placement-smoke b
python3 machinekit/examples/machine-tending/run_electric_simulation.py --clearance-first
```

The focused locating/tray checks seed the arm at a reviewed search entry and
stock on a temporary support. They establish a physical grasp, remove that support, then
exercise contact search, release, settling and withdrawal (plus clamping for the
vise). The offset and tray checks seed a part 0.82 mm low with about 0.002 rad
orientation error and verify that the nonideal grasp survives jaw closure. Their
reports identify the seeded setup: they do not qualify pickup, transport, a
companion payload or the complete sequence. The separate vise-pickup check
physically settles and clamps an offset blank, collects the companion through
its tray route, then checks observed registration and loaded extraction. It
excludes the primary blank's earlier loading and machining cycle.

A later full run passed the corrected B extraction and stopped during the third
load when the first light stop contact recoiled during position hold. The
preserved report is `build/electric-physics-third-load-locating-baseline.json`.
The horizontal touch-and-rest check addresses that distinction; it does not
relax support-load, force, speed or penetration checks. The focused A regression
using that low, tilted grasp passes locating, release, supported settling,
withdrawal and the final full-pose clamp check, even after the contact load
recoils to zero.

The open-vise support check constrains the stock's bottom normal to 0.001 rad and
requires positive load on both parallels, low velocity and the existing position
tolerance. Yaw is observed and then aligned by the jaws; the post-clamp check
retains the full 0.001 rad orientation tolerance. Tray placement retains its
full-pose check because no subsequent alignment occurs.
`--clearance-first --seat-yaw-clamp-check` separately verifies that a supported,
level blank with 0.0025 rad yaw reaches that full-pose tolerance after clamping.
Both companion-loaded vise-pickup checks pass, including the full return to
park. Their peak carried-point separation speeds are 0.0502 mm/s (A) and
0.0712 mm/s (B), below the 1 mm/s limit. Continuous observed-geometry checks
cover 16 guarded edges on either side and 116/106 free return edges for A/B.
Both focused tray support-search checks also pass at the final 0.05 mm/s
command speed. Physical validation of the complete strategy remains pending
until the full four-part cycle passes.

The headless host and application share `SimulatedContactSurfaces` for hard-fixture
contact compliance. The electric host's metal finger friction remains an explicit
assumption. CAD jaw openings are converted using the imported CAD default as the
runtime origin; 24 jaw/finger frame checks cover both sides at 0, 17 and 25 mm.
Empty companion jaws park at 17 mm, and only the active empty jaw opens to 25 mm,
matching the approach states used for certification.

Each certified joint edge is submitted to `RobotRuntime` as a synchronized C2
quintic polynomial. A common scalar preserves that edge's geometry; duration is
bounded by the model's joint rates and accelerations and the simulation's 0.5
rad/s, 0.5 rad/s² caps. Native CSP interpolation and inertia feedforward run at
physics substeps. Gravity bias includes physically held stock and resets with the
simulation. Jerk qualification remains pending. Exact-stop admission requires
measured low velocity and five settled samples, as well as joint position.
The vertical-axis physics check verifies trajectory tracking, effort-cap rejection,
bias reset and a controlled hold that stops before the original goal.

On failure, the host consumes the controlled hold and measures whether the arm
settles. It records contacts and joints at the fault separately from the final
stop observation. No failed operation is automatically retried.

Pickup departure always begins with bounded separation from the named support
faces, even when the geometric path has no penetration. These contacts must clear
before ordinary transfer. `--withdraw-smoke` starts at the reviewed first pickup,
confirms an actual grasp and runs withdrawal to park; its independent report is
`build/electric-physics-withdrawal-smoke.json`. It does not certify the full cycle.
Arm plans command only the six arm joints. Their uncommanded reference metadata
uses the ideal jaw coupling; actual jaw state and grasp feedback stay measured.

Placement positioning allows the named support contacts without requiring the
supports to carry weight while the jaws still carry the stock. After confirmed
release, `profile.seating` requires measured force on both vise parallels (or all
four tray pads), the fixture datum within 1 mm and 0.001 rad, and ten settled
samples. At the vise, the load connector's lateral clearance is retained while
Z settles to the parallels; the final seat datum is checked after closure. Jaw
closure uses the authored preset grip coordinate (3 mm in this fixture), rather
than the cylinder's 6 mm end travel. The closed-jaw check does not require the
parallels to carry all the weight once jaw friction can share it. Clamping force
is not modeled by the prescribed fixture motion. Contacts of released stock
with its supports are separate from robot/carried-payload withdrawal contacts.
Unexpected contacts and the 0.001 mm penetration limit remain enforced during
motion. Ordinary waypoint playback rejects profiles needing seating feedback.

The clamp command finishes its bounded ramp before bilateral grasp confirmation
admits transport. Stopped-pose initialization includes process-owned and coupled
joints. The host selects implicit integration, elliptic friction, impedance ratio
10 and ten no-slip iterations with main and no-slip solver tolerances of 1e-12. These numerical
settings reduce artificial contact creep without changing the finger friction
coefficient. The world uses an explicit 128 MiB arena budget. Topology rebuilds
prepare kinematics/inertia without solving temporary unseeded poses; exhausted
contact/constraint storage fails a real physics tick rather than omitting contacts.
The generic SDK defaults and the world descriptor's 96-byte layout are preserved.

Dynamic stock uses CAD-derived mass, centre of mass and centroidal inertia in SI
units. The mass-property regression checks unit conversion, declared-mass scaling
and explicit inertia overrides. Collision shapes come from authored CAD geometry;
coordinate-only links do not acquire fallback bounding spheres.

`--seat-settling-check` drops a slightly tilted blank onto the actual vise parallels
and requires positive support loads, settled position and low linear/angular
velocity for ten samples. Its report is `build/electric-physics-seat-settling.json`.

`--placement-smoke` runs the canonical collection, door/unclamp, vise placement,
release, support confirmation, withdrawal and clamp sequence through its first
clamp. Its separate report is `build/electric-physics-placement-smoke.json`.
It does not certify cutting, unloading, delivery or the complete four-part cycle.
The focused MuJoCo regression passes pickup, release onto both parallels,
withdrawal and the authored 3 mm clamp target. Its final measured seat error is
about 0.0044 mm; this is a simulation result, not a hardware accuracy claim.
