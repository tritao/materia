# 3D printing plan

Materia designs, slices, simulates and executes a CoreXY print, and exports the same assembly/job to Klipper.
The next machine is a Cartesian pellet printer with print-then-trim capability. Units are SI internally.
This ledger records each item's design, current decision and useful acceptance numbers. Detailed experiments,
failed attempts and run chronology belong in commit messages; historical evidence remains in git history.
“Complete locally” does not imply published or landed.

## Execution and publication contract

- Continue in the DATA1 `3d-printing` worktree. Preserve unrelated work and do not modify the dirty main checkout.
- No push. Publish NativeKit `6168df3b8ab0b1937f87bdcc636b79744438cdad`, Haxeon
  `e32024b6d8ae02e51ed5c6cf4d67bb8cbd2bb638`, TOPP-RA `558a3404cfd5839ad3539d07e4b5cbd361812848`,
  then Materia. The TOPP-RA follow-up retains original forward controls atop cleaned `materia` `11a85f364`;
  Release and Debug tests pass. The retired `materia-tr2-free-end` branch receives no new commits.
  Future fork changes build on `materia`, with upstreamable subjects/bodies, and remain for the user to push.
- Scope is frozen: finish the checkpoint, commit all own work and stop/report. No PL1, new features or further
  changes to other examples' behaviour. Verify workspace without test cache, unsharded MachineKit, MotionKit
  native/Haxe, printing, every app project-source case, editor and full cube; report the branch's own commits.
- The branch is linearly rebased onto local main `73b480f4c0bd8419f7f4bec2c901062e7ee9dc78`. Preserve main's
  async simulation Stop, reference-arm, screwdriving and sheet-metal behaviour. Old merge/prototype history is
  consolidated; the reach certificate is regenerated, not hand-merged. Do not merge main or move local main.
- The previous PL0 smoke pass is historical evidence, not proof of the final checkpoint after current changes.

## Architecture

The MachineAssembly and print job own configuration; exporters derive their outputs from them. ToolpathKit
stores deposited bead area rather than a filament E coordinate. Filament diameter and screw calibration belong
to the machine/material. Extrusion is a distance-driven process joint written after geometric IK, not a pose axis.
Heater loops and faults run on the device; the simulator has an explicit thermal plant. Deposition uses the physical
nozzle into the same StockKit model used for CAD comparison and later milling. Moonraker uses documented network
interfaces; no Klipper/Moonraker source is copied. Generic assumed mechanics remain labelled as assumptions.

## Core printer (AM)

- **AM0 / AM0b — complete locally.** Isolated worktree, source/build separation and baseline diagnosis.
  Changes unrelated to the printer remain separately identifiable; never infer success from an old main baseline.
- **AM1 — complete locally.** Modal deposition, filament moves, heat, fan, layer, feature and object markers in
  the toolpath IR. Preserve provenance and safe process states through lowering/restart.
- **AM2 — complete locally.** Marlin/Klipper parsing and writing, absolute/relative extrusion, G92, temperature
  waits, fan, retraction and markers. LinuxCNC behavior remains covered by its existing tests.
- **AM3 — complete locally.** Slice exact CAD planar sections into perimeters, skins, rectilinear infill,
  travel and retraction; holes/islands remain geometric. Mesh import/supports are later work.
- **AM4 — complete locally.** Printing benchmark records slicing, lowering, planned time, CPU/wall time,
  segment counts and memory, with a look-ahead estimate. Performance claims require matched measurements.
- **AM5 — complete locally.** Short/curved/non-planar corner handling and bounded numerical timing windows
  carry attained speed across interfaces. Current gate: cube 870.1 s plain / 881.7 s shaped, 35,216 / 55,847
  segments; gear 1047.9 s / 165 sections; zero boundary lowerings. These are pre-TR3 acceptance numbers.
- **AM5b — complete locally.** Three TOPP-RA rounding fixes (upstream #302), micrometre-path timing,
  motor-derived axis rates and reduced planning/simulation overhead. Earlier matched runs found planning
  4.6–7.3 times cheaper and the full cube about 14 times faster than real time; these are historical measurements.
- **AM6 — complete locally.** Real CoreXY assembly, Z support, extrusion, hotend, bed, fans, homing and probe.
  Refuse non-rotation placement frames; ground placed members and check post/bed-wing clearance with `printer-frame`.
- **AM7 — complete locally.** Device heater control, cold-extrusion guard, watchdog/runaway faults and
  explicit simulated plant. Heater tuning, sensor and ADC/power data are required, not inherited defaults.
- **AM8 — complete locally.** Plan the extruder from deposited path distance and its flow/drive limits;
  retraction/prime, pressure advance and device execution are included. Physical simulation uses the same program.
- **AM9 / AM9b — complete locally.** Additive stock and actual-nozzle deposition, CAD volume/extent/hole
  comparison, app job/player/panel and faster simulation. Keep physical checks distinct from deterministic replay.
- **AM10 — complete locally.** Generate Klipper config and G-code from the model, including transmission,
  direction, resolution, heater and shaper data. Unknown hardware information must prevent hardware export.
- **AM11 — complete locally.** Moonraker upload/start/status and live twin, with shared event-pump networking.
  Preserve absent status values, lifecycle/fault behavior and IPv6; nested dependency readiness is below.
- **AM12 — complete locally.** ZV/MZV/EI shaping, including lag/tail, process-event time mapping and boundary
  certification. MuJoCo belt dynamics ring near 52 Hz rather than the former erroneous 5 Hz; gate checks physical
  attenuation and model/tuning agreement. Shaping consumes its share of the total path-deviation budget.

## Review follow-ups (RF)

- **RF1 — complete locally.** RKD6 revision 16 represents E as a float step base plus a local residual;
  exact-base domain is 2^53 steps, with explicit soft-f64 cost on Cortex-M4F. No integer-wire claim.
  Start-of-print tick reduced from 538 to 23 ms. Pre/post-shape merges share one deviation budget.
- **RF2 — complete locally.** Every nonzero coupling stiffness is compliant, derived from transmission
  geometry/material with assumption provenance; zero alone means rigid. Shared spring model drives simulation,
  planning, observation and shaping. Mill compliance attribution and independent prediction are under RF12; rapid
  compliance clearance is RF14.
- **RF3 — complete locally; retain the dynamic stretch reserve.** Deposit from the physical nozzle, with
  SmallTab and short cube in MuJoCo. Pulley/carriage deviation is distinct from motor hold and bed sag.
  `STRETCH_MARGIN=1.1` is still empirical, not a certified error bound. RF12(1) removes its old justification
  (constant stiffness at one pose), but does not establish the existing single-step overshoot formula for repeated
  acceleration reversals, coupled modes or finite-step replay. Removing 1.1 requires a forcing-response derivation
  or a demonstrated bound for these workloads, rather than silently shrinking the tolerance. RF14 already uses
  an absolute-impulse-response bound for arbitrary bounded forcing; assess that formulation here, accounting for
  position-varying stiffness. Recorded RF15 SmallTab: 835 um deviation / 919 um permitted, 118.1 mm^3 deposited.
- **RF4 — implemented locally; dependency publication remains user-controlled.** NativeKit HTTP and WebSocket
  message/text mode replace a second networking implementation; share the pump and retain cancellation/ownership.
  NativeKit origin is `https://github.com/tritao/nativekit.git`. Pin `6168df3b` passes CI-pinned ClangFormat 18.1.8
  and the 31 runnable Linux CTests, including transport/PTY contracts; privileged joystick input is skipped.
  Haxeon `e32024b6` adds four printing commits on local main `1434bcea`: message transports, repository
  formatting, the NativeKit repair pin and standalone module identities. Format, compiler/runtime, integration,
  Wasm and fixed-point self-hosting gates pass at this exact pin. `printing-without-rf13` must fast-forward local
  Haxeon main at publication, then be deleted. The shared base is local-only: `6464b8c9` was 87 commits ahead
  of origin/main; `1434bcea` adds its formatter repair. Publish pins in the order above.
- **RF5 — complete locally.** One coupling projection and separate `PathSampler`, `TimedMotionTransforms`,
  `PlanCertification`; distance-drive columns follow IK for every solver. Preserve refinement, one-sided curvature,
  process discontinuities, entry/retreat E, clearance-proof invalidation and held process state in transitions.
  Full MotionKit suite passed 1,329,624 assertions at completion; printer gate passed.
- **RF6 — complete locally.** Complete controllable intervals and the reachable forward pass determine actual
  interface speeds, including nonzero lower bounds. No fixed share of an optimistic upper bound. TR2 supplies
  free-end forward solving; TR1's failed interpolation experiment does not weaken continuous validation.
- **RF7 — complete locally.** One polynomial merge in trajectory_core, typed failure causes and original-row
  retry only for merge-attributable failures. Timing/path/link budgets are derived; clock and unrelated-limit
  failures cannot be hidden by retry. Keep accumulated shaping/merge error inside the authored path budget.
- **RF8 — complete locally.** Drive-resolution admission at startup and along paths; device/simulation use
  matching nearest-step rounding. Physical re-baseline and full motion suite passed; no arbitrary homing epsilon.
- **RF9 — complete locally.** Export Klipper's worst-direction Cartesian acceleration from the coupled axes.
  Existing printer max_accel did not change because its own axis limits are tighter.
- **RF10 — complete locally except the explicitly retained TR1 reserve.** Required HeaterTuning and one policy,
  model-owned thermistor front end, pressure-advance reserves from drive limits/K, turn regression, affine adaptive
  grid and continuous path-budget merges replace unexplained constants. Clock representability belongs to device
  conversion, without a clock-sliver heuristic. Heater defaults cannot stand in for unknown installation data.
- **RF11 — complete locally.** Profile-owned purge prime, one BeadSection, correct startup retraction,
  material specification, scene/job adapter, startup feed hold and kind-based peripheral input. Preserve absent
  Moonraker status fields and issue heater faults once. Generated build files are excluded from history.
- **RF12 — complete locally.** Position-dependent belt/screw stiffness with certified travel minima and backend
  updates; nut/bearings/shaft in series; energy-based settling; measured resonance acquisition/calibration.
  Damping ratio remains the authored quantity: hysteretic loss gives zeta=eta/2 without load knowledge.
  SFU1605/400 mm example: 66.4 N/um shaft alone, about 43 N/um with nut/support bearings; assumed catalogue
  stiffnesses remain labelled. Shaping must cover the travel-dependent mode range. Kinematic fan-out relations
  omit physical constitutive laws, consistently with constant stiffness; a native regression covers gantry setup.
  Mill X/Y/Z tracking versus fresh main: .03956/.10933/.06839 versus .02384/.01478/.05993 mm. Main's unstated
  damping made screws ideal; RF2/RF12 exposes their compliant response with declared zeta=.03. Independent CAD/
  command-only two-body prediction: .04112/.09816/.06240 mm, roughly 4–10% gap; Y peaks at the measured 51.00 s.
  Native ideal-screw counterfactual: .03593/.01933/.06492 mm, so compliance adds 3.6/90.0/3.5 um: it dominates Y,
  with smaller contributions on X/Z. The remaining 12.1/4.6/5.0 um against main accompanies the changed certified/
  clearance trajectory (118 versus 78 plans);
  no gains, tolerance or example changed. RF15 replaces the unit-incorrect solver-cost allowance. RF3's dynamic-error
  derivation remains separately open.
- **RF13 — moved to follow-up branch `rf13-motor-holds`.** Physical motor holds, absolute joint referencing,
  pulse counters and takeover/brake changes are excluded from this landing. Quantized positions overwrote analytic
  hold-reference endpoints while retaining unquantized velocities: preserve one consistent plan sample for position,
  velocity and acceleration; round only emitted/reported steps. Carriage-to-arm inertial feedforward regression:
  predicted/measured missing response 50.15900180/50.15900174 urad; corrected response effectively zero.
  Converse reaction-load check: predicted/measured carriage displacement 139.693387817 um. These native checks
  do not explain remaining app errors: track seam 1.631 mm, overhead path 2.319 mm; gantry rejects a start
  1.374 um below its X limit. No gains, example limits or tolerances were tuned. Mill servo policy remains a user
  decision: derive gains from reflected inertia at a declared bandwidth, author gains on the example, or retain
  main's ServoDrive defaults. Generic fallback at 10 Hz predicts roughly 6.75/9.85/9.63 mm X/Y/Z lag; this is not
  a reason to change the landing's existing servo policy. The unchanged 1 um no-creep assertion moves with RF13:
  the generic law has timestep-dependent bed creep (fresh main -16.23 um at 2.5 ms; landing with soft impedance
  -14.19 um at 2.5 ms; max-impedance diagnostic +9.89 um at 10 ms). Landing retains the sag diagnostic and RF3 constant-offset assertions. The settled
  reference hides a constant physical bed offset: raw commanded-versus-physical Z is 175.84 um low at 10 ms,
  and 196.48 um low after 100 s at 2.5 ms (210.67 um at 1 s). RF13 must resolve that offset and the generic law's
  timestep sensitivity. Maximum ideal-transmission
  impedance and the missing follower-load correction must land together on RF13. Acceptance includes the housing
  fourth flange screw (soft impedance completes in 185.66 s versus main 185.64 s; maximum impedance times out)
  and the split free-rundown/frictionless/frictional screw oracle. The original screw oracle passes with the landing's
  soft impedance; both the impedance change and its corrected oracle/regression move to the follow-up.
- **RF14 — complete locally.** Fixed-stock CNC rapids reserve cutter envelope plus absolute compliance response,
  backlash and one step; motor-hold displacement is deferred with RF13. Close portions become exact-stop feed moves; risky travel without
  a cutting feed refuses. Damped bounded-force gain is coth(pi*zeta/(2*sqrt(1-zeta^2))) below critical damping,
  otherwise 1; undamped compliance refuses a finite bound. This fixed-stock pass does not certify moving blanks.
  Belt/screw routers: zero rapid contacts/holder collisions/stalls; 297.2/315.8 s machining, roughly 96 s extra each.
  Belt displacement reservation [8218,11311,54] um; screw [54,54,54] um. Accuracy allowances were not borrowed
  from a leftover-stock percentage. Bench mill: 69.0 s, 3866.3 mm^3 removed / 3.5 leftover / 0.3 gouge.
- **RF15 — complete locally.** Observation precision is relative position representation roundoff, not MuJoCo's
  dimensionless solver cost tolerance. MuJoCo reports eight mjtNum epsilons (1.776e-15 for double), scaled at the
  runtime boundary by max(1,abs(limit)); rigid core reports zero. This does not bound physical soft-stop compression
  or integration error. Rated-effort stop tests still fault without modeled allowance; tests cover both signs
  and coordinate scales 1e-3, 1, 1e3. Backend/runtime/MuJoCo suites passed at completion.

## Next after landing

Simulation fidelity levels ([SF0–SF6](../../../simkit/plans/FIDELITY.md)) -> PL1–PL12, with TR6 alongside
PL2/PL11. Resume TR1 whenever planner work resumes. Later features remain parked unless the user pulls
stock-printer features such as bed mesh or pause/runout forward.
[Online motion](../../../motionkit/plans/ONLINE_MOTION.md) is independent and may run in parallel with FIDELITY
and PL: Ruckig for free/reactive motion; path stops stay on TOPP-RA/TR3; jerk on paths remains research.

## Remaining planner work (TR)

- **TR1 — evaluated; adoption deferred.** Keep collocation and `interpolation_margin=0.8` for nonlinear paths
  (affine paths use 1). The 0.8 factor is an empirical 20% reserve on the acceleration constraints, also used in
  adaptive-grid selection, originating in the curved Paint regression. It is not a universal interpolation-error
  theorem. Polynomial lowering and strict continuous validation against the caller's original limits bound accepted
  results; a failed check is refused or retimed according to the endpoint contract. Interpolation without reserve
  failed the 135-degree / 50 um corner: 12.5106619431 versus 12.5 m/s^2, with zero adjustments required.
  Removing or reducing 0.8 needs a continuous error bound/refinement and Paint/corner/printing acceptance, not
  endpoint checks alone. Preserve this reserve while finishing TR3.
  Payoff: up to ~20% higher cornering acceleration; removal needs a continuous interpolation-error bound with
  refinement plus Paint/135-degree corner/printing acceptance.
- **TR2 — complete locally.** Free-end forward solving returns the actual reached speed, not the interval upper
  endpoint. API is in cleaned fork `11a85f364`; future upstream submission/publication remains user-controlled.
  Upstream hungpham2511/toppra#302 (degenerate stages) is open; repoint the submodule upstream when it merges.
  Submitting TR2 upstream is the user's decision.
- **TR3 — adopted locally; publication remains user-controlled.** One compiler produces owned timed windows in order;
  whole-program timing drains the same stream. Lazy exact-stop sections and indexed drive slicing permit early
  execution. Publication preserves pressure advance, shaping, moving-cut position/velocity/acceleration checks,
  source/event maps and one final tail. Two executors queue a moving successor and retire consumed state.
  Authored stops settle; asynchronous starvation stops safely. Feed override/runtime retiming remain deferred.
  Safe/optimistic native profiles compare original LP controls as well as squared speeds; closed input needs only
  its actual terminal solve. Revision tokens reject stale/foreign/double retirement. The integer clock retains
  controls and fractional tails; representation refinement never changes its stages or physical tolerances.
  Bernstein interval velocity/acceleration and both curvature-join sides certify the strict constraints. Eligible
  stopped affine moves use a certified chord and integer-clock triangle/trapezoid. Keep the shared solver leased
  across adjacent passes and prove the default geometric merge budget once per immutable section; custom factories
  retain their per-chunk contract. Each pending native window owns its immutable interval table until retirement:
  solver switching reuses it, without the former two-entry LRU's repeated proofs. Retention follows the existing
  section sample/pending-window budget, not total program duration. Resident CAD/program input is counted separately.
  Matched CPU-5 gear/cube planning: exact b8 native baseline 12.97/10.04 s; strict whole-section single window
  (no prefix proofs/publication cuts) 19.06/13.83 s; current streaming whole drain 21.03/14.20 s. Thus strict
  certification contributes 6.09/3.79 s, with streaming adding 1.97/.37 s (10.3/2.7%), rather than attributing
  the entire historical increase to streaming. The earlier 2.2% compared two drains that both paid streaming.
  Bounded gear costs 22.91 s in 886 publications (20.2% over strict single-window, 8.9% over the whole
  drain), with 569212 KiB RSS and the same clock; its additional cost is publication/filter work.
  Gear backward CPU: b8 .755 s, strict single window 6.533 s, streaming 7.673 s; interval preparation nested
  within backward/forward is 3.444/4.499 s for strict/streaming. Streaming proof construction itself is 3.43 s;
  initial retention costs .36 s and solver-rebind copies .71 s. The previous LRU built 228 tables: window-owned
  tables build 175, once per immutable window; switching passes copies results, and a genuinely new constrained
  problem still needs new proofs. Matched streaming gear improves 22.70 -> 21.03 s; all 602 per-plan Float and
  integer clocks remain bit-identical. Whole gear/cube peak RSS is 550916/476652 KiB; the diagnostic strict
  whole-window gear needs 2047400 KiB, so it is a cost baseline, not a replacement for bounded windows.
  Short planning entry points at b8/current: six-axis jog median .124/.123 ms, stored three-waypoint tending
  transfer timing 1.007/.989 ms (200 samples each, CPU 5, same Haxe sources). The transfer uses its certified
  route and a free-space oracle: these numbers exclude collision search and runtime submission. Motion clocks
  are identical; neither entry point uses the path-stream machinery.
  Checkpoint CPU-3 bounded/whole CPU: cube 15.36/13.86 s, gear 22.51/20.51 s. Matched 1x/4x/8x cubes
  reach first motion in 11.1/12.4/10.9 ms with peak process RSS 512764/524000/552144 KiB. Bounded/whole clocks agree:
  cube 874355978950 ns and six-layer gear 808918423491 ns; the strict single-window baseline agrees too.
  Matched 1x A/B against pre-split `08d61e739` (four alternating pairs, identical harness/compiler, matching native providers)
  separates executor submission, runtime admission and observed movement. Median CPU/wall ms, earlier -> landing:
  submission 3.79/4.85 -> 3.69/4.76; admission 6.40/7.45 -> 6.21/7.28; observed movement 6.40/7.45 -> 8.78/10.92.
  First-command latency is the planner acceptance metric and has not regressed. RF13's observer/counter behavior
  reports movement at clock zero; landing reports it at the next 10 ms tick. Process CPU during the extra owner step also counts concurrent
  planning; first-command certification has not grown. This deterministic-backend trace does not prove servo lag.
  Keep both submission/admission and movement fields in the harness; no planner or tolerance change is warranted.
  Earlier merge/lowering work preserved all 541 historical plan clocks and 655 reports: exact-degree polynomial
  operations, absolute-sum error enclosures with the original exact fallback, on-demand candidate rejection and
  reused lowering compositions. Matched gear/cube CPU improved 13.73/10.89 -> 12.30/9.64 s before stricter
  certification; those older clocks differ from TR3 because the physical constraints were subsequently corrected.
  The table-ownership change passes all 21 native tests and the printing gate. Checkpoint coverage includes
  workspace no-cache, unsharded MachineKit, MotionKit native/Haxe, printing, all app cases, editor and full cube.
  Tending's 75% target also fails main (73.9768%; landing 73.9872%); no tending behaviour changes belong here.
  Force search is 7.49589 N versus fresh main 7.49383 N, with the 25 N bound unchanged.
  Nine mobile-welder selectors also fail fresh main: three probe-clearance/IK errors are byte-identical;
  registration fits leave retained zero Z/roll/pitch bounds (cutoff Z: main/landing 54.12/53.84 um; mission
  -29.71/-27.56 um; later energy probe 35.36733/35.36721 um); the pack mass/CAD-inertia check fails identically.
  These are inherited app-gate failures, not a green app gate or a reason to change examples/tolerances here.
- **TR4 — removed.** Optional parallel window preparation showed no repeatable gain: matched gear elapsed
  serial 21.171 s, one helper 17.212 s, two helpers 20.871 s; peak RSS 631444/722380/736220 KiB.
  Keep one retained-window timing implementation and cooperative single-threaded execution. Remove the pool,
  its optional environment switch and duplicate worker-mode runs; retain the serial path/retained-window corpus.
- **TR5 — complete locally.** Versioned degenerate-stage replay/generated corpus and shared Seidel workspace
  reuse across problems. Matched full-print allocation comparison found about 70% fewer allocations; this does
  not imply a timing/RSS gain or allocation-free initialization of differently shaped problems.
- **TR6 — planned with PL2/PL11.** Torque constraints from inverse dynamics for the heavy gantry, including
  moving mass coupling and gravity on Z; linear axis acceleration rows alone do not establish motor feasibility.

## Pellet printer

PL0 is complete locally; PL1–PL12 have not been implemented. Do not start PL1 before the requested verified
checkpoint and user report. Keep assumed drive/thermal/calibration data explicit; no hardware-interface claim.
Previous PL0 full unsharded MachineKit smoke: exit 0, all 85 named groups completed, including P1–P5 press brake,
routers, CoreXY, printer/Klipper, tending and recipes. Evidence: `build/pl0-corexy-fixed-machinekit.log` and
`build/pl0-corexy-fixed-coverage.json`. This pass predates current TR3 changes.

**PL0 reference facts and assumptions.**
- Rechecked the [manufacturer catalogue](https://www.iemai3d.com/index.php/pellet-3d-extruders/) and its
  [P10 kit parameter sheet](https://www.iemai3d.com/wp-content/uploads/2026/09/Pellet-Ext-P10-ExtruderControllerHopper-DryerCompressed-Air-Source-Components.pdf)
  (PDF metadata dated 2026-09-10). The dryer mass is 40 kg; 25 kg is capacity. Per-zone heater loads and the controller
  command protocol/pinout remain unspecified. Its three heater-controller ratings do not establish actual heater
  loads. The 950 W controller and 2.1 kW head totals are not a zone-power allocation.
- The listed controller needs more than 450 L/min inlet air, while the kit compressor lists 180 L/min. Rating conditions
  and duty cycle are not given; do not assume the pair meets continuous demand. PL10 needs explicit air-supply data.
- [The robot-system P10 sheet](https://www.iemai3d.com/wp-content/uploads/2026/09/Pellet-P10-KUKA210-TDS.pdf)
  qualifies maximum throughput for high-speed PLA with a 10 mm nozzle. The derived 2.78 g/screw-turn value is a
  reference-condition estimate, not a calibrated material/nozzle flow law. Screw-flow parameters remain PL3 inputs.
- **Recorded simulation assumption:** command screw angle/speed through an abstract simulated servo. No step/dir,
  analog or fieldbus interface is claimed for the vendor controller; electrical export needs a verified drive profile.
  Zone powers, sensors, drive resolution and flow calibration remain explicit unknowns rather than heater defaults.
  No vendor contact was sent. [Support-material terms](https://www.iemai3d.com/index.php/support/) request authorization;
  no reusable CAD grant is established here, and vendor CAD/PDF files are not added to project history.
- **Ideas-only references:** [ORNL/SPR-2022/2669](https://info.ornl.gov/sites/publications/Files/Pub186519.pdf)
  studies temperature-based layer timing and exponential cooling fitted for CF/PC. This is a technical report with
  government-report notices, not an imported software dependency; its fitted material constants are not defaults for
  our machine. [CuraEngine's timing discussion](https://github.com/Ultimaker/CuraEngine/wiki/Inserts) covers including
  minimum-layer waits in planned time; its [engine source licence](https://raw.githubusercontent.com/Ultimaker/CuraEngine/main/LICENSE)
  is AGPLv3. No code or assets are copied from either reference. Our CAD slicer and thermal model remain our own.

**PL1 design and acceptance constraints.**
- Build the head as an includable `MachineAssembly`, following the existing assembly composition API rather than
  hiding the motor/reducer/screw in a single rigid component. Expose the carriage mount and nozzle tool frame;
  keep the screw's process joint distinct from the motor's shaft joint. The shaft joint must be supported by
  its own `ServoMotor` member (RF13), with the 50:1 reduction represented exactly once.
- `AssemblyInventory.massProperties` sums member masses. The 10 kg catalogue head mass is therefore a budget
  for the complete purchased head, not an extra 10 kg housing beside a separately counted motor, reducer and screw.
  Allocate positive member masses with their assumptions recorded; reject a configuration whose remaining housing
  budget is non-positive. Keep gantry adapters and the trim spindle outside that head budget. Test total assembled
  mass as well as individual parts. Centre of mass and inertia distribution remain assumptions, not vendor facts.
- Existing `Gearbox` has an envelope-derived declared mass and assumed input inertia; existing generic servos
  have assumed winding, encoder and torque-speed data. Reusing these requires an explicit mass allocation and
  assumption provenance, not relabelling a generic model as a verified P10 drive. The catalogue's 1.91 Nm maximum
  is not evidence for continuous torque, and 60 rpm at the screw with 50:1 reduction requires 3000 rpm at the motor
  but does not establish its full torque-speed curve. Preserve these distinctions in the executable model.
- Keep the three barrel zones independently addressable. Do not substitute the filament printer's 40 W cartridge
  defaults or interpret the controller's channel ratings as installed heater power. Simulation thermal parameters
  and screw mass-per-turn calibration must be explicit assumptions/inputs; hardware export needs a verified drive
  and I/O profile. Optional cached vendor display geometry must not determine kinematics, mass or tool frames.
- Acceptance must exercise assembly inclusion, motor-shaft ownership, reduction direction/rate, the complete-head
  mass budget, nozzle/tool-frame placement, recipe reconstruction and independently addressed zones. These checks
  supplement the full MachineKit baseline; they do not replace it. PL1 starts only after the requested checkpoint.


**Decisions.**
- PL-D1. Cartesian gantry, not CoreXY. The head (10 kg plus a spindle) dominates the moving mass, belts are too soft over
  1 to 1.5 m, and milling needs stiff drives both ways. Y is a beam driven at both ends (two servos on racks, kept square:
  `GantrySpec.dualY` and its racking tolerance), X a carriage on the beam (rack and pinion or ballscrew), Z a ballscrew
  on the gantry. Servos throughout, with motor-derived limits.
- PL-D2. The screw is the extruder joint of AM-D3, driven by path distance through mass per screw turn, with its motor's
  limits through the reducer. Its flow responds over seconds (melt pressure), so the plan gets a screw-flow model in the
  simulation (lag, ooze, pressure) and a feed-forward term in the planner (pressure advance at this scale). Starts and
  stops are planned out where possible (continuous paths, coasting, wipes) because retraction barely works.
- PL-D3. The layer-time window is a planning constraint: a layer printed too fast slumps, too slow does not bond. A
  per-layer thermal model (bead size, length, convection, material) gives the window; the planner slows a short layer
  or the program waits. It is checked in simulation, not only asserted.
- PL-D4. Hybrid from the start: the slicer prints oversize by a trim allowance, the CAM trims the printed stock to CAD
  with the router's milling stack, on the same dexel stock the deposition fills.
- PL-D5. The extruder controller is a device peripheral: our device drives the screw (step and direction to its servo
  driver, if the vendor confirms) and the heater zones' targets; the vendor box's own heater loops are modelled, not
  replaced, unless we take over the heaters (then AM7's heater module runs three zones).
- PL-D6. Cell equipment (dryer, compressor, conveying) is ProcessKit sequencing and scene geometry, not motion.

**Test parts.** A tall thin-walled box 600 x 300 x 400 mm with a single bead wall (layer time and slump); a mould-pattern
segment printed oversize and trimmed (hybrid accuracy); a ribbed panel (sparse infill sized to the bead).


**Steps.**
- PL0. **Complete locally, 2026-10-16.** Survey and worktree: confirm the P10's drive interface and controller I/O with the vendor (or record an
  assumption); licences of the STEP files; references for large-format slicing and layer-time control (ideas only,
  licences checked); baseline suite.
- PL1. `PelletExtruder` part (`machinekit.printing`): screw, barrel zones, reducer, servo, feed throat, nozzle set, mount
  plate and tool frame, mass from the specification, assumed values marked; vendor STEP as optional display geometry
  from `materia-cache`. Controller, dryer and compressor as cell equipment parts.
- PL2. The machine: the gantry assembly (dual-driven Y on racks, X, Z ballscrew), vacuum table, the P10 and a trim
  spindle beside it, homing, a Start-page entry; motor-derived limits including the moving mass of the head.
- PL3. Screw-flow model: melt-pressure lag and ooze in the simulation (first order to begin with, parameters per
  material and nozzle, assumed), deposition rate from it, and a test that the deposited flow follows a step in screw
  speed with the modelled lag.
- PL4. Planning the screw: feed-forward on the screw joint from the flow model, start and stop handling (coast, wipe,
  prime), a check that the deposited bead width stays within tolerance along a path with speed changes.
- PL5. Thermal layer model: per-layer cooling from bead size, length and convection; the layer-time window per material
  as data; layer temperature at the next layer's start recorded per layer.
- PL6. Layer-time control in the planner: slow a layer that is too short (down to a floor), then wait; fail a program
  whose layers cannot meet the window. Test on the thin box: every layer in its window.
- PL7. Slicer for large beads (`camkit.additive`): single and double bead walls, continuous spiral and minimal-start
  paths, sparse infill sized to the bead, the trim allowance, layer-time awareness (reorders or merges regions).
- PL8. Deposition at this scale: bead footprints in the dexel stock at 5 to 15 mm beads, slump as a function of layer
  temperature from PL5 (a simple model), checks of volume, bounding box and wall thickness.
- PL9. Hybrid trim: CAM roughs and finishes the printed stock to CAD with the router's milling stack, the extruder
  parked or retracted, tool change to the spindle; check the trimmed part against CAD within tolerance.
- PL10. Cell: dryer and compressor as equipment, pellet conveying and hopper level as signals, the print-then-trim cycle
  in ProcessKit, heater and feed interlocks (a cold barrel never turns the screw, an empty hopper pauses).
- PL11. Input shaping and gantry dynamics: the gantry's structural modes in the simulation (beam bending, rack
  compliance), shaping tuned to them, residual vibration and the head's deviation checked at print speeds.
- PL12. End to end: the thin box and the mould segment printed, cooled, deposited and trimmed in the app; numbers
  recorded (print time, layer temperatures, slump, volume, trim time, accuracy).

**Later.** Robot-arm or track-arm deposition (non-planar), the L35 class with a melt pump and closed-loop melt pressure, an IR camera in the simulation, real hardware.

**Order.** PL0 -> PL1 -> PL2 -> (PL3 -> PL4) and (PL5 -> PL6) in parallel -> PL7 -> PL8 -> PL9 -> PL10 -> PL11 -> PL12.
PL8 and PL12 use Kinematic for full-size parts and Physical on short representative sections
([fidelity levels](../../../simkit/plans/FIDELITY.md)).


## Later features

Bed mesh/contact probing, independent Z tilt and skew correction; pause/filament change/runout/cancel-object/
power-loss recovery; STL/3MF and mesh slicing, supports, variable width and multimaterial; verified real-board
heater/TMC configuration; robot-arm/non-planar deposition, WAAM and larger melt-pump pellet systems. These remain
explicit future scope, separate from the checkpoint and PL0–PL12 acceptance.

Model-based deflection compensation for motor-encoder arms requires declared drive/reducer data and calibration;
it is later feature work, outside this branch.
