# SR7 ATC sequence evidence

The native fixture includes all eight holders, collets and cutters, and transfers
each holder root between its own pocket and the spindle. Nominal tool data are
in `tool-table.json` (tool/pocket, diameter, gauge length, nominal holder/cutter
keys and completed-job life count); measured gauge-to-tip lengths and pocket ownership are saved
by `SheetRouterAtcState`. The library updates the CNC H table in metres before
acknowledgement and before compiling the next G43 segment. An interrupted change
requires physical inventory recovery; saved state never resumes a release.
A fresh exchange starts inside the work Y range (including park), not from
an arbitrary service-bay pose. `actuatorCommandsEnabled()` gates cover/shoe
commands: construction or checkpoint loading never arms those targets. Fault
release/spindle OFF and axis-abort outputs remain mandatory even when that
gate is false. The host must tick fresh observations before applying commands.

## Geometry and motion

Populating the old rack exposed collisions hidden by the bare assembly: a
neighbouring holder hit the Z ram at 110 mm pitch; at 150 mm pitch the offset
Z screw hit the neighbouring fork. The selected eight-pocket pitch is 220 mm,
with pocket centres X=30..1570 mm. The rack datum is 280 mm beyond the rear work limit; pocket seats are at
Y=workY+160 mm. The setter is ahead of the rack at Y=workY+60 mm, in the
front approach lane, with 240 mm of extra Y travel remaining. The 1760 mm hollow rack case overhangs the
unchanged 880 mm support plinth. The cover is a 1780 x 300 mm steel pan with a
3 mm web and 30 mm upward edge returns. The structural frame is unchanged.

The nominal PA12 U forks are top-loading: a 40 mm nose passes through the
41 mm opening and the 50 mm flange rests on the fork. Return and pickup move
vertically through a 55 mm clearance, then retract fully to Z=0 before any
cross-pocket traverse. Before changing X, retreat straight to the front
approach lane: the offset Z screw is fixed to the carriage and a diagonal
crossing hit a parked holder even at Z=0 and 220 mm pitch. Pickup likewise
raises fully, then leaves along its own fork approach. The setter is on this
front lane; its former position was obstructed by the widened hollow rack case. Purchased forks must permit this path. The nominal closed-cover clearance above the modelled 45 mm taper is 7 mm
(about 6 mm after the pan self-weight deflection estimate). The existing holder
recipe does **not** include the retention knob; the purchased spindle/holder/knob
envelope must be added and swept, and the cover raised if required, before
hardware release. This packet does not qualify an unspecified pull stud.

All eight docking positions,
five filled-rack cover positions and T1 -> T3 -> T1 are checked on both beds.
The fixed Y-chain mounting arm is below the preload unit, with its post
400 mm outboard of the rail datum, clearing the motor swept envelope. Five
additional mid-travel poses per bed guard this previously unsampled crossing.
Nine full-Z cutting poses per bed also check the closed cover: the original
position hit the ram/Z rails at the rear work edge. Moving the rack
20 mm rearward and the cover another 20 mm relative to the rack clears that
envelope without shortening the working bed.
Collision samples include endpoints and ownership transfers; this is a discrete
sweep, not a continuous swept-solid proof.

Axis trajectories use the retained native Ruckig sampler: 3 m/s², 20 m/s³,
40 m/min XY rapids and 10 m/min Z. Docking is 1000 mm/min and probing 1 mm/s.
The reference runner parameterises a coordinated line by its largest axis
displacement; rates cap axis components. SR16 must preserve or recheck that
convention in the host adapter. An independent seven-phase estimate checks
move duration. The setter latches
position at the fresh trigger edge, then completes a jerk-limited stop before
retracting; the stopped position is not the measured tool length. The fixture
uses preset 120 mm tools within +/-0.5 mm; shorter tools cannot reach the same
work envelope and must not silently inherit that reach.

## Interlocks and controller boundary

Release requires independent VFD zero-speed **and** spindle-stopped confirmations,
local pressure >=600 kPa, fresh input timestamps and an exclusive axis lease.
Every observation wait has a named timeout. Clamp/release conflicts, stale
motion tokens, occupied return pockets, missing tools, failed probing, lost
lease and persistence failure hold motion and suppress acknowledgement. Fault
outputs disable spindle-run and release and require explicit recovery. Completion
rechecks clamp, tool presence and inventory with the current request token.

`SheetRouterAtc` provides the sequence policy and `SheetRouterAtcSimulation` the
native assembly/plant runner. The robot `ToolChangeTransaction` is not reused:
its robot runtime bindings differ; the fresh-confirmation and ownership pattern
is retained. Operator changes remain available to machines without this ATC.
The host increments life counts only after a completed cutting job; the ATC
exchange itself does not consume tool life. Catalogue keys remain nominal until
SR13 selects the cutters and spindle-specific retention knobs.
The actual app/dispatcher adapter and RKD6 IO binding belong to SR16. Therefore
this packet does **not** claim the PLAN's in-app exchange acceptance or live
hardware acceptance. JSON rename is atomic in the normal host process; power-loss
fsync qualification is still required for hardware persistence.

## Pneumatics and hardware release

`cylinder-sizing.json` records force, air consumption and stiffness assumptions.
The 25 x 600 mm cylinder drives a separate guided slide; its rod carries axial
load only. Guide hardware is **not** modelled. Catalogue guide capacity, actual
support/bracket envelopes, bend radii, cylinder cushions and installed manifold
flow must be selected and swept before hardware release. Nominal simulated
spindle-stop (0.5 s), drawbar (0.15 s) and cover delays are not measured hardware
stopping times. Independent stopped sensing must be commissioned.

## Reproduction and retained evidence

Run `bash machinekit/examples/sheet-router/atc/check.sh` from the repository root.
It uses the prebuilt native providers, heavy-run and a private pinned planner
runtime. No OCCT rebuild is needed. Independent native runs can select `--bed2600` or
`--bed3150`; each saves its own sequence report after every completed exchange
and reports PASS only after both changes. For the focused mass/modal/weld recheck, run
`bash machinekit/examples/sheet-router/atc/check-structure.sh <inputs>`, where
`<inputs>` is the archive's `structural-inputs` directory containing the retained
`y2600.json` and `y3150.json` SR6 manifests. The sampler is launched without the host
LD_LIBRARY_PATH to prevent loading an incompatible MotionKit ABI.

`build/sheet-router-evidence/sr7-atc.tar.gz` retains reports, native logs,
planner/runtime dependencies, mass exports, the focused joint-model structural
recheck and source inputs. Its member manifest and archive index record hashes.
The earlier SR6 archive remains immutable evidence for the released weld model;
The 3 kg guide/adaptor allowance must be replaced if selected hardware exceeds
it. Local pan/guide modes are not resolved by these mass-only auxiliary bodies;
include the installed closed cover in commissioning measurements.
This check changes only fixed ATC/bracket masses and the cover position, at the two
previously governing poses. Commissioning SR6-C remains mandatory.

## Focused structural result

[Structural validation](structural-validation.json) retains deck, layout, source
and mass-export hashes. These are the governing **dynamic** poses, not a new
full static pose envelope. The unchanged SR6 full-pose stiffness envelope remains
the structural reference. Fixed mass increases 30.72 kg; moving cutting mass
is unchanged.

| Bed Y | Pose X/Y/Z (mm) | X/Y stiffness (N/µm) | First mode | Contour / residual | Dynamic weld range |
|---|---|---|---|---|---|
| 2600 | 800/0/200 | 2.240/2.199 | 27.416 Hz | 0.01275/0.01301 mm | 9.048 MPa |
| 3150 | 800/3150/200 | 2.254/2.199 | 26.979 Hz | 0.02011/0.01280 mm | 9.068 MPa |

Both pass the 0.05 mm motion budgets and the 12.669 MPa weld cut-off used in
SR6. Maximum combined design stresses are 44.01/44.60 MPa against 207.85 MPa.
The fixed setter and pedestal also have separating CAD bounds from the rack case
and all eight forks; this supplements the moving-body collision checker, which
intentionally excludes fixed/fixed contacts.

## Native sequence result

[Validation](validation.json): 42 policy assertions, 83 static poses per bed
(including full-Z closed-cover cutting and mid-travel bracket positions), and
four complete native exchanges with 28 controller/measurement/restart checks.
Ownership transfers have zero tool-body position jump. Final recovery/start
guards were checked in the final policy run; they leave the validated active
work-area sequence unchanged.

| Bed Y | Change | Simulated cycle | Estimate error | Measured gauge-to-tip |
|---|---|---:|---:|---:|
| 2600 | T1 -> T3 | 49.00 s | 0.35% | 120.003 mm |
| 2600 | T3 -> T1 | 48.56 s | 0.43% | 119.996 mm |
| 3150 | T1 -> T3 | 50.23 s | 0.32% | 120.003 mm |
| 3150 | T3 -> T1 | 50.20 s | 0.39% | 119.996 mm |

Maximum recorded dynamic clearance-sample spacing is 56.30 mm; docking,
vertical and probe sampling is finer, with forced endpoints and handoffs.
The conservative 1000 mm/min local docking rate contributes much of the cycle
time. These are reference-plant times with assumed pneumatic/spindle feedback
delays, not measured machine times. App dispatch, actual holder/knob and guided
cover hardware acceptance remain open as described above.
