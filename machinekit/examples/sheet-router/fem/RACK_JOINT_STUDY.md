# Rack load-path and corrected motion-envelope study — 2026-10-22

**Do not adopt the draft support.** Its finite seat compliance makes the fine-mesh
static target fail. Nominal motion also fails, before any hold or override is
admitted. Retain every brace and the frozen CAD. Use the shared RM2 mount family
for the eventual support; its physical design and the frozen rebuild need review.
The catalogue rack and RM2 plan are separate changes. No RackSeat,
RackMountPattern or RackSeatHoles component is introduced by this study.

## Motion envelope correction

The [audit](MOTION_ENVELOPE.md) and [machine-readable audit](brace-results/envelope-audit.json)
identify the original histories and poses. J4 suffixes `1p5` and `3` are **request
times in seconds**, not override multipliers. All eight controlled exports request
75% feed. Their ordinary-planner offline prefix/stop/resume/tail construction has
continuous V/A/J certificates, but does not exercise J5's ready stop and filtered
executor clock. These histories are diagnostic only; excluding them does not
remove a nominal contour failure.

The accepted envelope is: ready jerk-limited hold/resume; at or below 100%,
jerk-filtered executor time scaling, including derivatives of the clock factor
in the V/A/J admission check; above 100%, next-window raised-feed replanning under
the original axis V/A/J and maximum feed. No `k>1` executor history is admitted.
The available J4 exports do not qualify those J5 controls or above-100% replanning.
They remain pending exports from the motion session. No artificial overspeed
history was substituted. Refer to MotionKit ONLINE_MOTION OM-R1/J5.

Original frozen model, **nominal only**, before support mass/compliance:

| Bed | h50 contour / moving handover, mm | Governing contour / handover pose X,Y, mm |
|---|---|---|
| 2600 | 0.063071 / 0.046303 | 1600,1300 / 800,2600 |
| 3150 | 0.041219 / 0.027831 | 800,0 / 800,0 |

The short-bed h100 contour is 0.064081; long-bed h100 is 0.044131. The former
0.063195 at X0/Y1300 is `override-3` (75% request at t=3 s); the former
0.043973 at X0/Y0 is `hold-resume-3`. W066's 13.027551 MPa is the long-bed
X800/Y0 prepared-weld model, `hold-resume-3`, frequency factor 0.85 and damping
0.005; halving the time step gave 13.029258 MPa. These are not 3× speed results.

## Review-only mass and compliance input

The removed draft is retained as text in the evidence archive, never as live
components. It is an intentionally assumed sensitivity, not a proposed catalogue
hole pattern or an RM2 implementation. It models a 50×8 mm shelf with an 8×11 mm
lip, 100 mm support spacing, neighbouring M6 joints, 30×30 mm replication pads,
4.210 mm assumed epoxy, and 4/8 mm Y/X risers. The lip receives no approved datum
status: RM-D1 requires assembly adjustment against a side bar with push screws.
RM2 owns the replication/riser allowance and shared element-derived seat holes.

Steel E=210 GPa, ν=0.3; epoxy E=4 GPa, ν=0.35; M6 threaded area 20.1 mm².
Bolt shear, bonded-pad shear and bar extension are in series. Normal bar bending
includes a simply supported 100 mm span and a 28 mm transverse cantilever.
The pressure-angle and helix separating forces contribute through their squared
projections into the effective tangential compliance. Ignoring those projections
was the earlier axial-only sensitivity, retained separately and superseded here.
There is no friction, perfect dowel, or composite-bedding stiffness credit.

| Axis | Effective seat, MN/m | Old drive per pinion, MN/m | Drive plus seat, MN/m |
|---|---:|---:|---:|
| X | 31.158 | 8.532 | 6.698 |
| Y | 28.680 | 8.532 | 6.576 |

Normal seat stiffness is 18.091 kN/mm: 350 N gives 0.01935 mm local deflection.
The Y pair falls from 17.065 to 13.152 MN/m. Contact opening, preload retention,
grout creep and global normal-force coupling are unresolved. These assumed
linear spring values need joint qualification; the catalogue feed-force rating
does not establish mesh or mounting stiffness.

CAD witness includes real fastener solids/BOM rows and corrected rack poses;
frame/rail-pad drilling mass credits are conservatively omitted. Added moving Y
mass is **10.470 kg** on either bed; added fixed mass **32.057 / 37.209 kg**.
Moving Y becomes **207.003 / 207.120 kg**. Gross support/hardware before rack
hole credits is 43.108 / 48.314 kg. These masses describe the draft only.

The study puts both Y racks below their pinions: pinion centre Z858, pitch line
Z842.084506 mm; the frozen right rack incorrectly has pitch Z873.915494 mm.
The X pitch line is Z1116.084506 mm. Study-only CAD sweeps pass 78/90 poses,
393/514 near pairs, minimum clearance 1.375 mm against 0.5 mm. Contact witnesses
are within 0.001 mm. This does not approve the assumed holes or datum scheme.
Supplier sections will require the recipe's actual 19 mm face and opposite-hand
pinion; the frozen assumed 20 mm face cannot silently acquire that rating.

## Full structural rerun

Anchored floor, unchanged sixteen braces, populated ATC, all nine full-Z poses,
both beds, h100/h50: **36 cases, 40 modes each**. Serial CCX only. The actual
seat springs carry their own strain energy; massless series nodes have no free
transverse modes. Reciprocal compliance, equilibrium, energy, positive frequency
and spring-condensation checks pass. The largest condensation-share residual is
2.40e-7 and partition residual 1.80e-7. Numerical checks passing does not turn a
design failure into a pass. [Condensed results](rack-results/summary.json).

| Bed | h50 worst X / Y, N/µm | Lowest global Hz and shape | Lowest gantry-dominant Hz |
|---|---|---|---:|
| 2600 | 2.03284 / **1.98085** | 25.3407, beam/gantry Y bending and Y drive | 25.3407 |
| 3150 | **1.98448 / 1.97582** | 23.3832, frame-X sway | 24.3584 |

Static X governs at X0/Y0 on both beds; Y at X800/Y1300 or Y1575. Lowest global
poses are X800/Y0 and X800/Y1575. Lowest long-bed gantry mode is X800/Y3150.
Gantry-dominant means at least half the computed kinetic energy is in the gantry;
mode crossings make this classification more useful than comparing mode numbers.
Frequency remains reported, not a gate.

First short-bed mode has beam 32.6%, Y drive 28.7%, Y blocks 11.2%, frame 10.3%,
seat/bolts 8.5%, feet/foot plates 7.3%, braces 0.9%. Lowest long-bed frame mode
has tubes 47.2%, braces 31.2%, floor 15.3%; it is a global frame mode. Full group
shares, fitted body motions, rotations, tip participation and acceleration residues
for the lowest six at each governing global pose are in the condensed results.


Lowest six at the governing global poses (direction is the fitted body motion;
percentages are strain energy, not modal participation):

| Bed | Hz | Frame / beam direction | Largest energy groups |
|---|---:|---|---|
| 2600 | 25.341 | Y / Y | gantry 32.6%, Ydrive 28.7%, Yblocks 11.2% |
| 2600 | 26.478 | X / X | frame 44.6%, feet 16.3%, brace_end 13.0% |
| 2600 | 28.089 | Z / Y | frame 93.7%, brace_side 3.4%, feet 2.0% |
| 2600 | 32.332 | Z / Z | gantry 61.5%, frame 20.7%, feet 5.3% |
| 2600 | 34.665 | Y / Y | frame 95.9%, feet 2.4%, brace_side 0.8% |
| 2600 | 35.120 | X / Y | gantry 56.2%, Ydrive 20.2%, frame 8.0% |
| 3150 | 23.383 | X / X | frame 47.2%, feet 15.3%, brace_end 11.2% |
| 3150 | 25.190 | Y / Y | gantry 32.1%, Ydrive 27.0%, frame 11.3% |
| 3150 | 27.759 | Z / Z | frame 93.0%, brace_side 3.7%, feet 2.4% |
| 3150 | 32.100 | Z / Z | gantry 69.7%, frame 10.9%, feetplates 9.3% |
| 3150 | 33.456 | X / Y | gantry 62.8%, Ydrive 21.7%, rackSeatBolts 6.5% |
| 3150 | 34.598 | Y / Y | frame 95.3%, feet 2.7%, brace_side 1.0% |

## Nominal motion and policy options

All forty modes carry history from rapid entry through contour and stop; no
zero-state reset at cut entry. Screen at frequency factors 0.85/1/1.15 and damping
0.005/0.0075/0.01/0.015/0.02/0.025/0.03. These **21 sampled uncertainty cases are
not a continuous-frequency certificate**. The nominal export's 0.25 ms sampling
and ordinary-planner continuous V/A/J certificates are retained. All following
errors are computed/unmeasured. The comparison is against the approved SR-D3
0.05 mm finishing budget and the selected J4 0.04 mm reserve.

Draft h50, unchanged selected J4 nominal timing:

| Bed/model | Contour, mm (X,Y) | Moving handover, mm (X,Y) | Stopped residual, mm |
|---|---|---|---:|
| 2600 merged | **0.056551** (1600,2600) | **0.064781** (800,1300) | 0.027505 |
| 3150 merged | 0.045720 (800,1575) | 0.046765 (1600,0) | 0.020146 |
| 3150 prepared-weld, one pose | **0.056012** (800,0) | 0.021687 (800,0) | 0.029938 |

Do not claim the long bed passes the budget from the merged model alone. Its
prepared-weld counterpart is more adverse. It is a one-pose diagnostic, not a
second all-pose fatigue qualification. The h100 short contour reaches 0.058844.

The policy alternatives below are **new ordinary-planner exports at lower axis
jerk**, not time-scaled CSVs or above-100% overrides. Acceleration remains 3 m/s²;
contour feed remains 20 m/min, rapid 40 m/min. Jerk multipliers apply to X/Y.
Short baseline is ZV (Jx=377.421, Jy=57.814 m/s³); long is unshaped
(Jx=134.712, Jy=14.709). Shaper centre remains the selected original J4 value
27.4344 Hz, damping 1%; no mode-by-mode retuning was used to hide detuning.

| Bed/policy | Worst of contour/handover/stop, mm, h50 | Added representative-contour time | Reserve screen |
|---|---:|---:|---|
| 2600 baseline ZV | 0.064781 | 0 | fail |
| 2600 ZVD, same jerk | 0.047093 | +0.344564 s (6.5%) | budget only |
| 2600 ZV, 0.75 jerk | 0.057608 | +0.271720 s | fail |
| 2600 ZV, 0.50 jerk | 0.037846 | +0.749962 s (14.2%) | sampled pass |
| 2600 ZV, 0.35 jerk | 0.021726 | +1.351359 s | sampled pass |
| 2600 ZVD, 0.75 jerk | 0.040694 | +0.619267 s | fail reserve |
| 2600 ZVD, 0.50 jerk | **0.027828** | **+1.091114 s (20.7%)** | sampled pass |
| 3150 baseline | 0.046765 / **0.056012 prepared** | 0 | fail |
| 3150 0.75 jerk | 0.038532 / 0.039758 prepared | +0.327013 s (5.7%) | little margin |
| 3150 0.50 jerk | **0.029552 / 0.030030 prepared** | **+0.864532 s (15.0%)** | sampled pass |
| 3150 0.35 jerk | 0.019273 / 0.017659 prepared | +1.463236 s | sampled pass |

Baseline contour durations including rapid/handover/tail are 5.270061 s short,
5.777883 s long. Costs are for this representative contour, not a constant delay
per arbitrarily subdivided move or a sheet throughput prediction. h100 0.5-jerk
ZV short gives 0.037949 mm; long 0.030011 mm. Short ZVD/0.5 jerk has more reserve
than short ZV/0.5 jerk and is the candidate for a future dense/J5 replay.

Pose scheduling can choose these lower-jerk policies in advance of a window;
the demonstrated all-pose fallback costs +0.750 s short (ZV) or +1.091 s (ZVD),
+0.865 s long. A fast pose can retain baseline timing. This bounds a scheduling
opportunity, not an implemented certified scheduler: geometry interpolation,
look-ahead window boundaries and J5 controls must be checked. No abrupt jerk
limit change or discontinuous acceleration is approved. A separate reduced-A
policy was not needed to reject the draft; it remains an option if the actual
RM2 modes make the demonstrated jerk policies insufficient.

## Drive stiffness bounds (option c)

Short bed X800/Y1300 h50: baseline Kx/Ky=2.22413/1.98085 N/µm, first
**gantry-dominant** mode 25.66177 Hz. Each alternative is also replayed at
X800/Y2600. These poses already fail the unchanged nominal policy, so no passing
all-bed/all-pose hardware qualification is claimed. All cycle-time costs below
are zero only because the nominal history is unchanged.

| Bound | ΔKx / ΔKy, N/µm | Δfirst gantry, Hz | Worst sampled motion, mm |
|---|---:|---:|---:|
| Y reducer stiffness ×2 | +0.0010 / +0.0209 | +0.319 | 0.064417 |
| Y hinge/motor mount ×2 | +0.0038 / +0.0776 | +1.329 | 0.064100 |
| Y seat ×4 | +0.0026 / +0.0527 | +0.899 | 0.064140 |
| Module 2, 15-tooth, 24 mm face | +0.0007 / +0.0148 | +0.268 | 0.062540 |
| Both X/Y seats ×4 | +0.1287 / +0.0527 | +0.900 | 0.050709 |
| Both seats ×4 + Y mount/reducer ×2 | +0.1345 / +0.1584 | +2.780 | 0.052710 |

Mass is held fixed as an **optimistic bound**, not a selected replacement's mass.
Only module 2 adds actual catalogue rack stock mass (4.1 kg/m); its pinion mass
is held fixed and its section inertia merely scaled, pending geometry. Its mesh
compliance scales by (1.5×20)/(2×24)=0.625, an assumption, not supplier stiffness.
It retains 100 mm/rev but requires raising the adjustable pinion plate at least
1.916 mm for an 8 mm shelf and 2 mm minimum replication. It is not a buildable
proposal. Capacity margin alone cannot justify its modal benefit.

Mode crossings explain why an apparent increase of the lowest *global* mode
is smaller than the gantry-mode change. Increasing stiffness is also not
monotonic in contour error. None of these optimistic local drive changes alone
meets 0.05 mm. Y-only changes do not repair the long-bed X static deficit.

## Weld diagnostic and release gate

Life stays **100 million full-load reversals**, acceleration 3 m/s² plus cutting
allowance, FAT36 cut-off 12.669282 MPa (γMf=1.15), unchanged. In the new prepared
long-bed X800/Y0 case the maximum nominal dynamic range is 11.199452 MPa at W054
(`lower0-p0`/`post03-p0`). W066 nominal is 11.078231 MPa. The excluded old
`hold-resume-3` diagnostic reaches 12.750252 MPa at W064
(`lower1-p0`/`post11-p0`), and W066 reaches 12.680337 MPa. All three maxima use
frequency factor 0.85 and damping 0.005. These diagnostics retain the cutting
allowance and response tail; they do not qualify J5 or all poses.

Propose paired 100×100×6 mm triangular steel gussets at W066 and W064, wrapped
load-transfer welds with no grazing ends: gross +0.471 kg fixed per pair. Retain
FAT36 until actual detail/reaction checks establish otherwise. Alternatively use
a designed full-penetration detail, with access, procedure and inspection needed
to justify its fatigue category. Neither upgrade has been modelled or put into
frozen CAD. Do not solve this by reducing loads or life. Recheck after the shared
mount and final J5 histories; fatigue release is still blocked.

## Recommendation and pending API landing

Keep all braces. Reject the draft support. Build the shared RM2 physical mount
with element-derived holes and measured-adjustment relationships, then approve
its router adoption and rerun both static axes, history and weld reactions.
Both X and Y seat compliance matter. A stiffness-only ×4 target is a useful
sensitivity, not a qualified mounting design or automatic approval. Short
ZVD/0.5 jerk and long 0.5 jerk are promising timing candidates; they require a
dense uncertainty sweep, ready-stop/clock/raised-feed J5 exports and SR6-C
measured modes/damping before acceptance.

The separate required-mount API patch passes isolated focused tests: physical
host/contact, wrong datum/normal rejection, mirrored pinion consistency and the
migrated AxisBuilder kinematic/atomicity checks. It removes the bounding-box
supportedSide heuristic and updates every discovered driveRack caller. It is
**held, not landed**: enforcing it now rejects the frozen unsupported router
before RM2 exists. The concrete patch is retained for review; do not mistake the
catalogue commit for enforcement. Shared mount implementation remains in RM2,
after tooling phase-7 step 2. MK10's existing fixed-part support-path plan/check
already covers the generic integrity requirement.

Cable-chain visuals remain queued. No fabrication release or frozen geometry
change follows from this study.

## Retention and reproduction

Retained archive: `/media/joao/DEV/materia-fem-runs/sheet-router-evidence/rack-joint-20261022.tar.gz` (29456117 bytes), SHA256
`cd6b03021d12c9bbaa90d499e456b5d73cd3d3267d483415fbeb8f737f1469bd`. All 344 regular files were read back and
verified against the internal inventory before deleting the untracked draft
text directory. [Provenance](rack-results/provenance.json) lists every retained
file/hash. Reproducible bulk solver DAT is not retained; its deletion/hashes are
recorded, while decks, logs, parsed modes, reaction histories, exports, immutable
manifests, source snapshots and CAD witnesses are retained. No symlinks were
archived or used for deletion. DATA1 could not fit the archive, so it is on the
separate DEV filesystem; the original historical brace evidence remains there.

Extract into a disposable directory. `rack-joint/run-coupled.sh` records the two
meshes/beds/all-pose solve invocation; set CCX's library path as in its existing
wrapper, run serially through heavy-run, and keep the 8 GB root reserve. The
retained pinned replay source is `pinned-replay/source/`; new policy CSVs are
losslessly compressed and `rack_policy.py` validates their uncompressed hashes
against their producer manifests. Existing native producer/CCX binaries are
external dependencies identified by the retained provenance, not rebuilt tools.

From the repository root after extraction to the original working paths:

```sh
python3 machinekit/examples/sheet-router/fem/rack_checks.py \
  --root build/sheet-router-work/rack-joint \
  --output build/sheet-router-work/rack-joint/verification.json
python3 machinekit/examples/sheet-router/fem/rack_report.py \
  --root build/sheet-router-work/rack-joint \
  --output build/sheet-router-work/rack-joint/condensed.json
```

Numerical coverage/energy checks pass for 36 cases and 1440 modes. Eight study
Python sources compile in memory. Catalogue focused checks passed separately;
required-mount focused tests passed against the isolated patch. This is an
engineering screen with stated assumptions, not measured machine acceptance.
