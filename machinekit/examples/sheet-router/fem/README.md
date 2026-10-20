# SR5 structural screen

## Anchored modal diagnosis (2026-10-20)

Full 100/50 mm, nine-pose, two-bed rerun of the f47ed13f4 CAD. Six anchored supports each use 50,000 N/mm in XYZ, with no rotational clamp. Foundation values are assumptions, not anchor/slab qualification. All masses and the original 428 mm tool lever are retained.

| Bed Y | Worst X N/µm | Worst Y N/µm | Lowest global Hz | Lowest gantry Hz |
|---|---:|---:|---:|---:|
| 2600 | 2.090 | 2.142 | 26.216 (X) | 26.948 (Y) |
| 3150 | 2.044 | 2.135 | 24.177 (X) | 26.054 (Y) |

All 36 solves pass the numerical gates: maximum static mesh change 0.994%,
twenty-mode frequency change 1.741%, force-balance error 0.0210%, static energy
error 0.0042%, and artificial rigid/weld connector compliance 0.0733%. Modal
strain energy agrees with independently integrated modal kinetic energy within
1.352%; the beam/spring fixture also recovers 96.55% spring energy for its soft
rotational-joint mode. Six-gantry-mode profiles below additionally pass forty-mode
convergence. Minimum combined compressive support margin is 1991 N.

The static target passes under the anchored assumptions, but the 40 Hz gantry
target fails. At the long-bed centre, the 26.054 Hz mode is beam/head Y sway with
beam roll about X and non-rigid beam bending: beam 31.3%, frame tubes 28.2%,
Y drive 23.7%, Y blocks 6.6%, floor 3.9% of strain energy. The Y-drive spring
combines rack mesh, reducer and preload hinge. Within it, the hinge takes 64.9%
of compliance (about 15.4% of this mode’s total strain energy), mesh 17.1%
and reducer 18.0%. Its X/Z carriage
elastic energy is small; adding head stiffness or mass does not address its cause.
The lowest 24.177 Hz global mode is X frame/table sway: frame tubes 50.3%,
braces 27.4%, floor supports 17.8%.

The free-standing long-bed centre reproduces the former 23.7 Hz concern at
23.692 Hz: Y frame/beam/head sway, with floor supports 30.2%, frame tubes 24.8%,
beam 17.8% and Y drive 14.8% of strain energy. Its first global X-sway mode is
9.985 Hz, versus about 5.2–6 Hz before bracing; this free-standing number is a
single-pose sensitivity, not a full free-standing envelope. Anchored global minima
are the full-pose values in the table above. Lower frame modes are not dismissed
by the moving-mass classification.

Retain the 40 mm X plate and aluminium Z for the next geometry pass. Returning X
to 25 mm gains only 0.390 Hz while failing static stiffness (1.986 X / 1.494 Y
N/µm at the governing modal pose); steel Z adds 22.95 kg and loses 1.357 Hz.
Increasing Y block spacing adds 5.20 kg for 0.172 Hz, and HGR30 adds mass without
helping this Y mode. None of these swaps approaches 40 Hz. This is a coupled
frame/beam/Y-drive stiffness problem, rather than a defect cured by stiffening
the Z ram further. Mass reduction remains useful only while preserving the
static budget; pocketing needs its own structural check.

Recommended next geometry: recover the 60 mm tool lever through the fork/riser
and matching table/setting-datum redesign, while correcting front-brace loading
access. Keep the 40 mm X plate and aluminium ram. The subsequent modal stiffness
iteration must target the slab-to-Y-rail/frame-to-beam load path and Y drive hinge,
using the measured energy split; lever recovery alone is not claimed to reach
40 Hz. Fabrication/access decisions and machining requirements are settled in
[SOURCING.md](../SOURCING.md#fabrication-and-access-decisions-after-the-braced-frame-screen).

Energy columns below are percentages of **modal strain energy**: native CalculiX ELSE totals plus ½kΔu² for every spring. KE is moving-mass kinetic-energy share; ≥50% defines the reported gantry class. Y guides credit block springs only; Y rail bending is not resolved. X includes plate, guide and drive springs; Z includes ram, ribs, rails, guide/screw and saddle joints. Table and artificial rigid offsets are shown so each row accounts for 100%.

Shapes use global XYZ. Each body’s direction is its RMS displacement direction. Beam rotation is a least-squares rigid-body fit, normalized to 1 mm maximum physical-node motion; bend residual is the remaining normalized RMS displacement. Eigenvector sign is arbitrary. These observations distinguish translation, rotation and beam bending without pretending the entire gantry is rigid.

### 2600 mm bed: first six global modes

Pose X=1600, Y=1300, full Z. Lowest frame-dominant mode is 26.216 Hz. These are the first six modes at the lowest-global-frequency pose, not six copies of one branch at different poses.

| Mode | Hz | Moving KE % | Shape | Floor | Frame | Braces | Y guides | Y drive | Feet/welds | Beam | X | Z | Table | Rigid |
|---|---:|---:|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 1 | 26.216 | 36.4 | frame X, table X, beam X, X carriage X, Z ram X, spindle X; beam RX +0.173 mrad/mm, bend residual 0.017 | 18.7 | 48.1 | 26.9 | 0.6 | 0.2 | 0.1 | 0.8 | 4.2 | 0.2 | 0.2 | 0.0 |
| 2 | 29.259 | 89.4 | beam Y, X carriage Y, Z ram Y, spindle Y; beam RX +1.223 mrad/mm, bend residual 0.102 | 6.5 | 18.6 | 3.6 | 8.0 | 34.6 | 2.4 | 26.0 | 0.3 | 0.1 | 0.1 | 0.0 |
| 3 | 34.978 | 0.9 | frame Z; beam RX -0.033 mrad/mm, bend residual 0.005 | 5.4 | 90.5 | 2.8 | 0.0 | 0.1 | 0.0 | 0.2 | 0.0 | 0.0 | 0.7 | 0.1 |
| 4 | 40.298 | 81.0 | beam Y, X carriage Y, Z ram Y, spindle Y; beam RZ +0.739 mrad/mm, bend residual 0.115 | 2.2 | 20.1 | 2.4 | 0.5 | 23.9 | 0.2 | 50.1 | 0.4 | 0.1 | 0.0 | 0.0 |
| 5 | 42.761 | 11.2 | frame Z, beam Z, X carriage Z, Z ram Z, spindle Y; beam RX +0.124 mrad/mm, bend residual 0.128 | 5.9 | 82.4 | 3.7 | 0.3 | 0.6 | 0.2 | 6.2 | 0.4 | 0.1 | 0.2 | 0.0 |
| 6 | 43.778 | 6.5 | frame X, table X, beam Z, X carriage Z, Z ram Z, spindle Z; beam RZ -0.099 mrad/mm, bend residual 0.076 | 5.6 | 74.2 | 6.3 | 3.2 | 0.7 | 0.8 | 8.7 | 0.2 | 0.1 | 0.1 | 0.0 |

### 2600 mm bed: first six gantry-dominant modes

Pose X=800, Y=1300, full Z, chosen for the lowest gantry frequency. Forty modes were extracted on both meshes to resolve six gantry-class modes; all forty frequencies converge within 1.711%.

| Mode | Hz | Moving KE % | Shape | Floor | Frame | Braces | Y guides | Y drive | Feet/welds | Beam | X | Z | Table | Rigid |
|---|---:|---:|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 2 | 26.948 | 93.2 | beam Y, X carriage Y, Z ram Y, spindle Y; beam RX -1.771 mrad/mm, bend residual 0.123 | 4.6 | 16.2 | 2.9 | 9.7 | 29.2 | 2.7 | 34.3 | 0.3 | 0.1 | 0.1 | 0.0 |
| 4 | 40.890 | 73.5 | beam Z, X carriage Z, Z ram Z, spindle Z; beam RX +0.535 mrad/mm, bend residual 0.140 | 14.3 | 23.3 | 2.8 | 1.2 | 0.8 | 1.6 | 49.7 | 3.0 | 2.9 | 0.2 | 0.0 |
| 5 | 41.543 | 78.9 | beam Y, X carriage X, Z ram X, spindle X; beam RZ -0.842 mrad/mm, bend residual 0.125 | 2.3 | 20.9 | 2.2 | 0.4 | 19.4 | 0.1 | 51.6 | 2.9 | 0.2 | 0.0 | 0.0 |
| 12 | 53.340 | 74.6 | X carriage X, Z ram X, spindle X; beam RZ -0.079 mrad/mm, bend residual 0.020 | 5.8 | 10.8 | 5.3 | 1.2 | 1.3 | 0.3 | 6.8 | 65.6 | 2.7 | 0.3 | 0.0 |
| 30 | 90.867 | 61.5 | frame Z, table Z, beam Z, X carriage X, Z ram X, spindle X; beam RX -0.156 mrad/mm, bend residual 0.122 | 18.2 | 9.5 | 2.1 | 0.4 | 0.0 | 1.2 | 66.8 | 0.7 | 0.8 | 0.1 | 0.1 |
| 37 | 102.085 | 93.9 | beam Z, X carriage Z, Z ram Z, spindle Z; beam RX -0.546 mrad/mm, bend residual 0.051 | 0.5 | 4.8 | 0.4 | 0.5 | 0.0 | 1.6 | 12.8 | 41.8 | 37.4 | 0.0 | 0.0 |

### 3150 mm bed: first six global modes

Pose X=1600, Y=1575, full Z. Lowest frame-dominant mode is 24.177 Hz. These are the first six modes at the lowest-global-frequency pose, not six copies of one branch at different poses.

| Mode | Hz | Moving KE % | Shape | Floor | Frame | Braces | Y guides | Y drive | Feet/welds | Beam | X | Z | Table | Rigid |
|---|---:|---:|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 1 | 24.177 | 31.9 | frame X, table X, beam X, X carriage X, Z ram X, spindle X; beam RX -0.115 mrad/mm, bend residual 0.013 | 17.8 | 50.3 | 27.4 | 0.5 | 0.1 | 0.1 | 0.5 | 3.0 | 0.1 | 0.2 | 0.0 |
| 2 | 27.920 | 71.2 | beam Y, X carriage Y, Z ram Y, spindle Y; beam RX -1.345 mrad/mm, bend residual 0.103 | 4.9 | 36.7 | 4.5 | 4.5 | 24.9 | 1.5 | 22.6 | 0.2 | 0.1 | 0.1 | 0.0 |
| 3 | 33.603 | 3.3 | frame Z, beam Z, X carriage Y, Z ram Y, spindle Y; beam RX +0.083 mrad/mm, bend residual 0.095 | 3.0 | 88.1 | 3.9 | 0.4 | 0.2 | 0.1 | 3.9 | 0.1 | 0.0 | 0.3 | 0.0 |
| 4 | 34.005 | 2.0 | frame X, table X, beam Y, X carriage Y, Z ram Y, spindle Y; beam RZ -0.342 mrad/mm, bend residual 0.031 | 1.9 | 79.6 | 6.1 | 4.1 | 2.8 | 0.9 | 4.4 | 0.0 | 0.0 | 0.1 | 0.0 |
| 5 | 34.517 | 1.1 | frame Z; beam RX -0.061 mrad/mm, bend residual 0.008 | 5.0 | 90.0 | 3.1 | 0.2 | 0.1 | 0.1 | 0.7 | 0.0 | 0.0 | 0.8 | 0.1 |
| 6 | 35.267 | 12.2 | frame Z, beam Y, X carriage Y, Z ram Y, spindle Y; beam RZ +0.355 mrad/mm, bend residual 0.064 | 0.9 | 77.3 | 2.5 | 1.3 | 1.6 | 0.9 | 15.2 | 0.1 | 0.0 | 0.1 | 0.0 |

### 3150 mm bed: first six gantry-dominant modes

Pose X=800, Y=1575, full Z, chosen for the lowest gantry frequency. Forty modes were extracted on both meshes to resolve six gantry-class modes; all forty frequencies converge within 1.540%.

| Mode | Hz | Moving KE % | Shape | Floor | Frame | Braces | Y guides | Y drive | Feet/welds | Beam | X | Z | Table | Rigid |
|---|---:|---:|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 2 | 26.054 | 81.3 | beam Y, X carriage Y, Z ram Y, spindle Y; beam RX -1.813 mrad/mm, bend residual 0.122 | 3.9 | 28.2 | 3.9 | 6.6 | 23.7 | 1.9 | 31.3 | 0.2 | 0.1 | 0.1 | 0.0 |
| 8 | 41.047 | 68.3 | frame Z, beam Z, X carriage Z, Z ram Z, spindle Z; beam RX +0.577 mrad/mm, bend residual 0.128 | 20.4 | 19.7 | 5.2 | 1.0 | 0.4 | 1.4 | 45.1 | 3.2 | 3.2 | 0.3 | 0.0 |
| 9 | 42.934 | 89.0 | beam Y, X carriage X, Z ram X, spindle X; beam RZ +0.654 mrad/mm, bend residual 0.107 | 2.1 | 7.8 | 1.6 | 1.0 | 24.0 | 0.8 | 57.7 | 4.8 | 0.3 | 0.0 | 0.0 |
| 13 | 52.290 | 76.5 | X carriage X, Z ram X, spindle X; beam RZ -0.087 mrad/mm, bend residual 0.013 | 5.6 | 12.9 | 4.1 | 1.4 | 1.5 | 0.3 | 6.2 | 64.7 | 2.9 | 0.4 | 0.0 |
| 30 | 88.317 | 70.2 | frame Z, table Z, beam Z, X carriage X, Z ram X, spindle X; beam RX -0.130 mrad/mm, bend residual 0.126 | 10.5 | 11.1 | 1.9 | 0.5 | 0.0 | 1.4 | 73.0 | 0.7 | 0.6 | 0.1 | 0.1 |
| 39 | 101.523 | 89.8 | beam Z, X carriage Z, Z ram Z, spindle Z; beam RX -0.540 mrad/mm, bend residual 0.049 | 1.2 | 7.3 | 1.3 | 0.5 | 0.0 | 1.5 | 11.8 | 39.6 | 36.5 | 0.1 | 0.0 |

### Single-pose sensitivities

Lowest gantry-dominant mode across both beds/all fine-mesh poses; governing target failure. Bed 3150, X=800, Y=1575, full Z, 50 mm mesh. These are analytical property/layout variants at fixed mounting interfaces, not collision-checked replacement CAD. Y-spacing includes longer steel feet and rails; HGR30 includes catalogue mass increments and ZA radial rigidity. Lateral guide stiffness remains assumed. Changing plate thickness requires reworking the actual mounting stack.

| Variant (ranked by tracked gantry frequency gain) | ΔX N/µm | ΔY N/µm | Δfirst gantry Hz | Δtracked branch Hz | Motion-signature MAC | Δmoving Y kg |
|---|---:|---:|---:|---:|---:|---:|
| X plate 25 mm | -0.272 | -0.641 | +0.390 | +0.390 | 1.000 | -6.531 |
| Y blocks 400 mm | -0.004 | +0.040 | +0.172 | +0.172 | 0.999 | +5.200 |
| welds x0.1 | -0.000 | -0.000 | -0.001 | -0.001 | 1.000 | +0.000 |
| Y HGR30 | +0.000 | +0.001 | -0.261 | -0.261 | 1.000 | +1.480 |
| steel Z ram | +0.415 | +0.309 | -1.357 | -1.357 | 1.000 | +22.946 |
| free levelling feet | -0.005 | -0.001 | -2.362 | -2.362 | 0.982 | +0.000 |

Tracked-branch MAC compares normalized body translations and rotations (200 mm rotation scale), not a consistent-mass matrix MAC. Both the classified first mode and matched branch are reported so threshold swaps are visible. Weld ×0.1 changes translation springs from 10¹⁰ to 10⁹ N/mm and rotation from 10¹³ to 10¹² N·mm/rad; these are quasi-rigid numerical connectors, not qualified weld flexibility. Free supports keep six vertical springs but only X at foot 0 and Y at feet 0/3; friction/no-slip is assumed.

HGR30 uses [HIWIN HGH25CA/HGH30CA](https://www.hiwin.com/wp-content/uploads/Linear_Guideway-E.pdf): radial ZA 539→618 N/µm, block mass increment 0.51→0.88 kg and rail increment 3.21→4.47 kg/m. Incremental mass is added to generic CAD masses; rail bending and supplier mounting-height changes are not resolved.

### Added moving mass

| Member | Previous kg | Revised kg | Added kg |
|---|---:|---:|---:|
| xCarriage | 10.886 | 17.417 | +6.531 |
| zCarriage | 6.623 | 12.030 | +5.407 |
| beamFootLeft | 5.258 | 7.886 | +2.629 |
| beamFootRight | 5.258 | 7.886 | +2.629 |
| beam | 71.934 | 72.502 | +0.569 |
| shoeLiftBracket | 0.173 | 0.292 | +0.119 |
| motorYLeftSupport | 0.084 | 0.158 | +0.074 |
| switchZpositiveLimitMount | 0.159 | 0.156 | -0.004 |
| switchZhomeMount | 0.072 | 0.068 | -0.004 |
| switchZnegativeLimitMount | 0.068 | 0.064 | -0.004 |
| motorZSupport | 0.094 | 0.084 | -0.010 |
| motorYRightSupport | 0.084 | 0.064 | -0.020 |
| **Total moving Y** | **175.860** | **193.775** | **+17.915** |

Fixed base braces are not part of moving Y mass. The beam mass also increases slightly because thicker feet change the generated span. Part-level inertia and material mass are changed in each sensitivity, not just a modal denominator.

Artifacts: `build/sheet-router-modal/anchored/{results,summary}.json`, `extended/extended-modes.json`, `sensitivities/sensitivities.json`, and `diagnosis.json`. The full-run solve-time sources, packet hashes and solver hash remain with the results. Extended/sensitivity runs record their source hashes separately. No fabrication release.


### Reproduce the anchored review

Use the existing mill FEM virtualenv and prebuilt CalculiX. Keep all outputs on
root; DATA1 is only read for the earlier moving-mass comparison. The packets used
here are the exact f47ed13f4 exports copied from
`build/sheet-router-stiffness/validated/y{2600,3150}` into `anchored/`.

```bash
export LD_LIBRARY_PATH="$PWD/machinekit/examples/bench-mill/build/fem/tools/calculix/usr/lib/x86_64-linux-gnu${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
router_python=machinekit/examples/bench-mill/build/fem/venv/bin/python
router_ccx=machinekit/examples/bench-mill/build/fem/tools/calculix/usr/bin/ccx
router_packets=build/sheet-router-modal/anchored
/home/joao/dev/materia-cache/tools/heavy-run --mem 6G --need 1G --label router-modal-full \
  "$router_python" machinekit/examples/sheet-router/fem/study.py --packets "$router_packets" --ccx "$router_ccx" --floor anchored
"$router_python" machinekit/examples/sheet-router/fem/report.py "$router_packets"
/home/joao/dev/materia-cache/tools/heavy-run --mem 5G --need 1G --label router-modal-sensitivity \
  "$router_python" machinekit/examples/sheet-router/fem/sensitivity.py --packets "$router_packets" --ccx "$router_ccx" --output build/sheet-router-modal/sensitivities
/home/joao/dev/materia-cache/tools/heavy-run --mem 5G --need 1G --label router-extended-modes \
  "$router_python" machinekit/examples/sheet-router/fem/extended_modes.py --packets "$router_packets" --ccx "$router_ccx" --output build/sheet-router-modal/extended
"$router_python" machinekit/examples/sheet-router/fem/diagnose.py --packets "$router_packets" \
  --extended build/sheet-router-modal/extended/extended-modes.json \
  --sensitivities build/sheet-router-modal/sensitivities/sensitivities.json \
  --baseline /media/joao/DATA1/materia-router-sr6/final
```

The native beam/rotation/crossing gate, all 37 mill Python regressions, and both
eccentric Y-drive checks pass. Maximum reserved Y-branch force is 931.07 N; the
Hertz diagnostic is 725.79 MPa and is not the rack acceptance limit. No CAD was
changed in this analysis pass. The six sensitivity geometries and fabrication/access
decisions require their own CAD and supplier qualification before adoption.

## f47ed13f4 CAD and comparison history

The working CAD now selects a 40 mm flat X plate, 30 mm welded steel beam feet,
180 mm moving-Z rail spacing and a 20 mm Z web with two closed side sections
25 × 120 × 6 mm. Sixteen RHS 80 × 80 × 4 frame diagonals brace the sides,
ends and lower plane. The original RHS 200 × 100 × 6.3 beam and 100 mm side
allowance remain. Removing that allowance failed native clearance: the outboard
Z screw intersected the base at X=0. The shoe lift has an offset bracket, and the
right Y motor support is lowered to clear the rack. The final geometry is validated against HEAD in an isolated worktree;
the checks and remaining structural work are recorded below.

The beam model now joins welded frame members at interior intersections, rather
than only at their endpoints. A native fixture compares automatic crossing
connectivity with four explicitly joined arms: both deflect 0.007976131 mm under
300 N, with four branches at one physical node. Welded beam feet use two spatial
contacts across the beam footprint instead of a guessed single rotational spring.
That revision originally used finite levelling-foot supports; the anchored baseline
and free-standing sensitivity above supersede that support selection.
Local weld stress, tube-wall flexibility and the built-up aluminium Z joint still
require qualification.

Model correction must be separated from hardware improvement. `compare.py`
re-solves the previous exact CAD with the corrected model at two critical poses
per bed, recording packet and source hashes. At the long-bed centre, the old
hardware gives 1.751 N/µm X and 0.487 N/µm Y. The earlier export of the revised
hardware gives 2.253 X and 2.135 Y at that pose, but its lowest classified
gantry-dominant mode is only 23.688 Hz. This is a single-mesh diagnostic, not
acceptance evidence. It predates the final motor-support clearance
fix. Mixed frame/gantry modes remain relevant regardless of classification.

Working evidence is under `build/sheet-router-stiffness`: `baseline/` contains
the corrected comparison, `cad/` the earlier revised CAD, and `cad-screen/` its
critical-pose screen. Fresh final CAD packets are in `validated/`, exported from an isolated detached
worktree based on HEAD with only the router patch. Both beds export 171 components
and 39 stock members using the existing OCCT libraries. This avoids the unrelated
casting refactor in the shared checkout. The full anchored two-mesh rerun is reported above; exports alone do not qualify
the structural targets.
The isolated guide gate passes both beds, including closed Z-section validity and
volume, sixteen frame braces, moving-rail reach and selected drives. The final
SR4 sizing/planner gate passes 82 assertions. Shared exporter fixtures pass
33 mill plus seven router assertions, native beam/spring/crossing fixtures pass,
and all 37 mill Python regressions pass. Both beds pass the complete assembly
and clearance sweep: 426 assertions, including travel extremes and tool-rack
approaches. Logs are retained in `build/sheet-router-stiffness/isolated-validation/`.

The revised long-bed moving Y mass is 193.77 kg. The ideal drive-only estimate is
47.23 Hz, falling to 36.78 Hz with half hinge stiffness, or 45.03 Hz with 10% extra
mass. The hinge accounts for 64.9% of series drive compliance.

The historical results below retain their original model and geometry; their
results use their original support/model assumptions. No fabrication release.

## Previous guide-first candidate (SR6, 2026-10-20)

This historical candidate implements the first guide revision in [SOURCING.md](../SOURCING.md).
The initial SR5 results below describe the superseded single-block assembly.
The new candidate uses two X rails/four blocks, moving Z rails/four fixed blocks,
280 mm Y block spacing, flat 25/20 mm X/Z plates, 20 mm steel feet and a direct
two-level spindle saddle. It retains the original steel beam and nominal rack size.
Z is 1610, 200 mm travel and 10 m/min; X and braked Z are 750 W.

New exports default to `build/sheet-router-sr6`. Set `SHEET_ROUTER_FEM_OUTPUT`
to place packets on another volume; pass that same path to `study.py`, `report.py`
and the eccentric-load checker. `SHEET_ROUTER_FEM_BINARY`, `SHEET_ROUTER_BINARY`
and `SHEET_ROUTER_SIZING_BINARY` relocate compiler outputs independently.
This run uses `/media/joao/DATA1/materia-router-sr6/final` for packets
and `/media/joao/DATA1/materia-router-sr6/logs` for checks, because root is nearly full.

The updated model uses per-block radial/lateral translation springs at actual CAD
connectors and zero guide rotational springs. X/Z plate grillages span the contact
locations. Moving Z rails receive a conservative 60% gross rectangular-section
reinforcement proxy; mounting stiffness remains assumed. The spindle saddle is
a rigid body supported by four translational mounting springs; clamp, bearing
and tool flexibility are excluded. Lateral guide stiffness and plate Poisson
coupling still require qualification.

The table reacts through six actual bed-stock seats rather than one nearest stock.
Floor-standing blower and cabinet devices do not contribute to frame mass. Other
fixed masses attach to nearest stock on a fixed 50 mm analysis grid. Twenty modes
are reported, including global frame modes. Gantry-dominant modes use a declared
50% moving-mass kinetic-energy criterion; this is a screening classification,
not a validated separation of frame and gantry dynamics. Static and first-six-mode
convergence gates remain 3% and 5%. Per-case raw field files are gzip-compressed
with integrity verification; decks, tables, logs and source snapshots remain.

The settled CAD passes 426 assembly/clearance assertions for both beds, 292
component assertions, 82 revised SR4 sizing/planner assertions, and the guide/reach
gate. All five legacy gantry hashes and 59 axis assertions remain unchanged.
The shared exporter/centreline fixtures pass (33 mill plus seven router checks),
as do the independent beam/spring native fixture, mill packet/STEP comparison
and 37 mill Python regressions. These checks do not establish the structural
targets. The current candidate remains a screening design, with no fabrication release. The shoe envelope is 150 mm diameter with a 40 mm brush
and a front-directed short hose neck; its lift and rack clearances are checked
with the same assembly poses.

### Settled candidate results

All 36 solves (two meshes, nine full-Z poses, two beds) pass the numerical gates.
Static compliance changes by at most 1.092%; all twenty frequencies by 0.875%.
Maximum force-balance error is 0.0292%, energy error 0.0108%, reciprocity error
0.000031%, and artificial rigid-member compliance 0.0752%. Every support retains
positive compression under the combined envelope.

| Bed Y | Worst X stiffness | Worst Y stiffness | Global first-mode range | Lowest classified gantry-dominant mode |
|---|---:|---:|---:|---:|
| 2600 mm | 1.154 N/µm | 0.481 N/µm | 5.727–6.027 Hz | 27.249 Hz |
| 3150 mm | 1.005 N/µm | 0.480 N/µm | 5.177–5.468 Hz | 30.760 Hz |

The 2 N/µm X/Y and 40 Hz gantry targets still fail. The modal classification is
only the declared ≥50% moving-mass energy screen: mixed frame/gantry modes below
that threshold remain relevant and the global frame modes are not dismissed.
The lower spindle datum necessary for rack clearance leaves approximately 428 mm
from beam centre to longest-tool tip at full Z. Shorter-tool reach remains open.

In the worst Y cut, the Z plate plus its moving-rail reinforcement account for
about 56% of compliance; the X plate contributes about 13% and the gantry beam
about 9%. The worst X case is led by the base/frame contribution. Investigate the
Z load path and frame bracing/supports before selecting a box X carriage. These
are model-based priorities, not fabrication approval.

Worst reserved eccentric Y branch demand is 874.60 N. Both drives pass torque,
reducer, preload and conditional manufacturer rack checks; the mounting-derated
rack limit is 3000 N, versus 935.77 N after including preload-flank demand.
The 715.08 MPa Hertz diagnostic remains visible without
using the old assumed 700 MPa limit to resize the rack. The specified hardened
family, mounting and application factors still require qualification.

The long-bed Y drive-only estimate is 49.58 Hz at 175.86 kg. Halving hinge
stiffness gives 38.61 Hz; 10% extra mass gives 47.27 Hz. Hinge compliance is 64.9%
of the series drive compliance. These estimates do not replace the full structure.

`summary.json`, `results.json`, `eccentric-drives.json`, posed STEP files, native
decks/tables, compressed field files and solve-time source snapshots remain in
the external packet directory named above. `summary.json` records the actual
postprocessor SHA separately from the solve-time source snapshot.

## Initial SR5 baseline (historical)

The current SR3/SR4 assembly is rejected for fabrication. The structural screen is
complete; the SR-D3 targets are not met. A redesigned carriage/guide layout must
be studied before SR6 releases drawings.

The shared `machinekit.structural.fem.AssemblyFemExport` writes posed STEP,
materials, mass, centre of mass, world centroidal inertia and bounds from any
`MachineAssembly`, with a caller-supplied solid selection. Both the mill and router
use it. Fresh mill exports before and after extraction match in all 220 component
records and every byte of the 36 STEP bodies; source/build provenance is excluded
from that comparison because the exporter source changed.

## Reproduce the current candidate

Use the preceding SR6 model description for current runs. The model description
and baseline results below this command section are historical.

Run from the repository root. Reuse the pinned mill Gmsh 4.12.2 environment and
CalculiX 2.21 installation; no native build or new Python environment is needed.
Use the project's `heavy-run` resource gate for compilation and the full study.
The example gate below is the workstation tool used for this run.

```bash
heavy_run=/home/joao/dev/materia-cache/tools/heavy-run
fem_python=machinekit/examples/bench-mill/build/fem/venv/bin/python
fem_ccx="$PWD/machinekit/examples/bench-mill/build/fem/tools/calculix/usr/bin/ccx"
export LD_LIBRARY_PATH="$PWD/machinekit/examples/bench-mill/build/fem/tools/calculix/usr/lib/x86_64-linux-gnu${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"

"$heavy_run" --mem 10G --need 1G --label sr6-export -- machinekit/scripts/test-sheet-router-fem
"$fem_python" machinekit/examples/sheet-router/fem/verify.py --ccx "$fem_ccx" --output build/sheet-router-sr6/verification
"$heavy_run" --mem 10G --need 1G --label sr6-study -- "$fem_python" machinekit/examples/sheet-router/fem/study.py --packets build/sheet-router-sr6 --ccx "$fem_ccx"
"$fem_python" machinekit/examples/sheet-router/fem/report.py build/sheet-router-sr6
"$heavy_run" --mem 10G --need 1G --label sr6-eccentric -- machinekit/scripts/test-sheet-router-fem --y-loads
```

`--checks-only` runs the Haxe checks without exporting. `--export-only`
exports a geometry iteration without repeating the unchanged exporter fixtures;
run those fixtures separately before completing a study. `--layout` checks the
new guides, moving-rail reach and selected drives; `--baseline` checks legacy
gantry hashes and axis behavior. The optional verifier
arguments `--mill-before <folder> --mill-after <folder>` compare fresh legacy mill
packets. `study.py --one` is a diagnostic centre-pose run; it writes partial output
and cannot produce a completed study. The full study publishes `results.json`
only after every solve, force/energy check and convergence gate passes. Per-case
input decks, solver logs and numerical tables remain beside the report.

## Model and limits

The packet includes all 165 posed components, seven selected STEP bodies, 24 RHS
stock descriptions, plate dimensions, nine full-Z poses and six foot locations.
Each bed length has a 3 × 3 corner/midspan grid at Z = 250 mm. At each pose:

- X/Y/Z cutting cases apply 300 N to the tool and the opposite force to the bed;
- gravity includes all authored part masses;
- X and Y acceleration cases apply 3 m/s² to the corresponding moving masses;
- six modes are solved with the same restraints and mass model.

Gmsh meshes quadratic lines. CalculiX B32R BOX sections represent the hollow
stocks; RECT sections represent full-width plate strips. The beam feet are strips
between the two Y blocks, 140 mm apart. The X plate joins the X block to the Z
column head; the Z carriage is a strip from its block to the spindle attachment.
Stiff offsets carry forces and moments to actual tool/guide positions. Each joint
uses translation and rotation reference nodes and six independent springs.
Direct SPRING2 rotational DOFs on beam nodes are unsuitable here; the analytical
fixture verifies the reference-node coupling.

Assumed inputs are steel E = 210 GPa, aluminium E = 69 GPa, nu = 0.3, guide
translation = 100 kN/mm and individual guide rotation = 10 MN·mm/rad. Bolt springs
are 1 MN/mm and 1 GN·mm/rad. Feet have 50 kN/mm vertical springs and three horizontal
restraints in a 3-2-1 arrangement. The actual SR4 rack/reducer/preload and screw
stiffnesses supply the held-axis drive springs. They include elastic compliance;
backlash, tuning and following error are excluded.

Native stock sections carry distributed mass. Base stock densities are scaled to
match the CAD weldment union mass, avoiding double-counting stock intersections.
Remaining part masses, centres and full inertia tensors are aggregated per rigid
carrier, then represented by six positive point masses. Entire cable chains move
with their carrier, matching SR4's conservative allowance. Fixed accessory masses
are transferred to nearby frame supports; separate accessory floor supports are
not modelled.

The bed reaction is a rigid link to the nearest authored bed stock member. Table
diaphagm action and distributed table attachment are excluded. Spindle mount,
spindle bearings, tool and local tube-wall deformation are rigid/omitted. No weld,
bolt/contact, stress, buckling, fatigue, damping or nonlinear qualification is
claimed. The very large calculated deflections exceed the useful precision range
of a linear model: they reject this layout rather than predict cutting accuracy.

CalculiX's authored beam/section and output definitions are documented in the
[official manual](https://www.dhondt.de/ccx_2.20.pdf). Line connectivity follows the
[Gmsh API reference](https://gmsh.info/doc/texinfo/): Gmsh start/end/mid nodes are
reordered to CalculiX start/mid/end.

## Result, 2026-10-20

| Bed Y | Worst X stiffness | Worst Y stiffness | First mode across poses | Largest gravity frame twist |
|---|---:|---:|---:|---:|
| 2600 mm | 0.006106 N/µm | 0.005849 N/µm | 2.7318–2.7329 Hz | 0.00825 mm |
| 3150 mm | 0.006100 N/µm | 0.005849 N/µm | 2.7316–2.7327 Hz | 0.00902 mm |

Targets are ≥ 2 N/µm XY and ≥ 40 Hz for the first gantry mode. The lowest mode has
substantial tool-to-bed motion, rather than being an irrelevant accessory mode.
The report retains all six frequencies and relative tool-motion indicators.

Worst full-Z X compliance is about 164 µm/N: the X guide contributes 90, the X
plate 45, and the Z guide 26 µm/N. Worst Y compliance is about 171 µm/N: the X plate
contributes 92, X guide 60 and Z guide 15 µm/N. The X plate term alone rejects the
2 N/µm target even if the guide assumptions are made much stiffer. Budgets are
strain-energy contributions for the closed tool/bed load loop, including springs.
Support reactions and acceleration displacements are retained for every case.

The eccentric Y branch sees up to 827.65 N after the 1.5 reserve, including the
300 N cutting vector envelope, Y acceleration and 15 N side drag. The servo torque
and preload still pass, but the original 20 mm engaged face reaches 706.06 MPa
against SR4's assumed 700 MPa contact limit. A **25 mm engaged face on both rack
and pinion** passes that tooth screen (about 632 MPa); its physical fit and the
redesign's changed moving mass must be checked before changing the selected parts.

Decisions for the next design iteration:

- Retain **steel RHS 200 × 100 × 6.3** as the beam baseline. Same-section aluminium
  has only 0.329 times the bending rigidity. Enlarging the beam alone cannot cure
  the dominant carriage/guide terms.
- Replace the open 12 mm aluminium X plate with a **closed box bridge**. Use spaced
  X rails and blocks to support roll as well as pitch/yaw; two blocks on the same
  X rail alone do not fix roll about that rail.
- Replace the single Z block with spaced blocks/rails and obtain supplier stiffness.
- Reject the **12 mm aluminium beam feet** for final design. **20 mm steel** is the
  next candidate, with about 14 times the strip bending rigidity. This is a design
  choice to package and recheck, not an analysed replacement assembly.
- Keep tubes **empty**. Filling the beam adds 39.31 kg/m to its 28.43 kg/m steel mass.
  Even a perfectly bonded 30 GPa fill gives only 1.37/1.24 times strong/weak bending
  rigidity; an unbonded fill gives no credited stiffness. Damping is unquantified.

These candidates are not claimed to meet SR-D3. SR6 must begin with structural
redesign and rerun SR4/SR5 before fabrication drawings are released.

## Verification

The 100/50 mm global meshes use four-times-finer plate-strip spacing. Across all
18 poses, maximum diagonal compliance change is 0.61% and maximum change of the
first six frequencies is 0.31% (gates 3% and 5%). The 2 m RHS cantilever agrees
with the stated bending/shear formula within 2%, and the rotational-spring
fixture within 0.07%. Halving its mesh changes deflection by 0.67%.

Each solve checks positive compliance, summed energy against external work, and
support force equilibrium. Postprocessing checks compliance reciprocity, negligible
artificial-member compliance and compressive foot reactions under gravity plus
the cutting vector and both acceleration envelopes. The smallest remaining foot
compression is about 75 N within this assumed foundation model. The Haxe gate
passes seven shared-exporter/frame-data checks and 33 existing mill datum checks;
the existing mill Python suite passes 37 tests.

Generated evidence is under `build/sheet-router-sr5/`: `summary.json`,
`results.json`, `verification/verification.json`, `y-loads.json`, and
`eccentric-drives.json`. Packet, solver executable, study source and report hashes
are recorded. Generated files are intentionally outside project history.
