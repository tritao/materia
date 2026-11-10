# Flexspline cup on an expanding mandrel: workholding FE study

Linear elastic, CalculiX 2.21 (C3D8I hex), own structured mesher (`cupfem.py`, no Gmsh needed). Units N, mm, MPa; results in um.
Reproduce from the repo root (about 20 min total): `python3 machines/flexspline-hobber/workholding/fem/case1.py`
then `case2.py`, `case3.py`, `case4.py`, `mesh_check.py`, `plots.py`. Solver runs go through `heavy-run`; scratch in `fem/work/` (deleted after the run), JSON in `results/`, figures in `plots/`.
Plots need only system python3 (numpy, matplotlib). Nothing outside this folder is touched.

## Model and assumptions

Cup (read from `FlexsplineCupBlank` / `FlexsplineHobber.hx`, module 0.3, 200 teeth, face width 8): bore O58.05 (r 29.025), root O59.25 (r 29.625) so
the wall is **0.6 mm under the root over the whole 40 mm height**, tooth band z 0..8 at the open end, 2 mm hub plate at z 38..40.
- Teeth are a **smeared ring at the root diameter** (teeth add no hoop stiffness); displacements are reported on the root surface r 29.625, band z 0..8.
- The hub plate is **omitted** (wall modelled to z 40, free top). Shell edge effects decay as exp(-0.31 z/mm), so the end is ~1e-3 of the effect 22 mm from a 16 mm sleeve; checked by fixing the top radially: identical to 4 digits (`L16_topfixed`). The "stiff closed end" therefore does **not** cause any taper here: the body is as thin as the rim. Only the sleeve end does.
- Seat: open end on the 0.3 mm stop land (O58.65: r 29.025..29.325), frictionless axially; radially "slide" (rim free) or "stuck" (friction holds the rim) as stated.
- Half model (symmetry plane y=0), 36 elements per 180 degrees, 2 elements through the wall, 0.5 mm axial elements in z 0..16.
- Sleeve (my assumption; the repo only gives wall 1.5, length 16, starting at the seat, 0.5 mm oil cavity): tube OD = bore minus the radial gap, wall 1.5, oil pressure on the inner face, **inner faces rigidly fixed over end lands** (mandrel shoulders). Variants: V1 lands 1.5/1.5 mm (open end / far end), V2 0.5/1.5, V3 free open end + 1.5 far land, V4 as V1 with L 24. Oil is constant pressure (cavity not modelled as a closed volume); gap 5 um radial = 10 um diametral unless stated.
- Plug (case 3/4): elastic tube r 5..(bore - clearance), z 0..18 (mandrel height), base fixed, frictionless contact (penalty K 1e5 MPa/mm).
- Force: total F spread over the tooth band (z 0..8) and +-5 degrees (about 5 mm) on the root surface, fixed Cartesian direction (radial = inward at the load point, tangential = +y). Axial pull: uniform line load on the top edge.
- Linear: scale per MPa / per N. Contact cases are essentially linear once engaged.

## Results

### Case 1: uniform contact pressure over the sleeve length (direct bore pressure, 1 MPa)

Thin-ring theory p r^2/(E t) = 6.69 um/MPa; FE band mean (L16) 6.84 um/MPa.

| pressurised z range | band mean um/MPa | taper over band um/MPa (max-min of circumferential mean) | u at seat | u at z 8 |
|---|---|---|---|---|
| 0..4 | 3.50 | 6.09 | 6.44 | 0.35 |
| 0..6 (stops short) | 4.95 | 5.75 | 7.23 | 1.48 |
| 0..8 (just the band) | 6.03 | 3.90 | 7.26 | 3.37 |
| 0..10 | 6.64 | 1.80 | 7.06 | 5.27 |
| 0..12 | 6.45 | 0.47 | 6.68 | 6.27 |
| 0..16 (nominal sleeve) | 6.84 | 0.245 | 6.73 | 6.98 |
| 0..24 | 6.75 | 0.008 | 6.75 | 6.75 |
| 2..18 (sleeve starts 2 mm above seat) | 4.98 | 4.38 | 2.57 | 6.95 |
| 4..20 | 3.27 | 6.17 | 0.30 | 6.48 |

Rule: the pressure must cover the whole band **plus about 8 mm beyond it (~3 shell lengths 1/beta = 3.3 mm)** to keep taper below ~4% of the mean; a sleeve ending at the band end gives taper ~60% of the mean. An open-end gap of 2 mm is as bad as a short end (the rim just does not expand). Taper becomes flank-line (helix/lead) error ~ taper x sin(20 deg) = 0.34 x taper. Tooth thickness changes by 2 tan(20 deg) = 0.73 x the radial change.
Nominal L16 sleeve, ideal uniform pressure: 0.245 um/MPa taper, i.e. 0.08 um flank-line deviation per MPa of contact pressure.

### Case 1b: pressure actually delivered by the assumed sleeve (oil -> contact, gap 5 um)

Taper is large because the end lands clamp the sleeve; the band sits at the sleeve's open end (the cup seats on the shoulder, so the sleeve cannot extend below the band).

| variant | oil MPa | band mean um | taper um | u at seat | u at z 8 |
|---|---|---|---|---|---|
| V1 | 10 | 2.0 | 5.1 | -0.3 (not in contact) | 4.8 |
| V1 | 15 | 4.1 | 9.8 | -0.45 | 9.3 |
| V2 | 15 | 7.1 | 11.4 | 0.9 | 12.0 |
| V3 (free open end) | 6 | 7.1 | 4.2 | 8.7 | 4.5 |
| V3 | 10 | 14.2 | 6.6 | 16.7 | 10.1 |
| V4 (L24) | 15 | 5.5 | 17.8 | -1.5 | 16.3 |
| V1, gap 10 um | 10 | 0.2 | 0.5 | no clamp |  |
(full table: `results/case1_sleeve.json`, plot `plots/case1_sleeve_profiles.png`.)
Findings: (a) with lands at the open end, the first 2-3 mm of the band is **not clamped** and the grip rises steeply along the band; V3 (free open end, needs a seal at the seat) clamps the whole band but tapers the other way, ~40-60% of the mean expansion. Either way the **sleeve design needs a deliberately flat contact-pressure profile over z 0..8** (thickness-graded sleeve wall, pressure-balanced lands); the taper is the dominant workholding error, µm-level at any pressure that closes the gap. (b) 10 MPa oil only just closes a 5 um radial gap with V1 (sleeve stiffness with clamped 12 mm active length: ~10 um free expansion at 10 MPa), so a 12.5 um radial loading gap (the 0.05 mm diametral default) cannot be closed at 15 MPa: the sleeve must be thinner/longer-active, or the loading gap small. Sleeve geometry here is my assumption.
Because the cup expands 6.8 um per MPa of contact pressure, with 1-3 MPa contact the whole rim grows 7-20 um; the teeth are cut in that expanded state and shrink back on release (a size/depth offset, calibratable, plus the taper above).

### Case 2: sleeve wall variation (amplitude a, wall = t0 + a cos n theta) and OD roundness (b cos n theta)

Change in clamped-rim radial displacement (um, harmonic amplitude, band mean) relative to the uniform sleeve; V3 at 6 MPa, V1 at 15 MPa. Contact modelled (gap 5 um).

| perturbation | V3 | V1 |
|---|---|---|
| wall n=1, a=2 um | 0.016 | |
| wall n=1, a=5 um | 0.040 | 0.071 |
| wall n=2, a=2 um | 0.013 | |
| wall n=2, a=5 um | 0.033 | 0.073 |
| **sleeve OD / bore mismatch n=1, b=1 um** | **0.94** | |
| **n=2, b=1 um** | **0.97** | 0.97 |
| **n=3, b=1 um** | **0.96** | |

(Amplitudes; peak-to-peak is 2x. A wall error of 2-5 um is 0.1-0.3% of the 1.5 mm wall and moves the cup by only ~0.8-1.5% of the wall error, 0.01-0.07 um.) **The sleeve wall tolerance is irrelevant; what matters is the shape of the sleeve OD (and the cup bore) once clamped: the cup copies it at ~0.95 um per um** (n=1 = eccentricity, n=2 ovality, n=3 trilobe; a contact-pressure picture: p_c variation is small, but with a gap-closed ring the geometry is copied). Hence the cup bore (specified roundness 3 um) and the clamped sleeve OD roundness map straight into the clamped tooth-ring shape: bore/sleeve roundness budget ~1 um each if the error budget is a few um.
Mapping to the released part: the hob cuts a circle about the table axis on the clamped (deformed) rim. After release the rim returns to its free shape, so the **released gear carries the clamped displacement field with reversed sign** (radial error -u(theta,z), tangential error -u_t): n=1 appears as gear runout (Fr), n=2/n=3 as tooth-ring out-of-round and tooth-thickness variation (0.73 x u), the mean as a uniform tooth-thickness/root-depth offset, the z-variation as flank-line error. Errors below the cutting torque grip limit are not recoverable by process correction because they are not repeatable per part.

### Case 3: 10 N hob force at the tooth band (elastic deflection at the load point, um per 10 N)

| support | radial (inward) | tangential |
|---|---|---|
| cup on land only (rim stuck), no radial support | 1.01 (max 1.24) | 0.037 |
| pressurised sleeve V3, tied (constant-pressure oil) | 0.066 | 0.0097 |
| pressurised sleeve V1, tied | 0.025 | 0.0075 |
| sleeve V3, oil locked (inner face rigid) | 0.0012 | 0.0028 |
| plug, zero clearance, zero rigid shift | 0.006 | 0.037 (plug does not resist tangential) |
| plug, 5 / 10 um diametral clearance, land slides | 0.0045 elastic **plus rigid shift 2.5 / 5 um** (cup slides until bore touches plug at the hob side; force reversal moves it back: 2x that) | 0.037 |
| plug, 5 / 10 um diametral, land stuck (friction) | 1.01 (below the 2.5/5 um gap, plug not reached) | 0.037 |

Per N: sleeve 0.002-0.007 um/N, plug-zero 0.0006 um/N, unsupported rim 0.10 um/N. The sleeve tie is a linear (sticking) model of a clamp with contact pressure above the load-induced traction; the cutting torque (10 N x 30 mm = 0.3 N m) is carried by friction, no extra deflection. For a plug the clearance is a **position** error, not a stiffness one: with a free-sliding rim the part sits anywhere within the clearance (radial offset up to C/2 = 2.5 or 5 um) and shifts by up to C/2 with force direction; the tooth ring is then concentric with the table but not with the bore (runout vs bore up to C/2), and any load change of 10 N or more moves it across the whole clearance. With a stuck rim there is no shift, but the 10 N deflection is 1 um (0.10 um/N) until the plug is touched; hob forces of 50-100 N would reach the gap.
Friction needed so the rim stays stuck: F_axial >= F_lateral / mu (mu ~0.15 dry): 10 N lateral needs 67 N; 100 N needs 670 N, which is large for a 0.6 mm wall (see case 4: still only 0.05 um distortion).
**Plain plug + axial clamp: viable only with the clearance near zero (below ~1 um diametral) or if the stuck-rim friction holds and the plug is not needed**; at 10-30 um diametral loading clearance it is not viable for um-level work (offset C/2 = 5-15 um, not repeatable with force direction). The expanding sleeve removes that offset (that is its job) but only the sleeve OD roundness and the pressure profile along the band then matter.

### Case 4: 100 N axial pull-down on the 0.3 mm land

| config | band mean um | taper um | at seat | top axial shortening |
|---|---|---|---|---|
| free cup, rim slides | 0.034 | 0.114 | -0.056 | 0.21 um |
| rim stuck | 0.049 | 0.062 | 0.000 | 0.21 |
| zero-clearance plug | 0.048 | 0.064 | -0.003 | 0.21 |
| sleeve-clamped (tied V1) | -0.001 | 0.005 | -0.000 | 0.12 |

100 N of pull-down distorts the rim by <= 0.1 um (and the shell stays far from buckling: 1 MPa axial stress). Axial stiffness of the wall ~ 480 N/um, so a 2 um seat flatness error closes only with ~1 kN: flatness and perpendicularity of the land matter more than the clamp force (not modelled: partial-contact tilt).

## Mesh checks

- Case 1 axial profile: refined (0.25 mm axial, 4 radial elements): 6.835 vs 6.836 um/MPa, taper 0.242 vs 0.245; 24 elements per 180 degrees: identical; L=6: 4.970 vs 4.951.
- 3D force cases (radial 10 N): 72 x 0.25 mm x 4 radial vs base: cup alone 1.089 vs 1.010 um (+8%), V3 sleeve tied 0.0684 vs 0.0658 (+4%). Quote force deflections to about +-10%.
- Case 2 differences are linear differences of identical meshes; OD copy ratios 0.94-0.97 are not mesh-checked beyond case 1. The wall-variation results (<0.1 um) are below any modelling uncertainty.
- Contact penalty K 1e5 MPa/mm: overclosure ~1e-5 mm per MPa, below result scale.

## Caveats

Sleeve geometry (lands, 12 mm active length, constant-pressure oil) is assumed; case 1/1b/3 sleeve numbers move with it, the direct-pressure case 1 and the plug results do not. Teeth stiffness, hub plate and friction are not modelled (friction only through stuck/slide limits). Hob force distribution is a uniform patch.
