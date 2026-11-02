# Flexspline mandrel: loads, pressure window, accuracy budget, supplier requirements

Reproduce every number with `python3 workholding_loads.py` (stdlib only). Status: hand calculation
(thin ring / Lame, series sleeve-and-cup spring). The FEM study is separate and should confirm or replace
items 2 and 3 (especially sleeve edge effects, which a ring model cannot see).

## 0. Inputs checked against the code

| Item | Task statement | Code | Verdict |
|---|---|---|---|
| m, z, face width | 0.3, 200, - | `GearSpec(0.3, 200, 8, ..)` FlexsplineHobber.hx:45 | b = 8 mm, d = 60 mm |
| Bore | O58.05 H5 | mandrel OD = 2*(rootRadius-0.6) - 0.05 = 58.00; root radius 29.625 -> 58.05 | matches; H5 (+0.013) is not in the model |
| Wall | 0.6 | bodyOutside 59.25 -> 0.6 | matches |
| Stop land | O58.65 | `stopDiameter = diameter + 0.6` = 58.65 | matches (0.3 mm radial land) |
| Clamp length | max(16, b+8) = 16 | `sleeveLength`, `sleeveStart = workSeat` | matches; band is the first 8 mm of the 16 mm |
| Table face to seat | 135 | `workSeat = 363 - TABLE_FACE` | not re-derived |
| Table speed | up to 300 rpm | 300 rpm is the table `maxSpeed`; the job runs **14.9 rpm** (`tableRpm`) | 300 rpm only matters for emergency stop (about 0.03 N m, negligible) |

## 1. Required holding

Evidence: `build/flexspline-worktree-artifacts/build/flexspline-reference-force-fast/loads.json` (full reference
stroke, 92151 windows, built from commit 6caeb51; I did not re-run it). Kienzle kc1.1 = 2400 N/mm2, mc = 0.25 are
documented as uncalibrated assumptions in PLAN.md.

- Tangential Fc peak 46.7 N; radial Fr peak 17.0 N (= Fc tan 20 deg); `maxTableTorqueBound` **1.49 N m**
  (= hypot(Fc, Fr) x 30 mm); hob torque 0.75 N m, power 233 W.
- No axial force exists in the model. **Assumption:** Fa = 0.3 Fc = 14 N (hob lead angle ~0.55 deg, feed
  0.5 mm/rev, so the true value is probably a few N; the load that matters is upward pull-off, since downward
  load goes into the stop).
- The forces are overlapping gash-window means, not tooth peaks. Sensitivity with peaks 2x higher is shown below.
- Friction 0.08 (oily, worst) to 0.15; bore area A = pi x 58.05 x 16 = 2918 mm2.

| | Required (SF 2) |
|---|---|
| Holding torque | **3.0 N m** (5.96 N m if tooth peaks are 2x window means) |
| Axial (pull-off) holding force | **28 N** |
| Contact pressure to hold, mu 0.08 / 0.10 / 0.15 | 0.46 / 0.36 / 0.24 MPa (0.91 / 0.73 / 0.49 with 2x peaks) |

Capacity is 6.8 N m per MPa of contact pressure at mu 0.08. The pressure the cup actually needs is below 1 MPa.
MACHINE_DESIGN's "below 5 N m" is not derived from anything here; use 3.0 N m.

## 2. Pressure window

Cup wall 0.6 mm under the root, E = 210 GPa, yield taken as 1000 MPa (40CrNiMoA, 38-42 HRC).

- Radial expansion of the cup wall: **6.8 um per MPa** of contact pressure (Lame and thin ring agree).
  Hoop stress: 49 MPa per MPa.
- 0.46 MPa (minimum to hold): 3.1 um radial, hoop 22 MPa. 1.0 MPa: 6.8 um. 2.0 MPa: 13.6 um, hoop 98 MPa.
  10 MPa: 68 um radial, hoop 489 MPa (yield SF 2.0). Yield is never the limit below about 10 MPa.
- **The current 2 um limit cannot be met and still hold the part**: 2 um is 0.29 MPa, below the 0.46 MPa needed
  at mu 0.08.
- **What the limit should be.** A uniform expansion e releases as a uniform scale change of the toothed ring.
  At 2 MPa that is strain 4.7e-4: tooth thickness changes 0.22 um, circular pitch 0.44 um, pitch/root diameter
  28 um. The only part that matters is the diameter, a pure size offset, compensated by cutting the root
  diameter oversize by the diametral expansion at the working pressure (2 u_c; 6-20 um in the window below).
  The tooth shape is cut about the table axis, so a uniform or tapered (axisymmetric) expansion does not
  create helix or profile error. Replace the single "clampingDistortion 2 um" with:
  - non-uniform (lobed) part of the clamped bore displacement, n >= 2: **<= 1.5 um peak**;
  - uniform part: any value up to 10 um radial (1.5 MPa), measured/compensated by hob depth, repeatability +-1 um radial.
- Clearance take-up (series model: sleeve wall as thin tube, then cup in contact):
  p = (P ks - g) / (ks + kc), with ks = r^2/(E ts) (2.67 um/MPa for **ts = 1.5 mm, assumed** typical sleeve wall;
  4.0 for 1.0 mm; 2.0 for 2.0 mm), kc = 6.8 um/MPa, g = radial clearance.
  - Collapsed diametral clearance 0.010-0.028 mm (OD 58.040 -0.005/0 against H5 bore) at P = 7.1 MPa gives
    p = 0.52 (loosest) to 1.48 MPa (tightest), cup expansion 3.6-10 um radial. That is the target window.
  - The **model's** 0.050 mm clearance with 10 MPa gives only 0.18 MPa: it **would not hold the cup** (needs
    11-13 MPa oil for p = 0.5-1.0). The 0.05 mm gap in `FlexsplineHobber.hx` is too large; use 0.02 mm nominal.
  - Sensitivity dp/dP is 0.28 (ts 1.5): P regulated to +-3% (+-0.2 MPa) keeps p within +-0.06 MPa.
  - Dry fire (no cup): 15 MPa expands a 1.5 mm sleeve 40 um radial, hoop 290 MPa; elastic, but the sleeve must be
    proof-rated and the control must not pressurise without a seated cup.
- Resulting **contact window 0.5 to 1.5 MPa**, oil 5-9 MPa for the supplier's clearance range.

## 3. Accuracy budget (ISO 1328-1:2013 formulae, d = 60, mn = 0.3, b = 8; mn is below the 0.5 mm table range)

| um | fp | Fp | Fr | Falpha | Fbeta |
|---|---|---|---|---|---|
| Class 5 | 5.2 | 11.4 | 10.2 | 4.2 | 6.8 |
| Class 6 | 7.3 | 16.1 | 14.5 | 5.9 | 9.6 |

(Fr = 0.9 Fp as I recall the 2013 text; confirm against the standard. Formulae are in the script.)

Workholding gets **50% of each tolerance** (25% of the variance), the rest is table, spindle, hob, thermal and cutting.

| Error | Class 5 allowance | Class 6 | How it arises |
|---|---|---|---|
| Fr (TIR cup-bore datum to teeth) | 5.1 um: mandrel static TIR 3.0, re-clamp centring 1.5, lobing 3.0 (bore roundness 3 um conforms to the round mandrel, springs back as tooth runout), RSS 4.5 | 7.2: 4.0 / 2.0 / 3.0, RSS 5.4 | cup axis is located by the mandrel, teeth are cut about the table axis |
| Fbeta (cup axis tilt vs table axis over b = 8) | 3.4 um: tilt effect <= 2 um over 8 mm (0.25 mrad) | 4.8: 3 um (0.38 mrad) | mandrel axis alignment to its mounting flange and seat |
| fp / Fp | 2.6 / 5.7 um: lobing gives about a/n = 0.75 um tangential, fine | 3.7 / 8.0 | pitch errors are not workholding-dominated if there is no slip |
| Falpha | negligible from workholding (profile is hob geometry) | | |
| Taper of the expansion along the 8 mm band | no effect on teeth (axisymmetric); keep <= 3 um on 16 mm to protect the 1.5 um lobing figure | | see 2 |

Cocking before clamp: 0.028/16 = 1.75 mrad (14 um over b) if the cup tilts in the clearance; the clamp must
pull it true, so the seat land (flatness 2 um) and sleeve (perpendicular, 3 um) both locate it.
The model's `eccentricity = 0.004 mm` is ambiguous (eccentricity or TIR); as an eccentricity it is 8 um TIR,
over the 5.1 um class 5 workholding budget.

## 4. Automation sizing

- **Air-to-oil booster, 6 bar:** area ratio 16:1 gives 7.2 MPa at 5 bar usable, 0.9 efficiency (union and tube
  drop). Maximum 10 MPa needs 22:1; the 15 MPa maximum rating in the model needs 33:1 (6 bar x 0.9: 28:1).
  Oil volume about 85 mm3 (expansion + 0.5 mm cavity compression): a tiny booster. Trapped-oil temperature
  drift is about 0.04 MPa/K at the sleeve.
  Monitor with an air-side pressure transmitter on the stationary side (oil = ratio x air at equilibrium) plus
  a stroke or pressure-drop check. Fail-safe: a lock/check valve on the oil side, so loss of air during a cut
  does not release the cup.
- **Drawbar / cone collet alternative:** normal load = p pi D L = 2.9 kN at 1 MPa. Cone half-angle 3-7 deg,
  mu 0.1: pull 450-660 N at 1 MPa (900-1300 N at 2 MPa), i.e. a 36-43 mm air cylinder at 5 bar. Spring-clamp
  with air release is fail-safe. Con: a segmented collet gives lobed contact on a 0.6 mm wall (violates the
  1.5 um lobing limit); only an uninterrupted sleeve driven by a cone is acceptable.
- **Robot lead-in:** assume robot plus gripper position error 0.1 mm, plus 0.014 mm radial clearance: nose lead-in
  radial >= 0.5 mm, 15 deg half-angle (2 mm axial); the cup bore edge only has a <= 0.1 mm break, so the lead-in
  must be on the mandrel; the cup lands on a 0.3 mm land, so the robot needs force-limited (<= 20 N) vertical
  placement.
- **Seat check** (no holes in the 0.3 mm land):

| Option | Pros | Cons |
|---|---|---|
| A. Air/eddy sensors on a post facing the cup's inner end plate (cup end at 38 mm, mandrel top at 18 mm: add a post to 34 mm) | direct axial position, no moving seals, resolution about 1 um, 3 sensors at 120 deg also give tilt | cup datum-to-end-plate tolerance (+-0.02-0.05 mm) must be small or taught per lot; oil film |
| B. Spring plunger with LVDT/inductive on the end plate | simple | moving part in oil, side loads, preload up to 2 N |
| C. Air through the sleeve-cup clearance | no extra hardware | detects only "on the sleeve", not axial seat or tilt |
| D. Booster stroke/pressure curve | free, already needed | indirect, cannot see debris on the land |
| E. Electrical land contact | direct land contact | neat oil insulates; unreliable |

Recommended: A with 3 sensors, plus D as plausibility check, and a robot force stop on placement.

## 5. Requirements for the supplier (one page)

Part: thin-wall steel cup, bore O58.05 H5 (58.050/58.063), wall 0.6 mm under the teeth, 40CrNiMoA 38-42 HRC.
Teeth are cut (neat oil) while the cup is clamped; table speed 15 rpm in cut; loading by robot.

| # | Requirement | Value | From |
|---|---|---|---|
| R1 | Collapsed sleeve OD (diametral clearance to the bore 0.010-0.028 mm) | O58.040 -0.005/0 | 2, 3 |
| R2 | Cylindrical hold zone, from the seat upward | 16 mm, sleeve starts at the seat | 0 |
| R3 | Torque the clamp must transmit at the bore (SF 2 included, mu 0.08) | >= 3.0 N m, and pull-off >= 28 N, at **contact pressure 0.5-1.5 MPa** over the full clearance range | 1, 2 |
| R4 | Free diametral expansion of the sleeve (no cup) | >= 0.045 mm at <= 9 MPa oil; proof 15 MPa, no yielding; sleeve wall about 1-1.5 mm | 2 |
| R5 | Non-uniform (lobed) clamped displacement of the bore | <= 1.5 um peak; sleeve edge effects must not taper the contact within the 8 mm band more than 3 um | 2, 3 |
| R6 | Mandrel OD runout to the mounting pilot | <= 3 um TIR (4 um class 6) | 3 |
| R7 | Re-clamp centring repeatability | <= 1.5 um TIR over 50 cycles | 3 |
| R8 | Axis perpendicular to mounting face; seat land flat | <= 2 um over the hold zone (0.25 mrad); land flat 2 um | 3 |
| R9 | Seat land | 0.3 mm radial land, outer O58.65, 2 mm high stop, nothing else may touch the cup face or the bore edge | 0 |
| R10 | Operation | pressurised by 6 bar air (5 bar usable) through a rotary union in the O40 bore; booster ratio set so oil = 5-9 MPa at 5 bar; regulation +-3% | 4 |
| R11 | Fail-safe | holds with air lost (lock valve or spring clamp); no pressurisation without a seated cup (sleeve is not rated for dry fire) | 4 |
| R12 | Seat check | non-contact sensors to the cup inner end plate (3 at 120 deg) or equivalent, resolution <= 2 um, signal to the PLC; no holes in the land | 4 |
| R13 | Robot lead-in | nose lead-in >= 0.5 mm radial, 15 deg; mandrel stays inside the hob keep-out below the seat (0.3 mm) | 4 |
| R14 | Low drag | rotary seal drag as low as possible; state it in N m | MACHINE_DESIGN |
| R15 | Surface | sleeve OD Ra 0.4, compatible with neat cutting oil | MACHINE_DESIGN |

## Things in the model that look wrong or stale

1. Collapsed clearance 0.050 mm diametral (`outside = diameter - 0.05`) is double the loading need and, with
   10 MPa and a 1.5 mm sleeve, gives only 0.18 MPa contact: no holding (2).
2. `clampingPressure` 10 MPa (`maxPressure` 15 MPa) are oil pressures; they do not set a 2 um clamp distortion.
   `clampingDistortion` is an input, not computed from pressure. The 2 um limit is the wrong quantity (2).
3. MACHINE_DESIGN says the mandrel protrudes "maximum 30 mm"; the recipe gives `protrusionAboveSeat = 16 + 2 = 18`
   mm (still cleared by the 35 mm robot transfer height).
4. MACHINE_DESIGN's "below 5 N m" holding torque: required is 3.0 N m (1.49 bound).
5. `eccentricity = 0.004 mm` is likely too large for class 5 if it is an eccentricity (3).
6. The force model has no axial force, uses window means (not tooth peaks) and assumed Kienzle constants;
   the evidence is the Oct 17 run, not re-run for this note.
7. A segmented collet alternative is a poor fit to a 0.6 mm wall (lobing).
