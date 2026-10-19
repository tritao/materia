# Strain-wave gears: research notes

Background for modelling strain-wave (harmonic) gears in MachineKit: how they
work, the equations we need, papers, open-source code and data. Compiled
2026-10-17. This is reference material, not a plan. The flexspline hobbing machine plan is
`machinekit/examples/flexspline-hobber/PLAN.md`.

Today a strain-wave gearhead is a `Gearbox` with a ratio and an assumed
efficiency of 0.85 (`machinekit/motion/Gearbox.hx`, `robotics/CobotJoint.hx`,
`examples/robot-arm/RobotArm.hx`).

## How it works

Three parts:

- **Wave generator (WG).** An elliptical cam pressed into a thin-race ball
  bearing. It is usually the input.
- **Flexspline (FS).** A thin-walled steel cup (or "hat", or a flat ring in the
  pancake type) with external teeth. The WG bends it into a two-lobed shape, so
  it meshes only near the major axis, in two zones 180° apart. About 30% of the
  teeth are engaged at once: high torque density and no backlash.
- **Circular spline (CS).** A rigid ring with internal teeth, usually
  `Z_c = Z_f + 2`.

The pancake type replaces the cup with a flat flexspline and two internal
splines: a stationary one (`Z_f + 2`) and an output one (`Z_f`) that turns 1:1
with the flexspline.

### Kinematics: a three-port differential

Each WG turn moves the FS two teeth relative to the CS:

```
Z_c·ω_CS − Z_f·ω_FS = (Z_c − Z_f)·ω_WG
```

- CS fixed, FS output: `i = −Z_f/(Z_c − Z_f)` (200/202 → −100:1, reversed).
- FS fixed, CS output: `i = Z_c/(Z_c − Z_f)` (→ +101:1).

This has the same structure as a planetary set.

### Flexspline deformation (inextensible-ring theory)

Let δ be the angle of an FS cross-section measured from the WG major axis, and
`r_n` the radius of the undeformed neutral line. For a two-lobe cosine
generator:

```
radial          w(δ)  =  w0·cos 2δ
tangential      v(δ)  = −(w0/2)·sin 2δ          (inextensible: w + dv/dδ = 0)
section tilt    μ(δ)  =  (v − dw/dδ)/r_n = (3/2)(w0/r_n)·sin 2δ
```

- `w0 ≈ κ·m·(Z_c − Z_f)/2`, where m is the module and κ ≈ 1 is the deflection
  coefficient. κ > 1 is positive deflection.
- Sign conventions differ between authors. The set above matches
  `pancake_swgb_designer`.
- With the CS fixed and the WG at angle θ, the mean FS rotation is
  `η = −2θ/Z_f`, and the section angle is `δ = η − θ`.
- Each FS tooth is treated as rigid and fixed to its neutral-line section.
- **Conjugate CS profile** = the envelope (swept union) of one FS tooth over a
  full WG cycle. The trajectory of tooth j is the trajectory of tooth 0 rotated
  by `j·2π/Z_c`, so sweep one tooth and repeat the result around the ring. In
  the output-spline frame of a pancake gear the teeth only wobble, and the
  motion has period π in θ.
- Real WG cams are elliptical, 4-roller or cosine. Their kinematic error
  differs; Gravagno 2016 found the Resàl curve lowest.

### Behaviour a dynamic model must reproduce

Sources: Tuttle, Taghirad, vendor catalogues.

- **Stiffening torsional compliance.** The catalogue gives this as piecewise
  linear: K1, K2, K3 with thresholds T1, T2. Wave-generator bearing radial
  stiffness is the most sensitive stiffness factor (Zhang 2020).
- **Hysteresis and lost motion** at torque reversal, with no backlash.
  Modelled with Bouc–Wen, Maxwell-slip or Dhaouadi.
- **Friction:** Coulomb + viscous + Stribeck. Efficiency depends on speed,
  torque and temperature, and is **asymmetric with power-flow direction**
  (forward drive vs back-driven; Graabæk 2025).
- **Kinematic (transmission) error and torque ripple** at 2× WG frequency,
  with smaller 3× and 4× components. Roughly arcminute-level. Ripple depends on
  input position, not speed.
- **Flexspline fatigue** sets rated torque and life. Bending from the
  deformation plus torsion; alternating stress is about 2× assembly stress.

## Papers

Foundational works were cited from memory; their venues and years are not
checked. Recent works were found and checked on 2026-10-17.

### Foundations

- C. W. Musser, *Strain Wave Gearing*, US Patent 2,906,143 (1959). The
  invention.
- M. N. Ivanov, *Volnovye zubchatye peredachi* ("Harmonic gear drives"),
  Russian textbook. Rigorous FS deformation and meshing kinematics.
- T. D. Tuttle, *Understanding and Modeling the Behavior of a Harmonic Drive
  Gear Transmission*, MIT AI Lab TR-1365 (1992),
  <https://apps.dtic.mil/sti/tr/pdf/ADA259610.pdf>. Also Tuttle & Seering, IEEE
  Trans. Robotics & Automation (1996). **Start here for dynamics.**
- H. D. Taghirad & P. R. Bélanger, "Modeling and parameter identification of
  harmonic drive systems," ASME J. Dyn. Sys. Meas. Control (1998).
- Kircanski & Goldenberg, "An experimental study of nonlinear stiffness,
  hysteresis, and friction effects in robot joints with harmonic drives and
  torque sensors," IJRR (1997).
- Ghorbel, Gandhi & Alpeter, "On the kinematic error in harmonic drive gears,"
  ASME J. Mech. Design (2001).
- Dhaouadi, Ghorbel & Gandhi, "A new dynamic model of hysteresis in harmonic
  drives," IEEE Trans. Industrial Electronics (2003).
- Kondo & Takada, "Study on tooth profiles of the harmonic drive," ASME J.
  Mech. Design (1990).
- S. Ishikawa, US Patent 4,823,638 (1989). The S-tooth / IH profile.
- Kayabaşı & Erzincanlı, flexspline tooth shape optimisation by FEM, Materials
  & Design (2007).
- Tjahjowidodo et al., "Theoretical modelling and experimental identification
  of nonlinear torsional behaviour in harmonic drives," Mechatronics (2013).
  Maxwell-slip model with nonlocal memory.
  <https://www.sciencedirect.com/science/article/abs/pii/S0957415813000706>
- Gravagno, Muciño & Pennestrì, "Influence of wave generator profile on the
  pure kinematic error and centrodes of harmonic drive," Mech. Mach. Theory
  (2016).
  <https://www.sciencedirect.com/science/article/abs/pii/S0094114X16300714>
- Zhang et al., "A refined dynamic model of harmonic drive and its dynamic
  response analysis," Shock and Vibration (2020).
  <https://onlinelibrary.wiley.com/doi/10.1155/2020/1841724>

### Recent and most useful

- **Guida, Bertolino, De Martin, Sorli, "A new computationally efficient model
  of the non-linear dynamics in harmonic drive reducers," Mech. Mach. Theory
  209 (2025)**, doi:10.1016/j.mechmachtheory.2025.105992. Repository:
  <https://iris.polito.it/handle/11583/2998667>.
  - A lumped dynamic model with kinematic error, Bouc–Wen hysteresis on the
    flexspline torsional stiffness, internal friction and tooth meshing.
  - Also models defects, wear, assembly errors and poor lubrication. Aimed at
    model-based design and health monitoring; validated against literature
    data.
  - **The closest template for our dynamic model.**
  - **PDF not obtained** (the repository is behind a Cloudflare check);
    download it by hand. The same group has a 2023 paper on wear, hysteresis
    and tooth cracks.
- **S. G. Graabæk, *Sensorless Torque Estimation on Robot Joints with Strain
  Wave Gears*, PhD thesis, Univ. Southern Denmark / Universal Robots (2025),
  CC BY 4.0.**
  [PDF](https://portal.findresearcher.sdu.dk/files/284857335/S_ren_Goddiksen_Graabaek_PhD_Thesis.pdf),
  [page](https://portal.findresearcher.sdu.dk/en/publications/sensorless-torque-estimation-on-robot-joints-with-strain-wave-gea/),
  [companion paper](https://portal.findresearcher.sdu.dk/en/publications/full-range-torque-estimation-on-drivetrains-with-strain-wave-gear/).
  - Real UR joint data.
  - Efficiency is asymmetric in power flow. The thesis models it with a
    "wedge" friction model and fixes the earlier wedge model's inability to
    simulate all four drive and back-drive quadrants.
  - A torsional-compliance estimator using datasheet parameters beat a
    momentum observer by 39% RMSE, and by 49% when fitted to data.
  - **Not yet read in detail.** Mine it for the wedge equations and the
    parameter values.
- **Schuller et al. (DLR + Schaeffler), "An experimental study of the
  sensorized strain wave gear RT1-T…," Front. Robot. AI 11 (2024)**,
  doi:10.3389/frobt.2024.1416360, open access:
  <https://pmc.ncbi.nlm.nih.gov/articles/PMC11341302/>.
  - Size 17, 100:1, strain gauges on the flexspline flange.
  - Two-inertia joint model.
  - Output-side friction: 0.24 Nm Coulomb + 0.17 Nm·s/rad viscous.
  - Sensing ripple 0.083 Nm at 50 rad/s, mostly at 2× input.
  - No stiffness, hysteresis or efficiency values.
- **Lian, Liu, Li, Li, "Research Progress on Mechanical Properties and Fatigue
  Failure of Harmonic Drive Flexspline," Sensors 26 (2026) 4204**,
  doi:10.3390/s26134204, open access:
  <https://pmc.ncbi.nlm.nih.gov/articles/PMC13363806/>. A review of profile
  methods, flexspline stress and fatigue.
  - Cylinder length is the most sensitive parameter: longer lowers bending
    stress but costs volume and torsional stiffness.
  - Fatigue life drops sharply at tooth widths of 5–8 mm and changes smoothly
    from 8 to 12 mm.
  - Larger root fillets lower stress concentration.
  - A smaller bending coefficient k lowers stress; a larger k gives taller
    teeth and a lower pressure angle.
  - Longitudinal CS modification cut contact pressure by 69.6% (He 2024).
  - Shot peening adds up to 66% life. A well-designed FS reaches a bending
    safety factor above 2.5.
  - Analytical profile papers to follow: Dong 2020 (double-circular-arc via
    centrodes), Wang 2015 (envelope and improved-kinematics conjugate
    equations), He 2024 (double-arc common tangent + CS modification), Zhen
    (profile straight from the neutral curve).
- **Liu et al., "A Novel Rapid Design Framework for Tooth Profile of
  Double-Circular-Arc Common-Tangent Flexspline in Harmonic Reducers,"
  Machines 13 (2025) 535**, open access.
  - The industrial-style tooth: convex arc + tangent line + concave arc.
  - Defaults: m = 0.5, Z_f = 100, Z_c = 102, w0 = 0.5. Wall thicknesses:
    ring s = 0.01·m·Z_f, cup t = 0.6·s.
  - Implemented in `brandonkxo/HarmonicGearTeethGenerator` (below).
- Xie et al., "A novel harmonic drive with the full conjugate tooth profile,"
  Proc. IMechE C (2024), <https://doi.org/10.1177/09544062241262654>. Both
  profiles derived from Camus's theorem; includes FE and a prototype.
- "A Novel Cycloid Tooth Profile for Harmonic Drive with Fully Conjugate
  Features," Actuators 14 (2025) 187, open access:
  <https://www.mdpi.com/2076-0825/14/4/187>. Envelope method.
- Composite cycloidal harmonic-drive profile, IJPEM (2025),
  <https://link.springer.com/article/10.1007/s12541-025-01321-7>. Notes that
  the CS profile assumes the theoretical FS deformation, so real deformation
  differences cost performance.
- Concave–convex active profile design by curvature vectors (ScienceDirect,
  2025): <https://www.sciencedirect.com/science/article/pii/S259012302504397X>.
- Transmission error of a space-robot harmonic reducer (Space, 2025),
  <https://spj.science.org/doi/10.34133/space.0233>. Static error from
  machining and assembly; dynamic error from variable stiffness and friction.
  Not read (blocked).

### Less useful

- Sorrentino et al., physics-informed NN friction for high-ratio harmonic
  drives (ergoCub), Humanoids 2024, <https://arxiv.org/abs/2410.12685>.
  Learned; no code.
- arXiv 2310.15875 (data-driven transmission error). Withdrawn.
- Wear and backlash degradation (Eng. Failure Analysis, 2022),
  <https://www.sciencedirect.com/science/article/abs/pii/S1350630722005878>.
  A damaged WG–FS interface cuts stiffness to as little as a quarter of
  nominal.

## Open-source code and data

None of these is a mature library, and there is no strain-wave model for
MuJoCo, Drake, Gazebo or Modelica. **Most repos have no licence: read them for
the method, implement from the papers, copy no code.**

### Geometry

- [erdemu/pancake_swgb_designer](https://github.com/erdemu/pancake_swgb_designer)
  (Python, no licence, [live](https://erdemu.github.io/pancake_swgb_designer/)).
  **The best template.**
  - `swgb/geometry/envelope.py` is the swept-union envelope above, with the
    single-tooth sweep and rotate-array trick.
  - `wavegen.py` has ball-race, roller and plain-cam generators.
  - Also an involute module, bearing and material tables, strain and
    printability checks, DXF/SVG export and tests.
- [brandonkxo/HarmonicGearTeethGenerator](https://github.com/brandonkxo/HarmonicGearTeethGenerator)
  (Python/DearPyGui, no licence). Implements Liu 2025 (double-circular-arc),
  with conjugate CS tooth and radial and longitudinal modification tabs.
- [brandonkxo/MITE_HarmonicDrive](https://github.com/brandonkxo/MITE_HarmonicDrive)
  (SolidWorks/STEP, no licence). An open actuator rated 75 Nm continuous: FS
  158T, CS 160T, strain-web torque sensing. A real reference design.
- [abssimon/harmonic-drive-tooth-profile-generator](https://github.com/abssimon/harmonic-drive-tooth-profile-generator)
  (Go, no licence). S-tooth from circle segments, with an HTML mesh animation.
- [OlegDydy/harmonic-drive](https://github.com/OlegDydy/harmonic-drive)
  (JS, GPL-3.0). The rolling-element wave-gear variant (no flexspline).

### Dynamics

- [earosenfeld/robot-test-bench](https://github.com/earosenfeld/robot-test-bench)
  (Python, MIT). Generic BLDC + harmonic joint, Stribeck friction, system ID.
- [BurhanMuhyiddin/Harmonic-Drive-Modelling-with-UR5-Manipulator](https://github.com/BurhanMuhyiddin/Harmonic-Drive-Modelling-with-UR5-Manipulator)
  (Simulink, no licence). Nonlinear spring and damper; the PDF report is a
  worked example.
- [Danielsong001/Torque-estimation---MATLAB](https://github.com/Danielsong001/Torque-estimation---MATLAB).
  Torque from flexspline deformation calibration.

### Data

- [GDUT-YANGGUO/Harmonic-Drive-Dataset](https://github.com/GDUT-YANGGUO/Harmonic-Drive-Dataset)
  (GD-HDD, CC BY-NC 4.0, Baidu Pan downloads). Vibration, servo and metrology
  data from axis 5 of an 8 kg industrial robot; 12 operating conditions; 7
  single and several compound faults. Use the healthy runs to check ripple and
  transmission-error spectra. Validation only; do not redistribute.
- Graabæk thesis (above, CC BY 4.0). The best openly licensed cobot-joint
  parameters.
- Harmonic Drive catalogues and engineering data: K1/K2/K3, T1/T2, hysteresis
  loss, lost motion, starting and back-driving torque, efficiency curves.
  Enough to parameterise sizes 14–32.

### Hobby and 3D-printed designs

- [HasslerEngineering/3D-Printed-Strain-Wave-Gear](https://github.com/HasslerEngineering/3D-Printed-Strain-Wave-Gear)
- [ttsalo/Armstrong-SCARA](https://github.com/ttsalo/Armstrong-SCARA)
  (LGPL, `flexspline.scad`)
- [RianPolMacCionaoith/3D-Printable-NEMA-17-Harmonic-Reducer](https://github.com/RianPolMacCionaoith/3D-Printable-NEMA-17-Harmonic-Reducer)
  (MIT)
- Reported failure: thin PETG/TPU flexsplines snap at the transition from teeth
  to cup wall.
- No academic papers on printed flexsplines; only patents (US 11,839,927 metal
  AM flexsplines; US 11,680,629 low-cost wave generators).

## Manufacturing

Researched 2026-10-17. Vendor claims are marketing and have not been checked
independently.

### Part requirements

- **Flexspline.**
  - Material: SNCM439 / 40CrNiMo(A), 38–42 HRC (through-hardened, not case
    hardened).
  - Wall typically under 1 mm, and it must be uniform around the circumference.
  - Module 0.15–1.5 mm (mainstream ≤ 1.25), 100–400 teeth (mainstream
    100–200), outside diameter 25–140 mm.
  - Gear quality ISO 1328 class 3–4 (MHI says class 3 or better), pitch error
    in single-digit µm, runout under 5 µm.
  - Pitch accuracy matters most: it drives indexing accuracy and torque ripple.
- **Circular spline.** Rigid ring, internal non-involute teeth, often alloy or
  nodular iron. It must stay concentric with its mounting datums.
- **Wave generator cam.** Non-round (not a true ellipse; engineered profile),
  profile tolerance about 3 µm. A thin-race flexible ball bearing is pressed
  on.

### Process chains

- **Flexspline, cut route** (EMAG's chain; Chinese patents agree):
  1. Forged blank or bar, or thin-walled tube (a CN patent route that saves
     turning).
  2. Soft turn while the wall is still thick.
  3. Harden and high-temperature temper, deep-freeze (liquid nitrogen), then
     low-temperature temper.
  4. Hard turn the thin wall.
  5. Oil-bath temper for stress relief.
  6. Hone the bore; it is the gear-cutting datum.
  7. **Hob the teeth** with a carbide hob on a hydraulic expanding mandrel
     (runout 3–5 µm; collets distort the wall).
  8. Deburr, clean, shot-blast (shot peening also adds fatigue life).

  Gear grinding is impractical at these modules (wheel making is hard, profile
  grinding is slow), so the hobbing machine sets final quality.
- **Flexspline, formed route** (CN113399935B, Patsnap):
  1. Rough turn.
  2. Spheroidise to 10–15 HRC.
  3. **Form the teeth by spinning.**
  4. Quench and temper, finish machine, nitride.

  Near-net forming is where people expect humanoid-scale volumes to go.
  Hobbing alone leaves Ra 3.2–6.3 (CN102489960A).
- **Circular spline.**
  - Turn, then generate the internal teeth: **gear shaping** (most common
    today, slower and less accurate), **power skiving** (rising; cylindrical
    tool synchronised with the part), broaching, or **wire EDM**
    (prototypes, hardened parts, odd tooth forms; leaves a recast layer).
  - Then heat treat and finish.
  - Non-involute profiles need custom hobs and skiving tools, which are
    expensive but unavoidable for accuracy.
  - Research: an error-free cylindrical skiving tool for circular splines,
    Mech. Mach. Theory 2021,
    <https://www.sciencedirect.com/science/article/abs/pii/S0094114X2100392X>.
- **Wave generator.**
  - Complete machining in one setup: turn the bore and outside diameter plus
    non-round turning or grinding of the cam, by an NC-interpolated path with
    a linear-motor X axis.
  - Patent US11808337: measure the circular spline, flexspline and bearing
    first, then grind the cam to suit (selective fitting).

### Machines

- **EMAG** (Koepfer heritage):
  - [K 160](https://www.emag.com/industries-solutions/workpieces/robotics/flexspline/)
    hobbing: module 0.3–2.5, part diameter ≤ 100 mm.
  - K 300: module 0.5–4, diameter ≤ 200 mm, flexsplines up to 140 mm.
  - Both use a polymer-concrete base and hydraulic mandrels.
  - [VLC 350 GT](https://www.emag.com/blog/en/high-precision-complete-machining-of-wave-generator/)
    vertical turn-grind for the WG cam: linear-motor X (120 m/min), internal
    grinding spindle 26–75 k rpm, external grinding wheel Ø400 mm, dressing
    spindle, 3–5 µm.
- **Mitsubishi Heavy Industries Machine Tool, GE15FRplus** small-module hobber
  (MHI Technical Review 57-3, 2020,
  <https://www.mhi.com/technology/review/abstract-57-3-30>). **The best public
  design reference for a flexspline hobber.**
  - Cutting force is only tens of newtons, so the design is about rotational
    accuracy, not rigidity.
  - Hob spindle: 8000 rpm direct drive, double-supported, HSK two-face arbor,
    balance faces, coolant held at machine temperature. Vibration ≤ 1 µm p-p,
    NRRO ≤ 0.5 µm.
  - Table: direct-drive motor chosen for low cogging (pole/slot choice, skewed
    magnets), full closed loop on a high-precision encoder, **non-contact oil
    seal** (contact-seal drag spoiled rotation accuracy), NRRO 0.3 µm.
  - Result on m0.5, 100T, 15 mm face, carbide Ø32 hob, 800 m/min, 0.5 mm axial
    feed: **ISO class 1 pitch** (single 0.4 µm, cumulative 2.0 µm), as good as
    grinding, at 1/3 the cycle time of their general GE15A (which reaches ISO
    class 5, 3500 N, 2000 rpm).
- **Others not checked for harmonic parts.** General small-module hobbers and
  power-skiving machines exist from Liebherr, Gleason, Präwema, DVS/Pittler,
  Samputensili and Kashifuji, and as skiving cycles on turn-mill centres
  (INDEX, WFL, DMG MORI, Mazak). Liebherr's LC 180/280 starts too coarse
  (module up to 6). Chinese reducer makers (Leaderdrive, Laifual) do not
  publish their equipment; their HK IPO filings may.
- **Low-volume and research routes:** wire EDM, metal-AM flexsplines
  (US 11,839,927 / 11,198,181), bulk-metallic-glass cast flexsplines
  (US 10,151,377, US 11,859,705), 3D-printed polymer.

### Machine architectures (added 2026-10-19)

- **EMAG Koepfer K 160 is a horizontal hobber**, listed under EMAG's horizontal
  gear hobbing machines: module 0.3–2.5, workpiece Ø ≤ 100 mm, length ≤ 1000 mm.
  The hobber plan's original HB-D1 wrongly called our vertical layout a K 160
  match.
  <https://www.emag.com/machines/gear-cutting-machines/gear-hobbing-machines/horizontal-gear-hobbing-machines/koepfer-k-160.html>
- **Mitsubishi GE15A is a vertical hobber**, about 7 t (15,400 lb), radial travel
  140 mm, axial 250 mm, table ≤ 300 rpm (dealer listing,
  <https://tramar.machinehub.com/?p=6632>). It is the vertical,
  moving-column-type reference our hobber follows (HB-D1 revised, 2026-10-19).
- **Turn-and-hob in one machine exists** (DMG MORI hob option on NLX/ALX/CTX TC,
  INDEX turn-mill gear cutting, EMAG VLC 250 WF / VSC 400 WF, Liebherr complete
  machining, Felsomat FHC green turning + hobbing). It is mostly medium precision:
  WTO's hobbing unit for turning centres quotes ISO 1328 **class 8**.
  <https://www.wto-tools.com/Marketing/Newsletter/WTO/WTO_Abwaelzfraeseinheit/NEWSLETTER_07_2020_KV1_0_US.html>
  Precision flexspline lines keep hard turning, honing and hobbing on separate
  machines. Our split: the lathe plan (`examples/lathe/PLAN.md`) makes the
  blanks; the hobber cuts the flexspline teeth; the skiver
  (`examples/circular-spline-skiver/PLAN.md`) cuts the circular-spline teeth.
- **Power skiving needs its own machine.** Both spindles are fast: about 1,500 rpm
  table and 6,000 rpm tool on our reference ring, against 15 rpm hobbing. That
  takes a cantilevered tool inside the ring, a crossed-axis angle of 15–30°, and a
  much faster gearbox. At 1,500 rpm, 1 µs of sync jitter ≈ 33″ ≈ 4.9 µm at r 30 mm.
  Dedicated makers: Liebherr, Präwema, DVS/Pittler, Mitsubishi, Gleason.

### Double-flexspline reducer (Ling et al. 2023)

Ling, Zhao, Xiao, Zhou, Ma, He, Liu, "A Novel Strain Wave Gear Reducer with
Double Flexsplines", *Actuators* 12 (2023) 313, doi:10.3390/act12080313. CC BY.
Authors are from AICI Technology, KUKA Robotics Guangdong and SIAT; the design is
patented by the first author (US 11,614,153).

- **Idea.** The circular spline is made slightly flexible, and an adjustable
  **limit ring** with a tapered contact surface preloads it. Backlash is set at
  assembly and can be re-tightened as teeth wear, so machining tolerances can be
  looser.
- **Measured.** Five size-14, ratio-80 units (rated 10 Nm, peak 61 Nm):
  - K1/K2/K3 ≈ 2.56/2.51/2.63 Nm/arcmin;
  - lost motion ≈ 11″, hysteresis ≈ 37″;
  - transmission error < 20″, repeatability ±4.9″.
  Without the ring: lost motion 44.8″ and hysteresis 87.9″. With it: 11.6″ and
  32.8″.
- **Bench.** Heidenhain encoders on input and output (±1–2″), HBM torque
  sensors, Kollmorgen motors, Wittenstein load gearbox. A template for our
  end-of-line test bench.
- **Weaknesses.**
  - No tolerance-relief study, no wear or life test, no efficiency or starting
    torque reported.
  - The preload raises stresses (flexspline 304 → 339 MPa, circular spline
    153 → 218 MPa, contact +32 MPa).
  - The comparison with Harmonic Drive catalogue values sets measured typical
    values against guaranteed maxima.
- **For us.** It adjusts backlash, not pitch accuracy, so accurate teeth are
  still needed. A compliant-circular-spline-plus-preload-ring option belongs in
  the tooth-design plan's later work. Check the patent before using the idea in a
  product.

## Suppliers and prices (collected 2026-10-19)

From web listings and marketing pages, not experience with these companies. All
specs are claims; request test certificates for the actual units.

### What a hobbing C table must state (send with every request)

| Item | Requirement |
|---|---|
| Use | Slow precision rotation under an electronic gearbox, ≤ 100–300 rpm |
| Size | Bearing ≈ Ø200–250 mm class, parts up to Ø140 mm |
| Runout | Radial and axial, total and non-repeatable, at the table face: ≤ 1–2 µm (class 5–6); ≤ 0.5 µm for class 3 |
| Tilt stiffness | Stated, with the mandrel and part stack ≈ 100–150 mm above the face |
| Encoder | Absolute, ≥ 2²³ counts; accuracy ±1–2″, two heads preferred; **interpolation error** stated; EnDat 2.2 or BiSS-C |
| Smoothness | Cogging torque, and torque and speed ripple at 10–20 rpm |
| Seal | Type and drag torque. Non-contact labyrinth preferred (contact-seal drag spoiled the GE15FRplus's rotation accuracy) |
| Through-bore | Room for a rotary union to actuate the mandrel automatically |
| Clamp | Holding torque |
| Thermal | Cooling, and heat at duty |
| Data | Inertia, torque constant, continuous and peak torque, mass, mounting pattern, test certificate for the unit |

Positioning and repeatability figures alone do not qualify a hobbing table.

### Complete direct-drive tables

- **Foshan Demat (德玛特)**, <https://www.maxfara.cn>, info@maxfara.cn,
  +86 18927758400, Foshan Nanhai. Machine-tool oriented; publishes spec tables
  (as images on product pages).
  - RTV vertical DD tables (air 6 bar clamp, IP67):

    | Model | Table Ø | Bore | Cont./peak torque | Rated/max speed | Load | Positioning / repeat |
    |---|---|---|---|---|---|---|
    | RTV140-030 | 125 | 30 | 8.65 / 38 Nm | 200 / 300 rpm | 20 kg | ±5″ / ±2″ |
    | **RTV210-050** | **200** | **40** | **54.9 / 223 Nm** | 200 / 234 | 50 kg | ±5″ / ±2″ |
    | RTV210-070 | 160 | 40 | 65 / 309 | 200 / 234 | 50 kg | ±5″ / ±2″ |
    | RTV230-070 | 320 | 30 | 65 / 309 | 120 / 150 (water, oil clamp) | 300 kg | ±5″ / ±2″ |
    | RTV385-050 | 356 | 60 | 390 / 735 | 60 / 100 (water) | 400 kg | 10″ / 4″ |

    **RTV210-050 is the first candidate for the hobber C table.** IP67 suggests
    contact seals; ask about drag and a low-drag option. No runout,
    interpolation-error or ripple data is published.
  - RTVS high-speed vertical DD tables (water-cooled):
    RTVS160-030 (Ø125, 19.1/38 Nm, 1000/2000 rpm, ±5″/±5″),
    **RTVS230H-030** (Ø280, 37/55 Nm, 1500/2000 rpm, ±10″/±6″),
    RTVS310-030 (Ø400, 133/286 Nm, 500/600 rpm, ±5″/±2″).
    **RTVS230H-030 is the candidate skiving work spindle.**
  - DDF water-cooled frameless DD motors, IP67: DDF145, DDF160, DDF195 and
    DDF230, stack lengths 30–200; continuous 14–507 Nm, peak 22–934 Nm. Claimed
    ±3″ positioning and ±1″ repeat in their system. For a built-up table or the
    A swivel.
  - airUPRT air-bearing rotary table: 70 nm axial and 100 nm radial runout, ±1″
    repeat, zero-cogging DD motor. For class 3 or a gear-measuring station; load
    and stiffness to confirm.
  - Also linear motors and modules (for the wave-generator cam's non-round X),
    robot-joint frameless motors, DD swivel heads and five-axis cradles.
- **Shenzhen ZRT**: DD tables for CNC, ±15″ absolute / ±2″ repeat claimed,
  ≈ US$1,700–4,300, MOQ 1. A budget probe unit.
  <https://direct-drive-motor.en.made-in-china.com/>
- **Jiangsu 3F (锋桦)**: DD motors Ø60–490 mm, 1–1200 N·m, ±10″ / ±0.5″
  claimed. Fits the A swivel. <https://www.3fgearmotor.com/products/dd157.html>
- **Qingdao Zhihe (智和)**: LDE outer-rotor DD motors, ±1.5″ repeat.
  <https://zcoe-dd.com/>
- **HIWIN (Taiwan)**: torque-motor rotary tables (water-cooled motor,
  integrated precision bearing, absolute encoder, clamp). The regional
  reference. <https://www.hiwin.com/torque-motors-rotary-tables.html>
- TJR (Taiwan) DD tables; Nanjing Qitu (≈ US$4,000 listing); Zolix (lab stages,
  not machine-tool loads).
- Tianjin Betek, a Chinese gear-machine builder, uses a DD C table on its YK3132Z
  six-axis hobber with NUM drives.
  <https://www.num.com/references/tianjin-betek-machinery-manufacturing-co>

### Components (built-up table)

- **Bearings (YRT / crossed roller, P4/P2 claimed):** ZWZ (Wafangdian), Dalian
  DYZV, and a Luoyang cluster (Hongyuan, Monton, LKP, Yufanda). One source cites
  ≈ 1.5 µm rotation accuracy without test conditions. Installation (seat
  accuracy, zero-clearance fits) matters as much as grade.
- **Ring encoders:** Changchun Yuheng Optics (长春禹衡), <https://www.yu-heng.cn/>.
  A broker report quotes the company claiming over 90 % of Heidenhain's
  performance at 20–30 % lower price. No ±1″ absolute ring spec was found.
- **Frameless torque motors:** Shenzhen Mosrac (强和, F/U series, custom),
  <https://www.mosrac.cn/>; Kinco (步科, low-cogging claim); also Han's Motor,
  Leadshine, Wolong (mostly for robot joints).

### Price anchors and estimates

- Found: ZRT US$1,700–4,300 (new); Nanjing Qitu ≈ US$4,000 (new); Nikken
  DDRT-200X ≈ US$12,500 (**used**); HIWIN TMN71E ≈ US$1,300 (used, single
  listing). Air-bearing stages (PI, Aerotech) are quote-only.
- Our estimates (order of magnitude, not quotes):
  - Demat RTV210-050 ≈ US$3,000–8,000;
  - RTVS230H-030 ≈ US$5,000–12,000;
  - DDF motor ≈ US$500–3,000 each;
  - airUPRT ≈ US$10,000–30,000.

  Western or Japanese equivalents are typically several times these. Budget
  extra for drivers (≈ US$500–2,000 each, if not included), cables, rotary
  union, shipping, duty and VAT, and test certificates.
- **Prototype hobber bought hardware: ≈ US$25,000–65,000** (rough), before our
  own machining, assembly and metrology:
  - C table $3–8k;
  - hob motor spindle $3–10k;
  - A swivel $1–3k;
  - three ball-screw axes with guides, scales and servos $5–15k;
  - controller with electronic gearbox $3–15k;
  - mineral-cast base $3–8k;
  - mandrel and hob $2–6k.

  The controller and spindle choices swing it most.

### Approach

1. Send one request for quotation with the specification table above to Demat
   (RTV210-050, RTV210-070, RTVS230H-030, DDF195/DDF145, airUPRT), HIWIN
   (reference) and ZRT (budget). Include the missing-data items: runout,
   interpolation error, ripple at 10–20 rpm, seal drag, stiffness, thermal,
   rotary union, test certificate.
2. Price the built-up alternative: a ZWZ/DYZV or Luoyang P4/P2 bearing, a Yuheng
   ring encoder, a Mosrac or Kinco frameless motor, and our own housing.
3. Buy two candidates (Demat or HIWIN, plus ZRT as a budget probe). Measure them
   on a small bench: encoders on both sides, a runout probe, speed ripple at
   15 rpm. This is the first piece of the end-of-line bench.
4. Enter the measured data into the hobber model's bought-table form and rerun
   the HB11 table sources.

## Open questions

- Get the Guida 2025 PDF and extract the model equations and parameters.
- Read the Graabæk thesis: the wedge efficiency model and the UR joint
  parameters.
- Collect catalogue stiffness, hysteresis and efficiency data for sizes 14–32
  as the default parameter table.
- Quotes and test data for the C table candidates (Suppliers and prices).
