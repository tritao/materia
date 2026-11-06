# Physical gear inspection

How the hobber's parts are measured. `gearkit/METROLOGY.md` grades sampled numerical geometry; this note
covers the physical instruments, what each can certify, and the order to build or buy them.
Prices are estimates unless marked as quotes. Decided 2026-11-04; nothing here is bought yet.

## Tolerance budget

ISO 1328-1:2013 formulas for the reference flexspline (m 0.3, z 200, d 60 mm, b 8 mm; see `PLAN.md`).
Module 0.3 is below the standard's range (m ≥ 0.5), so these are extrapolated values, as in `gearkit`.
Values in µm.

| Parameter | Class 6 | Class 5 | Class 3 |
|---|---|---|---|
| fp | 7.3 | 5.2 | 2.6 |
| Fp | 23.5 | 16.6 | 8.3 |
| Fr (0.9·Fp) | 21.1 | 14.9 | 7.5 |
| ffα | 7.3 | 5.2 | 2.6 |
| fHα | 5.9 | 4.2 | 2.1 |
| Fα | 9.4 | 6.6 | 3.3 |
| ffβ | 8.2 | 5.8 | 2.9 |
| fHβ | 7.6 | 5.4 | 2.7 |
| Fβ | 11.2 | 7.9 | 4.0 |

Measurement uncertainty should be at most a quarter of the tolerance (4:1). The tightest parameter is
fHα, so the required uncertainty is about 1.5 µm at class 6, 1.0 µm at class 5 and 0.5 µm at class 3.
Fine-pitch work outside ISO 1328's range is normally specified to DIN 58405; drawings and lab orders
must name the standard and the extrapolation.

## Chosen setup

1. **Backlit telecentric station** (built in-house): profile, pitch, runout and tooth thickness of
   straight spur teeth on both splines, plus the wave-generator cam profile and outlines of turned parts.
2. **Hand gauges**: gauge pins or balls (over-pin size), bore or air gauges, ball micrometer (cup wall),
   surface plate with height gauge, and a TESA GT21-class inductive comparator (also used on the hobber
   for mandrel and part runout; its 0.63 N force rules out flank contact).
3. **Actuator end-of-line bench**: motor encoder against output encoder gives transmission error and
   lost motion; a load arm or torque sensor adds torsional stiffness; no-load torque gives friction.
4. **External lab**: first-article certification, helix (lead), flank topography, housing geometry (CMM),
   and certification of a master gear.

A contact gear checker (generating method, scanning probe) is deferred until volume, class 3 or
customers require in-house lead measurement.

### What the station can and cannot certify

| Class | Verdict |
|---|---|
| 6 | Pass/fail on fp, Fp, Fr, profile and thickness (fHα ratio ≈ 6:1 at ~1 µm uncertainty) |
| 5 | Pitch, runout and thickness yes; profile slope borderline (≈ 4:1), only after calibration against a lab-certified part |
| 3 | No; needs ≤ 0.5 µm (lab or a future contact checker) |

It never measures helix: the shadow is the outermost material across the face width, so helix
deviation and taper merge into the profile result. Lead is held by the process (hobber geometry and
setup) and checked by the lab at first articles and periodically.

## Backlit station

**Principle.** A collimated backlight sits behind the part and a telecentric lens with camera in front,
both on the part axis. The silhouette of a straight spur gear viewed along its axis is the transverse
tooth profile. The part stands on a precision rotary table and is indexed a few teeth per image.
- The circular spline is an open ring: no obstruction.
- The flexspline cup works if the body behind the tooth band is smaller than the root diameter (normal,
  because the wall is thinned there); the closed bottom is then inside the outline.

**Optics sizing (example, to be confirmed with real parts).**
- Sensor: 20 MP, 2.4 µm pixels (about 13.1 × 8.8 mm).
- Lens: 2× bi-telecentric with matching telecentric illuminator.
- Object pixel 1.2 µm; field of view about 6.6 × 4.4 mm.
- At r 30 mm the field spans about 12.6° of rim: about 7 tooth pitches (πm = 0.94 mm), tooth depth
  about 0.6–0.7 mm, so one tooth of overlap per image and about 30 images per part.
- Subpixel edge repeatability about 0.1 px, roughly 0.12 µm. Accuracy is set by calibration, not pixels.

**Requirements and risks.**
- **Axis alignment.** A tilt θ between the view axis and the part axis widens the shadow by up to b·θ.
  For 0.5 µm over b = 8 mm, θ ≤ 60 µrad (≈ 12″). Align with a rotating precision cylinder or pin gauge
  and calibrate the residual.
- **Grazing reflections.** Over an 8 mm deep tooth, light reflects off the flanks near grazing
  incidence and biases the edge outward. Match the illuminator to the lens aperture, consider a
  dark or matte flank only as a diagnostic, and calibrate edge bias with a gauge pin of the same length
  and finish.
- **Depth of field.** It is smaller than the face width, so the edge forms over a long tunnel. Choose
  the focus plane by test (camera-side end or mid-face) and keep it fixed per part type.
- **Part ends.** Burrs and chamfers at the tooth ends change the silhouette; parts are deburred and
  measured in a stated state.
- **Datum.** Measure on the same arbor datum as cutting (bore, see the mandrel specification), not on
  an arbitrary centre. The flexspline's loose shape differs from its clamped shape by up to
  `lobedDistortion` (1.5 µm).
- **Table.** Pitch uses the table angle: Fp/4 at class 5 is about 4 µm, about 28″ at r 30 mm. A
  ±5″ encoder is enough for classes 5–6.

**Software.**
- Subpixel edge extraction per image; pixel-to-mm and lens distortion from a chrome-on-glass grid.
- Transform to part polar coordinates using the calibrated image position of the table axis and the
  table angle; stitch the overlapping images.
- Feed `gearkit`: implement `GearRings` from the silhouette, giving the material angular intervals at
  each radius for one axial position. `GearMeasurement.fromRings` then evaluates fp, Fp, Fr, profile and
  thickness. Helix fields must report as not measured (never zero), with a warning.
- Non-involute teeth (commercial S-tooth, later double-arc) need a custom nominal profile; until then
  they are compared to a measured reference, not graded.

**Cost estimate:** telecentric lens and illuminator €1–3k, camera €1–2k, plus a table with encoder
(reuse of the motion stack) and a rigid stand. Software is the main effort.

**Off-the-shelf alternative.** A Keyence LM-X (±0.7 µm single-field accuracy in high-precision mode,
manufacturer figure) is this station bought ready-made; gear evaluation would still run in our
software on its exported edge points. Check it with a demo measurement on the reference gear before
building.

## Reference parts

- **Size-25, ratio-100 strain-wave gear, component type** (flexspline z 200, circular spline z 202,
  wave generator, no housing), from Leaderdrive or Laifual. Commercial size 25 is pitch diameter
  63.5 mm (m ≈ 0.3175), close to our 60 mm/m 0.3. Teeth are probably non-involute, so it serves gauge
  repeatability, vendor demos and lab comparison, not involute grading.
- **Involute m 0.3 master gear, lab-certified.** Used for calibration and grading. Once the hobber
  cuts, one of our own parts, certified by the lab, can take over.

## Other methods considered

| Method | Verdict |
|---|---|
| Contact gear checker (generating method) | Deferred. Needs an air-bearing table (e.g. Dynametic airUPRT), exposed scales (Renishaw RESOLUTE or Heidenhain LIF; sealed Yuheng JFT has ≤ 4 N drag and no published interpolation error), a probe and evaluation software |
| Chinese gear measuring centre (Golp G-20/G-30, Harbin) | Possible purchase; rated from m 0.2. Verify on our parts: stylus size, internal teeth, custom profile, accuracy |
| Double-flank roll tester | Cheap production check (composite error, runout, functional thickness); needs a master gear |
| Laser triangulation, confocal | Flanks too steep, gap too narrow; only for tip runout and outside diameter |
| DLP fringe projection | Line-of-sight and specular problems; useful for burrs, chamfers, cup shape and hob wear |
| Focus variation (Alicona) | Full flank topography, but €150k+; a lab service |
| Micro-CT | Several µm; gross checks only |
| On-machine probing on the hobber | Runout and gross errors only; too warm and dirty to certify |

## Probe roadmap (only if needed)

For in-house lead or class 3, build a 1D, then 2D, flexure probe for the generating method:
- a wire-EDM parallelogram flexure (17-4PH or Ti-6Al-4V), stiffness about 0.1–1 mN/µm, range
  ±25–50 µm with hard stops, eddy-current damping, a magnetic kinematic breakaway mount, bought ruby
  styli Ø0.125–0.3 mm;
- sensing by a free-core LVDT (Solartron MD/SM, Microstrain NANO-LVDT), which adds no force or
  friction; commercial gauge probes (TESA GT21, Mahr P2004) push 0.6–0.75 N and are unusable on flanks;
- our own EtherCAT board: sine excitation, simultaneous-sampling 16–18-bit ADC, ratiometric
  synchronous demodulation, sampled at SYNC0 with the axis scales;
- calibration against a scaled axis, a ceramic reference sphere and a certified master gear.

Phases: P0 bench test (noise < 20 nm, hysteresis < 50 nm), P1 our own electronics, P2 second axis and
first profile scans on the station's table, P3 (optional) 3D.

## Open items

- Confirm the drawing tolerance class and the standard named on drawings (ISO 1328 extrapolated or
  DIN 58405).
- Identify a fine-module gear lab for first articles and the master gear.
- Buy the reference gear; ask Keyence and Golp for demo measurements (requests not yet drafted).
- Dynametic air-bearing table and linear-stage questions wait for their reply on the hobber C table.
