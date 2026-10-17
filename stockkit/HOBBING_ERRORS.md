# Hobber error sources and budget (HB11)

`FlexsplineHobberErrors.budget` runs a fresh physical rake sweep and HB4
inspection for the baseline, each selected source, and all sources together.
Installed part values supply mandrel eccentricity, bearing runout, encoder
errors, arbor/tool/outboard runout and tool profile error. Source reports retain
part names, numerical assumptions, errors basis and source text. The hob recipe
now carries `profileError` (default assumed peak 0.001 mm).

`HobbingPoseOffset` separates work-frame and rotating tool-frame translations:

- Mandrel eccentricity is an offset fixed in the rotating work frame. Table
  radial runout is treated the same way, using half its TIR.
- Table axial runout is an assumed cosine per table turn, amplitude TIR/2.
- Hob radial runout is a fixed rotor-frame offset, so its spatial displacement
  rotates with B rather than merely oscillating the machine X coordinate.
  Arbor, tool and outboard are independently selectable. The outboard source is
  a parallel-axis approximation; it omits shaft tilt and bending.
- Hob axial runout is an assumed cosine per hob turn along the actual tilted
  hob axis, amplitude TIR/2.
- A single-readhead encoder uses the first-order angular eccentricity e/R sine.
  Graduation error uses an explicitly assumed fourth harmonic with the part's
  angularAccuracy as peak amplitude; this is not a measured graduation map.
- Profile error warps the physical rake vertices before tool creation. The
  supplied sinusoidal radial waveform repeats per axial pitch and supports zero
  rake. It is an assumed error shape, not a catalogue profile map. Arbitrary
  caller warps must preserve valid planar rake regions. Native tool validation
  remains active.

Runout fields are explicitly interpreted as **TIR**, while `eccentricity` is
centre-offset amplitude. Error sources are applied after motion interpolation;
recorded offsets also survive interpolation. Measured motion knots are retained
by the error-job runner. Gash scheduling still follows the actual B trajectory;
these sources do not modify B or impose an ideal C relation on recorded motion.

## Synchronization

Supply `HobbingErrorSource.measuredSync` with HB7's measured C-minus-baseline
following error, preserving real encoder, loop and prediction behaviour. The
baseline must exclude that same error: do not inject sync twice into recorded C.
Encoder errors already included in a runtime trace must likewise be disabled as
separate sources unless the trace is regenerated without them. A per-source
runtime study requires rerunning HB7 with that source disabled, not merely
subtracting a presumed scalar error.

The latency factory is a **diagnostic** model, not an alternate servo simulator.
It evaluates r*(B(t-delay)+delay*omega(t-delay)-B(t)) with prediction enabled,
and omits the velocity term otherwise. Its history functions must cover t-delay.
At constant speed, lag without prediction is a fixed phase displacement; it does
not by itself produce cumulative pitch variation. Changing speed creates varying
following error. Quantization, loop dynamics and saturation belong to HB7's
actual observations, not an assumed latency multiplier.

## Wall compliance

`RingCompliance` uses the n=2, in-plane thin-ring energy approximation:
I=width*wall³/12; ring stiffness=9*pi*E*I/R³. For w=d*cos(2 theta), integrating
0.5*E*I*curvature² around the ring yields this generalized stiffness. Geometry
is mm and E is N/mm². Radius is the centroid of the wall below the tooth roots.
See [Roark's ring conventions](https://jackson.engr.tamu.edu/wp-content/uploads/sites/229/2023/03/Roarks-formulas-for-stress-and-strain.pdf)
for ring radius, section and modulus units; the n=2 reduction here is an explicit
approximation, not a general structural solution.

A **separately stated support stiffness** is added in parallel. It is mandatory
input, with provenance; zero selects an unsupported diagnostic ring. Clamping
pressure does not determine contact stiffness. The source uses HB10 radial load
history divided by total stiffness as a quasistatic local wall recession. The
caller states steel modulus (e.g. 210000 N/mm² assumed); it is not inferred from
a density-only material. Higher modes, teeth, cup bottom, contact lift-off,
clamping distortion, shaft bending and dynamics are omitted. An unsupported ring
must not be presented as the actual expanding-mandrel installation.

This is one-way coupling to a stated HB10 load history, which can be its maximum
as an explicit conservative load case. It does not recompute chip forces and
contact compliance iteratively inside every perturbed pass. Supported compliance
needs measurement or an external structural/contact model before HB12 grading.

## Budget and validation

`gearkit.ErrorBudget` reports signed changes from the baseline for each ISO
parameter and identifies the largest absolute contributor's part. Grades are
not summed. For profile/lead traces, cumulative pitch and radial probe values,
it checks the combined residual against the sum of individual trace residuals.
The default tolerance is **0.1 µm + 25% of the summed absolute trace envelope**,
per channel. Nonlinear interactions are reported, not forced into additive
metrics. Radial probe coordinates are converted from mm to µm for this check.
Actual ISO eligibility and inspection warnings remain GearMeasurement's.

Focused checks cover rotating-frame offsets, B-periodicity, encoder harmonics,
constant-speed latency/prediction, varying-speed error, supported ring scaling,
native profile-warp acceptance, translated-involute Fr≈2e and first-harmonic
pitch, and additive/nonlinear synthetic trace budgets. These validate the model
and reporting, not a full physical error acceptance.

The standalone `stockkit/tests/hobbing-errors/haxeon.json` has a separate
`physical` mode: baseline, mandrel, radial/axial hob and combined coarse gear
cuts, with measured Fr, profile-form and trace-superposition checks. The fast
m1/z20/2 mm case uses four gashes and an explicit pitch-to-tip diagnostic
inspection. Its grid covers the lead evaluation window. It **passes**: combined
Fr=7.896 µm for 4 µm mandrel eccentricity, Fp=8.505 µm, profile form=3.889 µm,
and trace superposition within the stated tolerance. Runtime is about 51–55 s,
using 126 MiB. Run it with:

```
machinekit/examples/flexspline-hobber/check-quick.sh errors
```

The full reference-part source budget, measured HB7 source-disable studies and
supported wall calibration remain pending. The coarse mode is not an ISO
certification or proof that a part meets HB12. HB10's full-reference queued
artifact remains separate.

### Assumed installed-mandrel support sensitivity

`machinekit.hobbing.MandrelRadialSupport` estimates radial support from explicit
assumed design parameters on `ExpandingMandrel`: sleeve wall 1.5 mm, end-ring span
16 mm, 8 mm tooth band, steel E=210000 N/mm² and nu=0.3, oil cavity 0.5 mm and
bulk modulus 1500 N/mm². The envelope CAD does not resolve this sleeve or cavity.

For w=d cos(pi z/L) cos(2 theta), integrating isotropic shell bending energy gives
k_s = D pi R [pi^4/(2L^3) + 9L/(2R^4) + (4-nu)pi²/(R²L)], where
D=Et³/[12(1-nu²)]. This is an assumed simply supported mode, not a calibrated
finite-shell solution. Membrane prestress, actual end restraint and local modes
are omitted. Fully seated steel contact uses harmonic half-space tangent
c=E q/[4(1-nu²)], q=sqrt(4/R²+pi²/L²), and
k_c=c pi R integral_band cos²(pi z/L) dz. Combine sleeve/contact compliance in
series. The half-space approximation and conforming contact are assumptions;
roughness, contact lift-off and finite wall effects need verification. The
[periodic elastic-contact treatment](https://www.frontiersin.org/journals/mechanical-engineering/articles/10.3389/fmech.2020.00045/full)
(section “Sinusoidal Profile”, reduced modulus for two contacting bodies)
supports the harmonic pressure/displacement scaling. Applying it to this finite
sleeve/contact is an engineering approximation.

A uniform fluid pressure cannot restore a first-order volume-preserving n=2
ovalisation: its integrated volume change is zero. The separate uniform oil
compression diagnostic k=K 2piRL/cavity is **not** added to n=2 support. A hoop-only
pressure-to-close-fit estimate checks seating provisionally; it omits end
restraint and does not prove full contact.

The sleeve's axial region is now explicit. The reference sleeve spans 71..87 mm
in mandrel coordinates; assembly transforms place the toothed band at 75..83 mm,
with a 0.025 mm radial fit gap. The report checks actual assembled blank placement
and bore radius, the available diametral expansion, and a provisional pressure
screen. These are assumed design dimensions, not a measurement of hardware. If
the band is outside the sleeve or contact is not seated, mandrel stiffness is
zero and the cup wall's own RingCompliance governs. The reference blank currently
models the annular toothed rim, not a detailed cup-bottom structure.

Run `machinekit/tests/hobber/build/host/main.hl support-sensitivity` with the
HashLink/native environment from the quick runner, or use the reference-force
runner for a report driven by its measured load statistics. The standalone case
uses a provisional assumed design load Fc=100 N, Fr=36.4 N: this is not a safe
bound on the running cut. It separately
bounds the unknown mean and varying range, rather than presenting these as
measured statistics. Support factors remain 0.3, 1, 3. The full-stroke force report
computes the time mean from energy / constant peripheral speed / stroke duration,
including air entry/exit. Variation uses a conservative 0..measured-peak envelope;
it is a range bound, not a measured minimum, instantaneous tooth peak or a
reconstructed force waveform.

Mean deflection is checked against an **assumed physical tooth-thickness tolerance
of +/-5 um**, and its magnitude is reported as an X-infeed compensation. HB3's
0.2 um figure is solely a numerical simulation tolerance. It is not a physical
deflection budget. Mean thickness change is 2 tan(alpha) times mean displacement.
The varying displacement range gets conservative profile/pitch geometry bounds
of twice the radial range and 2 tan(alpha) times the radial range respectively.
The wall is explicitly allocated 20% of class-6 Falpha and single-pitch fp limits
inside the eventual HB11 budget. At m0.3 these ISO formulas are engineering
extrapolations. Passing this wall allocation does not establish the combined
error budget or an ISO class.

The report retains sufficient effective and nominal (including x0.3 uncertainty)
stiffness requirements. If its bounds do not meet the allocations, acceptance
stays open pending a justified design or measured stiffness. A conservative
bound exceeding an allocation is not proof that an inspected cut fails.
`wall-cuts` also runs baseline and three physical source cuts in a small
m1/z20/2 mm integration fixture. It verifies the stiffness-to-thickness response;
that fixture is separate from the full reference budget and uses a constant load.


### Parallel source sweeps and force provenance

`HobbingErrorJob` accepts an explicit thread count (default 4); the reference
`FlexsplineHobberErrors.budget` adapter defaults to 8. The sweep tolerance is
unchanged: force-only tolerance selection never propagates into metrology.
Every source run logs its thread count, sweep/path tolerance, stock spacing,
source ids and elapsed time. Native rake workers own complete pose-segment
chunks; their arcs are sorted before interval union, with no parallel floating
reduction. Tests compare endpoints bit-for-bit and preserve provenance at 1, 4
and 8 threads; force tests additionally compare every gash load and final stock.
The sampler caches affine endpoints without changing arithmetic or refinement
criteria; pre/post stock and per-pass load hashes are checked on two fixtures.

`HobbingForceHistory` optionally records every radial/tangential gash-window mean
and supports a radial history callback for wall sweeps. Its time-weighted mean,
standard deviation and range describe the sum of overlapping rectangular windows.
Successive per-pass maximum/RMS steps describe individual gash means, a different
quantity. The wall report identifies which radial statistic drives each mean,
peak, varying-range and per-pass displacement. Fc is never substituted for Fr.

The original fine force cut was deliberately stopped by the user at 40,400 windows,
without a completed report. The optimized complete stroke is the authorized HB10
reference, with a CSV/settings sidecar and short-cut tolerance convergence below
1%. No fine full-stroke comparison or missing fine variation is claimed.
The combined reference-source budget still requires its own physical validation;
no full-reference per-source runtime target is claimed from the small fixture.

With the optimized complete force history, engaged-time mean Fr=4.9563 N drives
mean X recession; peak Fr=16.990 N drives seating and the varying radial range.
The separately reported whole-stroke mean Fr=3.0357 N includes air and is used for
energy/full-run comparison, not the mean tooth-thickness compensation. The x0.3
case bounds mean thickness shift at 0.142 um and varying profile/pitch at
1.337/0.487 um, inside the assumed +/-5 um and allocated 1.9/1.5 um limits.
These wall-allocation checks **pass dependent on the assumed support/contact
model** and the short-cut force convergence basis. The full HB11
combined-source budget is not established by this wall-only screen.

The `physical-fine` integration fixture runs the baseline, three sources and
combined cut at the reference **0.000025 mm sweep tolerance / 8 threads**, in
**51.92 s** total. Its stock sampling remains diagnostic, not ISO-qualified.

### Complete hobber reference study

`machinekit/examples/flexspline-hobber/check-reference-budget.sh all` runs baseline,
13 installed sources and combined cuts on the HB8 feed trajectory, at 8 threads,
0.000025 mm sweep / 0.0000015625 mm path tolerances, 0.0015 mm radial / 0.04 mm
axial sampling. Each completed inspection is saved before the next case. Progress,
settings and binary/input hashes are retained under `build/flexspline-reference-budget`.
The study is running; no completed reference budget is claimed yet.

The native HB7 gearbox exports independently regenerated prediction enabled and
disabled observations through its assumed 200 Hz unloaded servo plant. Installed
B encoder resolution is checked by the assembly consumer. This does not replace
loaded full-machine observations. The frozen HB10 radial window history uses
centred continuous ramps at jumps; integral and peak must stay within 0.1% of the
original statistics. This regularization is explicitly assumed, with no modal or
force/compliance feedback claim. Nominal support uses the assembled rim coverage
check and the existing assumed sleeve/contact design estimate.

`HobbingErrorJob` sorts input motion knots in O(n log n), then binary-searches each
pass's local knots. The SDK's insertion-sort lowering must not process a large
concatenated trace. A physical regression checks exact parameter/profile/lead
agreement against direct sweeps with unsorted, duplicate and outside-run knots.
