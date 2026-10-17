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
