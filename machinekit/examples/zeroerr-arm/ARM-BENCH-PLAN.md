# Joint and connection bench plan

This is the next physical-validation plan for the original 110T/70T arm. No
hardware is ordered, supplier inquiry sent, fixture fabricated or test performed.
It follows the [standalone requirements and evidence gates](../reference-arms/ARM-DESIGN.md).

The [module and mass audit](MODULE-MASS-AUDIT.md) supplies the current register,
source conflicts, drawing materials and load witnesses. Measure the J2 moving
housing and upstream output separately; whole-module Appendix 3 data cannot
resolve that split by itself.

## Freeze the test article

Start with the original shoulder module family, **110T**, because its assumed
continuous torque and moving-side mass dominate the current load screen. Confirm
the delivered revision, complete code, ratio, brake/encoder options, torque-speed-
temperature curves, allowable bearing loads and electrical/ESI requirements before
selecting the article or defining energized test loads. Nominal 100 is a model
assumption, not a completed purchase specification.

Retain the supplier J2 mounting and moving-side convention. In this assembly its
output flange is upstream and the housing moves with the downstream arm. The
fixture, load lever, mass bookkeeping, command sign and harness must reflect that
arrangement. A generic fixed-housing joint test is not automatically equivalent.
The previous 80T wrist bench request is a separate variant study.

## First, inspect the connection without power

Use a dimensioned, revision-matched interface coupon based on the original
supplier mounting surfaces. Record pilot/dowel fits, fastener engagement, access,
alignment and repeatability of reassembly. Review thread stripping, preload,
slip, support stiffness and load paths before loading it. Establish fixture and
load-catch capacity from the confirmed article/load case; the current CAD is not
a machining release.

## Then measure the joint and connection

| Measurement | Evidence to collect | Decision it informs |
| --- | --- | --- |
| Mass and geometry | Weighed moving parts, measured lever/CoG and actual flange/mount datums | Replace exterior-volume mass surrogates |
| Reversal and lost motion | Calibrated applied torque and external angular/displacement traces | Joint/connection error budget |
| Static deflection and slip | Torque versus displacement, unloading/reversal and reassembly repeats | Separate joint compliance, connection compliance and fixture compliance |
| Holding and brake behavior | Delivered-drive procedure, actual load, position drift, current and temperature | Holding/duty limits; brake rating remains separate from servo torque |
| Motion duty | Low-speed commissioning followed by a defined representative standalone duty, with current/voltage/temperature/time traces | Coupled torque-speed and thermal model |
| Harness travel | Measured cable/air routing and bend/strain behavior through approved travel | Physical joint window and serviceability |

Choose load steps and motion limits only from the confirmed article, fixture
capacity and supplier procedure. Record serial/hardware/firmware revisions,
ambient conditions, calibration/measurement resolution, fixture baseline and
uncertainties with each trace. Do not infer continuous capability from gearbox
momentary/average allowances.

## Acceptance remains to be allocated

The 0.03 mm repeatability benchmark applies to the complete arm at the TCP; it
cannot be assigned wholesale to one joint or demonstrated by FK residuals.
Allocate backlash, connection, elastic, thermal and metrology budgets before
setting pass/fail thresholds. Likewise, ideal tube stiffness does not establish
whole-arm TCP stiffness.

Use the measurements to update mass/inertia/compliance and duty models, then
review the next joint and full-arm test. Whole-arm repeatability/accuracy,
controller calibration and collaborative safety behavior require their own
validation. CNC tending integration follows the arm-level baseline and these
engineering decisions.
