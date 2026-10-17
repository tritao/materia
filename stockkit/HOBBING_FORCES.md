# Hobbing cutting loads (HB10)

`HobbingForces` observes **every completed gash pass**, in start-time order. It
keeps a copy-on-write previous RingStock and integrates the native per-circle
removed angular intervals with `RingGrid.sectorVolume`. Chips already removed by
an earlier gash are not charged again. The observer must start before the first
cut and must not see unrelated stock edits. `HobbingStream.onPass` supplies exact
window endpoints and their interpolated measured poses.

For each window, with removed volume V in mm³, peripheral hob travel L in mm,
and explicitly assumed effective edge width b in mm:

```
A = V/L                         mm²
h = A/b                         mm
Fc = kc1.1 * b * h^(1-mc)        N
Fr = Fc * tan(pressureAngle)     N (assumed separating-force ratio)
TB = Fc * hobRadius/1000         Nm
TC_bound = hypot(Fc, Fr)*workRadius/1000  Nm
E = Fc * L/1000                  J
P = E/windowDuration            W
```

Zero removal gives zero load without evaluating a singular specific force.
The law follows the [Sandvik specific-cutting-force definition](https://www.sandvik.coromant.com/en-gb/knowledge/materials/specific-cutting-force).
The **40CrNiMo at 38–42 HRC coefficients are assumptions**, not values attributed
to that source: kc1.1=2400 N/mm², mc=0.25. The app assumes b=1 mm. Removed volume
alone cannot recover b and h separately. Coefficients, edge width and separating
force need calibration; no tool-wear, rake, minimum-chip/ploughing or speed
correction is included. Travel uses hob rotation at outside radius and neglects
feed/shift velocity relative to peripheral speed.

Samples are window means, represented as rectangular loads. The accumulator
sums overlapping windows at each start and retains only active windows and scalar
maxima. It reports both the largest single-gash mean and the largest overlap sum.
These are **not resolved tooth-force peaks**. Table torque is a conservative
resultant-at-radius bound, not a signed torque or a claim that this bound consumes
additional spindle power. HB11 must use these distinctions when applying loads
or refining wall compliance.

`FlexsplineHobberLoads` reads installed B/C motor continuous torque, rated/max
speed and nominal power. It checks loads with a default 20% reserve and supplies
X radial force and C torque bound to `PlanCheckOptions.resistingEfforts`.
These nonnegative joint loads are conservatively present throughout the checked
position plan and are copied into forked checks. The process-owned B spindle is
checked separately because it is absent from the position polynomial. This API
supports an offline load-informed recheck; it does not automatically apply cutting
forces to MuJoCo or replace HB11's loaded simulation.

The app report shows assumed loads and their basis. Retained stock accounting
includes the observer's previous-stock view; shared storage may be counted twice.

Focused checks use known ring-sector removal, zero/repeated cuts, the
feed^(1-mc) relation at fixed edge width/travel, overlap summation, energy/torque
units, and real reference-hob rake sweeps. A 0.0402 s mid-face plunge into virgin
stock produces an estimated 4.45 kN / 22.2 kW overload. This intentionally rejects
an unsafe starting trajectory; it is **not** the reference CAM entry or evidence
against its normal feed load. No coefficients were adjusted to meet a target.

The complete ideal HB8 reference stroke is the separate `reference` test mode
of `stockkit/tests/hobbing-forces/haxeon.json`. It uses m0.3/z200/8 mm, 0.5 mm/work
rev and 300 m/min, entering below the blank using HB8's computed overrun. It checks
the planned tens-of-newtons target and 20% spindle reserve. Its coarse radial
0.05 mm / axial 1 mm integration is a load smoke check, not a converged force
prediction or ISO inspection. **This full-stroke acceptance is pending.** Run it
through the shared `heavy-run` queue; focused compiler/native checks need no new
native libraries. Ring-stock/rake native sources are unchanged.

## Fast smoke cut

`machinekit/examples/flexspline-hobber/check-quick.sh force` runs a complete
m0.3/z40/1 mm ideal cut at 5 mm/work rev, with four gashes and a coarse grid.
Measured here: 391 windows, 10.5614 mm³ removed, 72.13 N overlapping window mean,
360.66 W and 1.154 Nm hob torque. It passes the 20% spindle reserve check and
chip-volume conservation in about 5 seconds using 113 MiB. This small-part
smoke check complements the pending full reference-stroke acceptance.

The complete reference-stroke and installed-rating check can be run with:

```sh
machinekit/examples/flexspline-hobber/check-reference-force.sh
```

This uses the existing native dependencies (the same `HOBBER_NATIVE_CACHE` and
`HOBBER_STOCK_NATIVE` overrides as the quick runner), compiles only the two Haxe
checks, caps the cut at 4 GiB, and saves provenance, progress, the force report and
installed-rating results under `build/flexspline-reference-force/`. It runs the
entire reference feed stroke; it is substantially slower than the quick fixture.
The force report is printed before acceptance assertions so a failing cut still
retains its measured loads. The machine check reads the report and checks B
power/torque, C torque and X screw thrust against installed continuous ratings
with 20% reserve. X thrust uses the installed screw lead and nut efficiency.
These are static process-load checks with assumed ratings, not an inertia/gravity
or instantaneous tooth-load validation.

The full-stroke report separates mean and varying load for HB11: time-mean radial
force is energy divided by the constant peripheral speed and total feed-stroke
duration, times tan(pressure angle), including air entry and exit. The varying
range is conservatively bounded by 0..measured peak. It is not an observed minimum
or an instantaneous tooth-force peak. HB11 reports the mean thickness shift and
X compensation separately from varying profile/pitch contributions.
