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
prediction or ISO inspection. **Full-stroke acceptance passed with the optimized settings below.** Run it
through the shared `heavy-run` queue; focused compiler/native checks need no new
native libraries. Ring-stock/rake native sources are unchanged.

## Fast smoke cut

`machinekit/examples/flexspline-hobber/check-quick.sh force` runs a complete
m0.3/z40/1 mm ideal cut at 5 mm/work rev, with four gashes and a coarse grid.
Measured here: 391 windows, 10.5614 mm³ removed, 72.13 N overlapping window mean,
360.66 W and 1.154 Nm hob torque. It passes the 20% spindle reserve check and
chip-volume conservation in about 5 seconds using 113 MiB. This small-part
smoke check complements the full reference-stroke acceptance.

The complete reference-stroke and installed-rating check can be run with:

```sh
machinekit/examples/flexspline-hobber/check-reference-force.sh
```

This uses the existing native dependencies (the same `HOBBER_NATIVE_CACHE` and
`HOBBER_STOCK_NATIVE` overrides as the quick runner), compiles only the two Haxe
checks, caps the cut at 4 GiB, and saves provenance, progress, the force report and
installed-rating results under `build/flexspline-reference-force-fast/`. It runs the
entire reference feed stroke; it is substantially slower than the quick fixture.
The force report is printed before acceptance assertions so a failing cut still
retains its measured loads. The machine check reads the report and checks B
power/torque, C torque and X screw thrust against installed continuous ratings
with 20% reserve. X thrust uses the installed screw lead and nut efficiency.
These are static process-load checks with assumed ratings, not an inertia/gravity
or instantaneous tooth-load validation.

The full-stroke report separates mean and varying load for HB11: time-mean radial
force is energy divided by the constant peripheral speed and total feed-stroke
duration, times tan(pressure angle), including air entry and exit. The recorded varying
range comes from the overlapping radial window history (including zero in air).
It is not an instantaneous tooth-force range
or an instantaneous tooth-force peak. HB11 reports the mean thickness shift and
X compensation separately from varying profile/pitch contributions.


### Force-only performance gate

The reference runner now builds StockKit in an isolated
`build/flexspline-force-parallel/native` directory. It never rewrites the libraries
of a running reference job. `check-reference-force.sh benchmark` runs two short
fixtures with reference geometry (virgin mid-face engagement and a common pre-cut
mid-face band). Every trial uses the same initial stock. The pre-cut stock uses
an explicitly recorded 1 um seed tolerance; it is a convergence fixture, not a
claim of a fully steady HB8 entry.

For each fixture, fine sweeps (0.00005 mm) at 1 and 4 threads must have identical
stock endpoint/provenance hashes and per-pass load hashes. Candidates 0.0002,
0.0005, 0.001 and 0.002 mm are tested at 4 threads. Peak tangential force, mean
radial force, radial standard deviation, and successive per-pass max/RMS radial
steps must each change by <1%. The largest passing candidate is selected for
forces only; the runner's default stock grid remains 0.05 mm radial / 1 mm axial.
Every force report includes threads, sweep/path tolerance, geometry, force
components and optional per-pass CSV. Metrology keeps its fine tolerance.

Initial validation selected **0.002 mm at 4 threads**: across both fixtures, peak
force changed at most **0.100%**, mean radial force **0.050%**, and per-pass RMS
variation **0.461%**. The completed optimized full stroke is the user-authorized
HB10 reference; the original fine run was deliberately stopped at 40,400 windows. These numbers do not validate arbitrary feeds, stock grids or hob
geometries. The timings and hashes are saved in the calibration report.

The default evidence directory is now `build/flexspline-reference-force-fast/`;
the retained original fine run stays in `build/flexspline-reference-force/`.
`compare-force-runs.py` requires matching stroke scope, duration and window count
and checks full-stroke peak tangential, peak radial, mean radial and removed
volume within 1% when two completed runs are available. No such comparison was
performed here. `finish-force-validation.py` gates the completed optimized run on
its short-cut convergence fixtures and regenerates installed-rating/wall reports.

`force-band-study.py` evaluates entry, two middle table turns and exit from the
completed full history, with duration-weighted means. It reports agreement with
the full stroke, but does not enable skipping omitted sweeps from virgin stock.
A shortened force study needs an equivalent stock checkpoint, and changed error
sources generally cannot reuse a baseline checkpoint. Grading always retains
its full required geometry and fine tolerance.

Measured optimized full-stroke wall time is **6:00.50**, at 4 threads / 0.002 mm
sweep tolerance (path 0.000125 mm), peak RSS about **182 MiB**. Peak Fc=46.679 N,
peak Fr=16.990 N, whole-stroke mean Fr=3.0357 N and engaged-time mean Fr=4.9563 N.
Use engaged-time mean Fr for mean thickness/X compensation; whole-stroke mean Fr
remains the common quantity for energy checks and comparison with the archived
fine run. Per-pass maximum and RMS radial steps are 5.8012 N and 0.7285 N.
The original fine full-stroke run was intentionally stopped, not failed; its
partial log is `build/flexspline-reference-force/force-partial-stopped.log`. Wall
allocations pass dependent on assumed support/contact and short-cut force
convergence. The combined HB11 source budget remains open.

The proposed band statistics fail the 1% gate: weighted mean radial error
20.375%, peak radial error 3.723%. The complete sweep remains enabled.
