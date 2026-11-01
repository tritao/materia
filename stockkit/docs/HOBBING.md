# Ideal hobbing sweeps


`stockkit.hobbing` cuts the actual HB1 rake-face polygons through the HB2 circles. It has no
machine, rigid joint coupling, target-profile cutter shortcut, or injected scallop-depth formula.
`HobbingAxes` supplies X/Y/Z/A/B/C as a function of time: mm, unwrapped radians and seconds.
X is centre distance; Y is shift along the hob axis; Z is head height. The frame chain tilts the
hob from -Y by A, rotates it by B, then applies inverse C into the work frame.

```haxe
var spec = new gearkit.GearSpec(0.3, 200, 8, new gearkit.Involute(20 * Math.PI / 180));
var hob = new gearkit.HobDesign(gearkit.ConjugateRack.of(spec.profile()), 1, 32, 12);
// Fast transverse slice with three measuring circles around the 30 mm pitch radius.
var stock = stockkit.ring.RingStock.annulus(new stockkit.ring.RingGrid(29.925, 30.075, 3.999999, 4.000001, 0.05, 1));
var job = new stockkit.hobbing.IdealHobbingJob(spec, hob, 3000, 1.1, 0, 4);
var sweep = new stockkit.hobbing.HobbingSweep(hob, -8, 8); // inclusive axial tooth indices
var report = sweep.cut(stock, job.axesAt, job.schedule(stock.grid));
sweep.dispose();
stock.dispose();
```

The ideal driver supports external spur gears and right-hand hobs. It sets the lead-angle tilt,
table ratio, static shift compensation and optional feed in mm per work revolution. General
motion is supplied independently. `HobbingMotion` interpolates controller samples and
`GashPass.fromMotion` locates measured-B phase crossings between ticks, preserving independent
actual C. Pass `motion.times()` as `motionKnots` to split sweeps at every controller knot.

Native rake tools triangulate simple planar regions once and intersect swept triangle-endpoint
convex hulls with each measuring circle. `HobbingSweep.cut` samples the actual rigid path,
includes exact event-centre times, and refines the whole job until its observed per-circle angular
symmetric difference, expressed as arc length, meets the requested tolerance (default 0.00005 mm).
This is numerical convergence on the sampled circles. It does not establish a machine accuracy
class. Nonlinear motion must be resolved by the caller's time knots and sampling interval.
A failed job restores the incoming stock; each native pass is atomic. `cutPass` exposes one
sampling resolution for studies. Gash/pass endpoint provenance is preserved.

`HobbingSweep.cutPassMany` can share native geometry processing between pairs of
independent inspection domains. Set `STOCKKIT_RAKE_BATCH_DOMAINS=1` before creating
the sweep to opt in. It uses `sk_ring_stock_sweep_rake_pair`; an odd final domain
uses the ordinary sweep. Both domains keep independent circle intersections,
interval unions and provenance. Invalid arguments leave both stocks unchanged,
and an exception during application restores both domains. Transforms and prism
bounds are streamed once per pair, with no geometry cache across passes. Sweep
sampling, tolerances and thread counts remain explicit. The ordinary single-stock
force path is unchanged.

Focused checks live in `tests/hobbing/haxeon.json` and also run as a main StockKit test group.
They cover generating facets and their inverse-square gash-count trend, multiple starts, pitch
thickness, axial-feed marks, controller interpolation and rollback. Their acceptance runs use
transverse slices; full-face throughput, application playback and ISO metrology are later steps.


## Gear metrology

`RingStock` implements GearKit's read-only `GearRings` interface. Use
`gearkit.GearMeasurement.fromRings(stock, spec, inspection)` for tooth/flank traces,
virtual-ball runout, circular thickness and per-parameter numerical grading.
See [evaluation settings, units and qualification rules](../../gearkit/METROLOGY.md).
The native adapter test is `stockkit/tests/metrology/haxeon.json`; its `reference`
argument runs the heavy full-face hobbing acceptance case.

The HB4 reference's aggregate convergence failure has a small boundary diagnostic:

```sh
machinekit/examples/flexspline-hobber/check-hb4-boundary.sh
```

This reuses the validated native provider (`HB4_PROBE_NATIVE` overrides its path),
compiles only the metrology entrypoint and runs 103 original gash windows at the
exact offending radius/Z. The first two endpoint/provenance pairs must exactly
match saved full-stroke snapshots before studying refinements 2–5. The script
records native/binary/fixture hashes, four threads, path tolerances and timings.
Observed time is about 88 seconds. It is a diagnostic, not full-circle convergence
or reference grading. `HB4_BOUNDARY_INPUT` selects the companion
`stockkit/tests/metrology/fixtures/hb4-profile-boundary.json` (111 windows); use a
separate `HB4_BOUNDARY_EVIDENCE` directory for its results. The `convergence-probe`
entrypoint retains the slower four-circle/full-stroke reproduction for rebuilding
these anchors. Reference acceptance still requires all inspection circles and the
physical face ends to satisfy the unchanged 0.00005 mm total-arc criterion.


Related records: [cutting loads](HOBBING_FORCES.md) (HB10) and [error budget](HOBBING_ERRORS.md) (HB11).
