# Numerical gear inspection

`GearMeasurement.fromRings(source, spec, inspection)` inspects undeformed spur gears
against the specified datum axis. `RingStock` implements the pure-Haxe `GearRings`
read interface. GearKit has no StockKit, CAD or machine dependency.

```haxe
var inspection = new GearInspection(spec, profileControlRadius);
var report = GearMeasurement.fromRings(stock, spec, inspection);
var profileMicrometres = report.value("Falpha");
```

Supply the profile control radius from the active flank design. The optional default
in `fromRings` uses a geometric root window and adds a qualification warning: a mating
gear's active contact range cannot be inferred from the loose gear alone. The default
measurement radius is explicitly the pitch radius; it is a chosen inspection location,
not an implementation of the standard's unspecified-diameter fallback. Profile and
pitch Z default to midface. Both flanks of every tooth are inspected there. Helix traces
cover every tooth at the measurement radius. This is a numerical inspection of sampled
geometry, not calibration or physical measurement uncertainty certification.

## Results and units

- `values`: worst magnitude across both flanks/all inspected teeth, in micrometres:
  `fp`, `Fp`, `Fr`, `Falpha`, `ffalpha`, `fHalpha`, `Fbeta`, `ffbeta`, `fHbeta`.
- `pitch` and `cumulativePitch`: two signed per-tooth arrays, low-angle/high-angle
  flank order, anticlockwise traversal, in micrometres. Index trace datum is tooth zero.
- `profiles` and `helices`: tooth/flank-labelled traces with mm abscissae, raw and
  filtered material-positive micrometre ordinates, least-squares line, total/form
  envelopes and signed slope. Profile slope is signed toward the tip. Helix trace slope
  is signed by material-positive flank displacement; its sign depends on the flank,
  and is not the ISO right/left-hand helix-angle sign. Aggregate slopes are magnitudes.
- `thickness`: circular tooth thickness at the specified measurement radius, mm.
  ISO flank
  classification does not assign a class to tooth size.
- `ballDiameter`, `ballCentres`, `dimensionsOverBalls`: the specified-size virtual
  balls reconstructed from pitch probings, and dimensions across opposing pairs
  (nearest opposing pair for odd tooth counts). External dimensions include both
  outer ball surfaces; internal dimensions use the inward surfaces. These are
  over/under-balls equivalents through reconstructed involutes, not physical contacts
  on unmeasured surface irregularities.
- `radialMeasurements`: reconstructed virtual-ball centre radii for each tooth space,
  mm. These use the nominal-base involutes through pitch probings and their normal
  offsets, following the Annex E.5 construction. They approximate two-flank probe
  contact; irregular surfaces can differ from physical ball measurements.
- `grades`: numerical candidate per parameter (null means worse than class 11).
  `overallGrade` is the worst candidate, null if any parameter is missing or fails
  class 11. It can be provisional or extrapolated: always display `warnings` and
  `extrapolation` alongside it. Missing coverage is never reported as zero.
- `isoClass`: populated only for complete, filtered, sufficiently sampled numerical
  inspections with explicit settings and dimensions in the standard's scope.
  It does not certify a manufactured part. Any qualification warning clears it.

Small-angle angular residuals are resolved at analytic ring endpoints. Profile and
helix deviations use the transverse base tangent (`rb * angular residual`); pitch
uses the measurement circle (`rM * angular residual`). Residuals are interpolated in
roll distance and axial position, keeping an ideal involute exact under interpolation.
The gear is never fitted/recentred to remove eccentricity. Missing, duplicate or
ambiguous flanks throw instead of creating a plausible report.

## Evaluation and filtering

`GearInspection` accepts an explicit control radius, optional tip form radius and
measurement locations. Profile evaluation ends at 95% of the roll distance from
control to tip form. Profile slope extrapolates to the actual tip diameter, helix slope
to the whole face width. The helix fit excludes `min(0.05*b, module)` at each end.
Excess material outside fit windows contributes to total/form envelopes; minus
material outside them can be ignored. Only the unrounded involute is supported.

The digital Gaussian kernel has 50% sinusoid transmission at its cutoff, 4.5 sigma
support, and no phase shift. Cutoffs default to the standard recommendations.
Endpoints extend the whole-trace least-squares line with zero residual. This explicit
numerical boundary convention can affect end results; raw traces remain available.
A zero cutoff requests unfiltered diagnostic evaluation and clears `isoClass`.
Source density (before interpolation) is checked for at least 150 profile points,
helix count `5*b/cutoff`, and spacing that resolves each cutoff with five samples.
Interpolation cannot promote sparse source data into a qualified class. Ring grids
usually omit exact face endpoints; such reports warn that end excess is unchecked.
A grid padded half a cell beyond the face can sample both face endpoints directly.

## Tolerances and references

Implemented against **ISO 1328-1:2013**, clauses 3.3–3.5, 4.4.4, 4.4.6–4.4.8,
5.2–5.3, Annex E. The 2013 formulas, rather than 1995 diameter-bin tables, are used.
`Iso1328.tolerance(spec, parameter, class)` returns rounded micrometres;
`rounded=false` exposes intermediate values for checks. Total profile/helix limits
combine **unrounded** slope and form limits before rounding. All classes use the
unrounded class-five basis and the square-root-of-two step. Runout uses Annex E's
0.9 cumulative-pitch basis. Unknown parameters and classes outside 1..11 throw.

Sources inspected:

- [ISO's official terms and scope](https://www.iso.org/obp/ui/?_escaped_fragment_=iso:std:iso:1328:-1:ed-2:v1:en).
- [ISO-authored 2013 document, mirrored PDF](https://normfile.com/iso/ISO%201328-1-2013%20PDF.pdf),
  printed pages 20–27 and 40 (mirror ends before E.4/E.5).
- [ISO-authored full text, indexed mirror](https://files.infocentre.io/files/docs_clients/126_2007770361_2900729_ISO%201328-1_2013.pdf),
  Annex E.4/E.5, printed page 41 (direct download may return HTTP 403).

The standard disallows extending its classes outside its stated dimensional ranges.
The project's module 0.3 reference therefore reports **experimental extrapolated
formula grades**, with `isoClass=null`. It cannot claim ISO class 3. Out-of-scope
agreements and fine-pitch standards need separate specification. No standard PDF is
vendored. Decimal fixtures at module 0.5, diameter 50, face width 10 independently
check the formulas; the standard does not supply a table of those fixture numbers.

## Checks

`gearkit/tests` covers ideal external/internal gears, an actual translated involute
fixture (`Fr` approximately twice eccentricity), pitch first harmonics, profile/helix
slopes, Gaussian transmission, end excess/relief, incomplete/sparse sampling and scope.
`stockkit/tests/metrology` checks the same API with native ring intervals. Its optional
`reference` mode performs an actual full-profile, full-face rake sweep of the HB3
reference gear. Run this heavy acceptance through the workspace heavy-run wrapper.
