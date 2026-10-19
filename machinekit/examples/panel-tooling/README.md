# Reusable panel joinery and fit coupon

This generic fixture demonstrates keyed tab-and-slot alignment, a rebated corner,
and two locating pins with a separate M6 through bolt, washers and nut. It uses
the shared tooling libraries and has no mill or casting layout dependency.

Run from the repository root:

```sh
./haxeon/scripts/haxeon run --project=machinekit/examples/panel-tooling/haxeon.json
```

The default sheet thickness is an example 18 mm. To regenerate with a measured
thickness, pass it in millimetres:

```sh
./haxeon/scripts/haxeon run --project=machinekit/examples/panel-tooling/haxeon.json -- --thickness=17.8
```

Stock dimensions, coating designation and thickness variation are example inputs,
not a supplier specification. CAD/BOM uses the existing birch plywood material;
pin and fastener geometry uses steel. Material, fit and load qualification remain
pending. The 6 mm router diameter is stored per routing operation and must match
the cutter used to make its reliefs. Drill operations declare their own diameter.

Outputs under `build/review/`:

- `panel-joinery.mtrg`: assembled three-joint fixture with the pin fixture hardware.
- `panel-joinery-exploded.mtrg`: separated panels after removing loose hardware.
- `panel-joinery.assembly.json`: rebuildable assembly with registered component
  recipes and mating connectors.
- Per-panel `.panel.json`, `.step`, `.svg` and `.dxf`: the same exact finished
  operation contours drive CAD and both existing CadKit drawing writers. Layers
  identify outside/inside profiles, rebates, pockets and drills when present.
- `coupon-bed.*` and `coupon-wall.*`: a slot strip and shared plug. The four slots
  are marked with total clearances of 0.1, 0.2, 0.3 and 0.4 mm.
- `review.json`: stock inputs, geometric blocking edges, assembly/removal orders,
  contact volume tolerance and coupon clearances.

Inspect the assembled or exploded scene in Materia:

```sh
./app/run-built.sh --project="$PWD/machinekit/examples/panel-tooling/build/review/panel-joinery.mtrg" --perspective
```

For the keyed joint, the two unequal tabs locate the wall in the bed. External
clamps provide closure; the tab geometry does not supply clamping force. For the
rebate, seat the return panel on the retained shelf and clamp externally. Those
clamps are declared in the joint records but their envelopes are not modeled.
For the pin fixture, use the round hole as the primary locator, the elongated
secondary opening to release spacing error, and the bolt to close the plates.

The blocking graph covers the panel solids after loose closure hardware is
removed. Thread release and clamp rotation are outside the straight-translation
checker. Its exact sweeps preserve holes and arcs, choose travel that clears all
bounds, and reject blocking cycles rather than testing sampled poses. The report
states a volumetric contact tolerance of `1e-5 mm³`.

Cut the coupon using the intended stock, facing and tool. Try the same plug in
each marked opening, record the fit and actual thickness variation, then select
the clearance for the full kit. SVG/DXF show finished boundaries, not compensated
toolpaths. CamKit integration is scheduled for phase 5; this example emits no
G-code and consumes no sheet inventory.

Focused checks:

```sh
./haxeon/scripts/haxeon run --project=manufacturingkit/tests/tooling/haxeon.json
./haxeon/scripts/haxeon run --project=machinekit/tests/tooling/haxeon.json
./haxeon/scripts/haxeon run --project=cadkit/haxe/tests/haxeon-drawing.json
```
