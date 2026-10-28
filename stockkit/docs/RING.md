# Ring stock


`RingStock` represents a gear tooth band as material arcs on cell-centred `(radius, z)` circles.
Angles are radians about +Z from +X. Endpoint positions are double precision, with no angular
raster; each endpoint carries the gash and pass that exposed it. The cells exactly cover the
specified annulus and height. Native ring structs and handles are separate from Cartesian stock;
the generated StockKit C ABI is version 5.

```haxe
var spec = new gearkit.GearSpec(0.3, 200, 8, 20 * Math.PI / 180);
var stock = stockkit.ring.RingStock.fromGearBlank(spec, 0.6); // mm; wall thickness BELOW the roots
var saved = stock.snapshot();
stock.cutRegion([new stockkit.ring.RingCut(130, 0, stock.grid.axialCount, 0.1, 0.02, 3, 1)]);
var arcs = stock.circle(130, 0); // sorted material intervals in [0, 2*pi]
var comparison = stock.compare(spec); // full tooth, root/tip fillets included
var mesh = stock.contour(); // StockMesh streams in mm; triangle source is a pass ID
stock.restore(saved);
saved.dispose();
stock.dispose();
```

`annulus(new RingGrid(...))` creates stock with explicit bounds and maximum radial/axial cell
sizes. `cutSlab` intersects a planar slab analytically. `cutRegion` applies a validated batch in
array order; negative starts and spans crossing the angular seam work, and a full revolution
removes a whole circle. `setThreads` changes execution parallelism without changing endpoints or
sources. Snapshots share immutable radial layers and remain valid after the original stock is
disposed. Restore and `compareStock` require identical grids.

`compare(GearSpec)` reports angular leftover/gouge per circle; `RingComparison` converts those
to arc lengths in mm and cell-volume estimates in cubic mm. This is a geometric comparison,
not ISO grading. `contour` displays the piecewise radial/axial cell boundary, suppressing internal
faces; only its angular chords approximate the arcs. Metrology reads `circle` directly.
`stockkit.hobbing.HobbingSweep` supplies the hob-edge sweep described in [HOBBING.md](HOBBING.md).

Focused ring tests are in `../tests/ring/haxeon.json` and are also a group of the main StockKit
suite. The CMake `stockkit_ring_tests` target exercises the C ABI and the reference-part memory
and snapshot budgets. The reference case includes a continuous 0.6 mm wall below the roots,
5 micrometre radial cells, 0.1 mm axial cells and 200 material intervals per tooth-band circle.
