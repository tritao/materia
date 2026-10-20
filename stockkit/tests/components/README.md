# Completed-cut component scan

`StockComponents` is a Z-dexel connectivity helper for post-cut planning. It
returns deterministic component order, material volume, bounds and support area
by caller-defined region. Adjacent XY rays connect through positive interval
overlap; vertical shared faces connect; diagonal-only contact does not.

Tests cover thin skins, through kerfs, disconnected layers, a layer bridge,
region partitioning, invalid intervals and an actual native 50 mm contour cut
with snapshot restoration. The native fixture uses a 1 mm Z lattice and checks
volume conservation. Run the router vacuum check script for compiler/native
provider setup; `--synthetic-only` skips the native cut fixture.

This is a checkpoint API: do not mutate stock concurrently. Two rows are streamed,
but union nodes can grow with region fragmentation. Sheet-scale storage budgets
belong to the subsequent StockKit tiling work.
