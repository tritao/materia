# Sketch edit benchmark

Run `./scripts/benchmark-sketch-edit` from the repository root.
The probe warms up five edits and reports median, p95, and maximum timings for
25 edits of the constrained slotted bracket. It separates isolated draft
solving, sketch profile construction, full document recompute, and viewport
tessellation at the editor's current and coarse preview deflections. It also
reports how quickly a cancellation request stops a solve.

The benchmark has no pass/fail timing thresholds. Results depend on the host,
build configuration, and native OCCT build.
