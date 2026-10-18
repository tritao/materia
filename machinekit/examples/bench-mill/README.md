# Bench mill and mineral-cast frame study

The working `BenchMill` has 250 mm X, 150 mm Y and 250 mm Z travel.
`EnclosedBenchMill` adds its stand, tray, guarding and tending interfaces.
`MillBasePreview` isolates that existing stand; it is not the machine frame.

## New structural concept

`MillFramePreview` derives the guide datums from the real `BenchMill` assembly
and builds a separate `machinekit.milling.MillFrame`. The existing machine and
its motion/controller model remain available unchanged.

The mill-specific layout uses two shared `MineralFoundation` assemblies:

- A lower casting with full-height longitudinal beams under the Y seats, a
  transverse rib, six underside-open pockets and a solid rear column support.
- A separate column casting with two rear-open chambers, a thick front web and
  a cross rib.
- Separate steel Y/Z guide inserts, four pairs of column mounting pads, and
  four underside foot interfaces. Each insert has a matching casting recess.

`MillFrameLayout` takes typed guide interfaces and structural options. It owns
mill layout constraints; shared casting, insert, mass and persistence code stays
in `machinekit.structural`. A future lathe bed should supply its own layout on
that same structural layer, rather than add machine-type switches here.

The initial concept uses a 520 mm wide, 220 mm high base and a 320 × 260 mm
column cross-section. Base depth follows the Y guides and column support,
not the enclosure footprint. Steel seats finish 4 mm above/in front of their
castings, with 16 mm embedded thickness. These are concept dimensions; they
are not validated process minima or calculated stiffness requirements.

Generate the inspection artifacts from the repository root:

```sh
./haxeon/scripts/haxeon run --project=machinekit/examples/bench-mill/haxeon.frame.json
./app/run-built.sh --project="$PWD/machinekit/examples/bench-mill/build/mill-frame-concept.mtrg" --perspective
```

Outputs in `build/`:

- `mill-frame.mtrg`: the castings and steel interfaces alone.
- `mill-frame-concept.mtrg`: frame plus reference rails, carriages, table, head,
  spindle, screws, support units and motors in their original initial arrangement.
- `mill-frame-concept.json`: travel, principal dimensions, casting masses and
  remaining design work.

The reference parts are a **static arrangement study**, not an operable machine.
The guides and moving bodies share a single vertical datum shift; their internal
axis geometry is not resized. The Z screw, support units and motor move sideways
by an additional explicit offset so the nut clears the wider column; the offset
is derived from the actual nut flange diameter and recorded in the report. The old structural bodies, electrical equipment and structure-specific
mounting brackets are omitted. Screw/motor support adapters, their steel insert
patterns and fasteners still need design, including the longer Z nut-to-head
adapter required by that sideways offset.
Do not infer full-travel clearance or load capacity from the initial picture.

Run focused checks:

```sh
./haxeon/scripts/haxeon run --project=machinekit/examples/bench-mill/haxeon.frame-tests.json
./haxeon/scripts/haxeon run --project=machinekit/tests/casting/haxeon.json
```

Checks cover connected CAD solids, non-overlapping steel/casting volumes,
solved guide planes and column-pad contact, assembly recipe roundtrips, reference
part datums and rejected layouts. They do not replace a full-travel collision
sweep, static/modal analysis, or mold/process design.

Next: resolve drive supports and fasteners, audit the full motion envelope,
compare column alternatives, then design chip drainage, lifting points, molds
and the separate enclosure. Material properties remain the shared assumed
mineral-cast values; this study does not claim the reference images' UHPC grade.
