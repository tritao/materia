---
title: Writing a part
description: How a machine part records its geometry as a CadKit document, so it has a history, named dimensions and stable face names.
status: draft
---

# Writing a part

A part a machine designs itself (a plate, a bracket, a casting, a sheet-metal
cover) is written as a **recording**: code that builds a CadKit document step
by step. The document is the part's geometry. It has a feature history the
editor shows, named dimensions, datums other parts mate to, and face names
that survive edits. Code stays the source; the document is what it produces.

## The shape of a recorded part

A `MachineComponent` records by overriding one method:

```haxe
override public function record(b:DocumentBuilder, detail:ComponentDetail):Null<Feature> {
  var thickness = b.length("thickness", values().number("thickness"));
  var plate = b.label(b.block(160, 90, thickness), "Stock · Plate");
  b.datum("seat", new Plane(new Vector(0, 0, thickness.value), Vector.X(), Vector.Z()));
  var window = b.boxBetween(-40, 40, -20, 20, -1, thickness.value + 1);
  return b.label(b.cutAll(plate, [window], ["window"]), "Window");
}
```

- **Its own frame.** Record the part where it is natural to draw it (a plate
  on z = 0, centred), not where it sits in the machine; the assembly places
  it.
- **Named dimensions.** Every numeric recipe input is an editable named
  parameter of the document, and the other way round (`PartRecordings`
  checks both). Values the part derives from other things are recorded as
  derived (`derivedBy("…why")`) or fixed (`fixedBy("ground flat stock")`).
  A part whose shape comes from a computed plan can declare its inputs with
  `PartRecordings.recipeInputs`.
- **Datums.** `b.datum(name, plane)` records the frames other parts attach
  to; `recordedDatum(name)` reads them back, and `BoltedFace.at(datum)` /
  `PinnedFace.at(datum)` put fastening faces on them.
- **Labels.** Label the steps people will look for ("Stock · Ground bar",
  "Front window"); the history shows them.
- **Holes come from the joints.** Do not cut fastener holes yourself. The
  joints that fasten the part derive its holes (`HoleSpec`), and recording
  adds them as one last "Fasteners · Derived holes" step, in the blank for
  sheet metal.

## Feature vocabulary

`DocumentBuilder` records CadKit's feature types; each is one class and one
registered codec, so the document saves and reloads them all the same way.

| Need | Record |
|---|---|
| A box by size, aligned to the frame | `block(x, y, z, align)`, or `box(x, y, z)` from the origin |
| A box between two corners | `boxBetween(x0, x1, y0, y1, z0, z1)` |
| A cylinder on any axis | `cylinderAlong(origin, axis, r, from, to)`, or `cylinder(r, h)` up +Z |
| A profile | `polygon(points, plane)`, `polygonSketch`, constrained sketches |
| Moving a solid | `place(...)` / `placeIn(frame)`; `translate`; `rotate` by an editable angle |
| Several cuts or unions in one step | `cutAll(target, tools, labels)`, `unionAll(parts)` |
| Holes | `holes(body, specs)`; `sheetHoles` for sheet metal |
| Sheet metal | `sheetBase`, `sheetFlange` (edges by name), `sheetCut`, `sheetHem`, ... |
| Lettering (terminal numbers, legends) | `strokeText(text, height, stroke, z0, z1, mirror, x, y, rotation)` |
| Cosmetic details on a face | `thread(source, "cyl.side", spec, start)`; `surfaceDetail(source, face, Knurl / Graduation / Display)` |

Packages add their own types through `FeatureTypeRegistry` (MachineKit's
castings record one step per casting feature this way).

## Face names

Drawings, CAM setups and mates refer to faces by name, so a recorded part
names every face strongly and once. MachineKit's naming audit checks every
recipe's default part. Faces are named from the features that
made them (`f3:box.+z`, `J1.h1.tap`); tools named by what they are
(`Solids.named`, a prism's id) keep their faces apart. The hobber's
equivalence gate fails on any weak or repeated name of a recorded part.

A face detail names its face as the feature that made it names it: a
cylinder's side is `cyl.side`, a box's top `box.+z`, and the face a revolved
or extruded profile's segment k sweeps `side(f<profile>:seg.k)`
(`HmiPart.sweptFace`). Details follow the face through later steps, so record
them on the feature that made the face, before it is cut or fused; details
can be chained.

Avoid geometry that splits a face into disconnected pieces (a bore tangent to
the sides of a pocket, a hole stopping just past a bore wall): each piece then
shares one name. Draw what the part is instead (a set-screw hole breaks into
the bore; a moulded comb is one section, not boxes side by side).

## Bought devices

A bought device of several bodies (`HmiPart.bodies`) records each body with a
`DeviceBodySpec`; the device's own geometry records them fused, and each
expanded body records its own. Displays, knurling and threads are recorded
face details.

## The immediate API is frozen

`Part.box`, `Solids.*` and `PosedParts.moved` build solids directly, with no
history and no stable names. CadKit's evaluators use them; new parts do not.
`tools/check-immediate-geometry.py` counts the remaining call sites per file
against a checked-in baseline: a file may lose them, never gain them.
