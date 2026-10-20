# SR10 planar DXF import

Run `machinekit/examples/sheet-router/dxf/check.sh` from the repository root. It
uses the shared build gate, pure geometry checks and the existing native runtime;
there is no OCCT rebuild. On 2026-10-21, **42 import assertions** and **653 existing
drawing assertions** pass. Tested source hashes are in `validation.json`.

`Dxf.read(text, options)` returns a `DxfDocument` with exact directed line/arc
segments, original layer names, labels, diagnostics and unit assumptions. Supported
cutting entities are LINE, ARC, CIRCLE, LWPOLYLINE (including positive/negative
bulges), ordinary 2D POLYLINE/VERTEX, control-net SPLINE and recursive INSERT.
Blocks honor base points, inherited layer 0, rotation, reflection, scale and array
rows/columns. Recursive/missing blocks and singular scales are refused.

Coordinates and tolerances in the result are **millimetres**. `$INSUNITS` supplies
common engineering units; callers can override `millimetresPerUnit`. Unitless R12,
including our writer, assumes mm and explicitly reports `assumedUnits=true`.
LINE and spline control points are WCS; planar arc/polyline OCS accepts ±Z. Tilted
extrusion, nonzero Z geometry, fitted/polyface/3D polylines and unsupported cutting
entities fail rather than silently projecting or dropping cutting geometry. Setting
`allowUnsupported=true` is a diagnostic import, with warnings the caller must
resolve before CAM. TEXT from our writer round-trips; block attributes are reported
as ignored annotations. `DxfReader.drawing` converts compatible layers/text back to
Drawing2D and refuses rotated text that Drawing2D cannot represent. General DXF
text styling, filled polyline widths, hatches and native ELLIPSE entities are outside
this planar centreline reader.

Circular arcs remain exact, including under similarity/reflection transforms. A
nonuniformly scaled arc becomes ellipse chords with interpolation error bounded by
`radius × ||transform||F × parameterStep² / 8`. Rational B-splines require explicit
control points, nondecreasing knots and positive weights. Knot insertion splits
all active spans into rational Beziers, then homogeneous de Casteljau subdivision
continues until every projected control point lies within `tolerance` of its chord.
Positive weights keep the curve inside that control hull. The default tolerance is
**0.01 mm** in final output coordinates; transform norm is accounted for before
spline subdivision. Fit-point-only splines and degenerate/unrepresentable curves
are refused explicitly. Tests include unclamped ends, nonuniform knots, rational
quarter circles and independent de Boor evaluation for quadratic/cubic curves.

`DxfContours.assemble(document.curves, gapMm, layerOperations)` joins unordered and
reversed edges within each source layer. Endpoint gaps accepted by the default
0.05 mm tolerance become explicit short line bridges, preserving authored arcs.
Larger gaps report their layer, endpoint, nearest endpoint and distance; branches
are diagnosed rather than arbitrarily selecting an edge. Callers must resolve
all `gaps` before generating a complete cutting program. Self-intersections and
crossing/touching loops of one operation are refused by exact line/arc checks.

The optional layer map retains both original layer and operation identity. Loops
mapped to the same operation are nested by geometric containment: even depths are
parts and odd depths are their holes, including islands inside holes. `DxfPart`
provides an exact validated `WallOutline`. Profiles on separate operation identities
are kept separate, so callers map outside/hole layers to their intended profile
operation before extracting material parts. No arc tessellation is used for this
containment or area calculation.

Primary format references: Autodesk's [SPLINE groups](https://help.autodesk.com/cloudhelp/2018/ENU/AutoCAD-DXF/files/GUID-E1F884F8-AA90-4864-A215-3182D47A9C74.htm),
[LWPOLYLINE groups](https://help.autodesk.com/cloudhelp/2024/ENU/AutoCAD-DXF/files/GUID-748FC305-F3F2-4F74-825A-61F04D757A50.htm),
[VERTEX bulge definition](https://help.autodesk.com/cloudhelp/2024/ENU/AutoCAD-DXF/files/GUID-0741E831-599E-4CBF-91E1-8ADBCFD6556D.htm), and
[INSERT groups](https://help.autodesk.com/cloudhelp/2017/ENU/AutoCAD-DXF/files/GUID-28FA4CFB-9D5E-4880-9F11-36C97578252F.htm).

Retained packet: `build/sheet-router-evidence/sr10-dxf.tar.gz`, SHA-256
`0fb5d9a3f6e23b606f641c8120fa29d4135ec94b49efe2e98f37cc60ba905ca8`; regular-file member hashes verified.
