# Operator controls (`machinekit.hmi`)

Panel-mount parts for operator panels and the tools to lay them out: used by the jog pendant
(`machinekit.pendant`), and by any door or machine panel.

## Parts

Each bought part is a MachineKit component with real geometry, mass, a BOM line and provenance, in its
own frame (origin at the centre of the panel's front face, +z toward the operator). A part states how it
mounts with a **`PanelMountFacet`** (`machinekit.component`): the cutout, keep-out, stud holes, height
above the face, depth behind it, rear body and the thickest panel it fits. The facet is added in the part's
constructor, so a part that forgets a number cannot be built, and any component with the facet (not only
these) can go on a panel.

| Recipe | Detail | Numbers from |
|---|---|---|
| `machinekit.hmi.hand-wheel` | Revolved flange, flared body, knurl ring and dial recess; spinner handle; boss through the opening; studs on a circle; terminal strip | Catalog rows YUMO ISM6045 (maker drawing) and Vallder (its drawing) |
| `machinekit.hmi.emergency-stop` | Mushroom head on a collar, stem, ring nut, base and one to three contact blocks; one NC contact port per block | Catalog rows XB2-ES542 (a seller's family drawing) and a compact EAO or APEM type |
| `machinekit.hmi.rotary-switch` | Knob, bush, nut, round body; `positions` (12 axis, 3 step) | Assumed |
| `machinekit.hmi.push-button` | Bezel and domed cap, barrel, nut, body; colour and LED | Assumed |
| `machinekit.hmi.oled-module` | Glass, carrier PCB, standoffs, header behind a window | Assumed |

They are registered in `MachineKitComponents` and filed under "Operator controls". The wheel and E-stop take
a `model` input that selects a row of `ControlCatalogs` (`machinekit.catalog.Catalog` with
`CatalogMetadata`: source, conformance, verified fields); `CUSTOM` takes caller values. Every part has a
`basis` input (catalog, stated, assumed, measured) that appears in its designation, so an assumed number is
visible in the BOM. `PartPresets` names the parts we buy; a measured part replaces one catalog row.

## Layout

`PanelLayout` is a panel (size, plate and wall thickness, case depth) with `PlacedPart`s on it, and
`problems()` checks it: every part inside the usable area, no two parts closer than the minimum gap, no two
rear bodies overlapping, bodies inside the case wall, the case deep enough, the plate thin enough for every
part's fixing. `PanelFacePlate` builds the plate as a solid with every cutout (printable as a mock-up).
`PanelAssemblyBuilder` expresses a panel as a `MachineAssembly` (plate, optional shell, every part in place)
so `AssemblyIntegrity.check` reports overlaps. `PanelLayoutSvg` draws a layout for review. The shapes
(`Footprint`, `FootprintGeometry`) and the rounded plate outline (`RoundedRectangle`) are in `cadkit.modeling`.

## Tests

`HmiPartsTests` (each solid against its own numbers, every option, flag and catalog row, rebuild identity),
`PanelLayoutTests` (each check catches what it should, plate volume, integrity) and `PendantTests` for the
pendant's reference layout and case.

## Decisions

- The layout stays here, not in `machinekit.enclosure`: that package builds machine enclosure walls, and a
  single-opening `EnclosureCutPanel` does not share code with a many-part layout checker.
- `PanelLayoutSvg` stays a small review tool: `cadkit.drawing` is for manufacturing drawings with bend and
  cut layers, which a layout review is not.
