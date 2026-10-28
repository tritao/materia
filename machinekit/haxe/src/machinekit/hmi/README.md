# Operator controls (`machinekit.hmi`)

Panel-mount parts for operator panels and the tools to lay them out: used by the jog pendant
(`machinekit.pendant`), and by any door or machine panel.

## Parts

Each bought part is a MachineKit component with real geometry, mass, a BOM line and provenance, in its
own frame (origin at the centre of the panel's front face, +z toward the operator). `PanelPart` is what a
part tells the layout about itself (cutout, keep-out, stud holes, height above the face, depth behind it,
rear body, mass, the thickest panel it fits), so the numbers live only in the recipe.

| Recipe | Detail | Numbers from |
|---|---|---|
| `machinekit.hmi.hand-wheel` | Revolved flange, flared body, knurl ring and dial recess; spinner handle; boss through the opening; studs on a circle; terminal strip | YUMO ISM6045 drawing (defaults); the Vallder wheel is the same recipe with other values |
| `machinekit.hmi.emergency-stop` | Mushroom head on a collar, stem, ring nut, base and one to three contact blocks | A seller's XB2-ES542 family drawing; a compact EAO or APEM type is the same recipe with a thin base |
| `machinekit.hmi.rotary-switch` | Knob, bush, nut, round body; `positions` (12 axis, 3 step) | Assumed |
| `machinekit.hmi.push-button` | Bezel and domed cap, barrel, nut, body; colour and LED | Assumed |
| `machinekit.hmi.oled-module` | Glass, carrier PCB, standoffs, header behind a window | Assumed |

They are registered in `MachineKitComponents` and filed under "Operator controls". Every part has a `basis`
input (catalog, stated, assumed, measured) that appears in its designation, so an assumed number is visible
in the BOM. `PartPresets` names the real parts we buy (`yumoIsm6045`, `vallderMpg`, `xb2Es542`,
`compactEStop`, ...); a measured part replaces one preset.

## Layout

`PanelLayout` is a panel (size, plate and wall thickness, case depth) with `PlacedPart`s on it, and
`problems()` checks it: every part inside the usable area, no two parts closer than the minimum gap, no two
rear bodies overlapping, bodies inside the case wall, the case deep enough, the plate thin enough for every
part's fixing. `PanelFacePlate` builds the plate as a solid with every cutout (printable as a mock-up),
`PanelAssembly` places the part solids and checks 3D interference against each other, the plate and an
optional shell, and `PanelLayoutSvg` draws it for review.

## Tests

`HmiPartsTests` (each solid against its own numbers, every option and flag, rebuild identity),
`PanelLayoutTests` (each check catches what it should, plate volume, interference), and `PendantTests` for
the pendant's reference layout and case.
