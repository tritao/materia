# Small / Compact VMC workholding

Both configurations default to `machinekit.milling.ZppWorkholding`: a **ZPPCNC
AR155-I single-station pneumatic vise**, mounted on a **manual TS96-155 zero-point
plate**. Changing the workpiece uses the vise; removing the entire vise uses the
separate zero-point plate. The plate remains manually locked.

## Pneumatic setup

The original supplier STEP is retained for the base, both steel carriers,
aluminium jaw blocks, serrated hard jaws, TS96 plate and TS96 pull studs.
A 200 × 190 × 20 mm fabricated steel adapter connects the plate to the table.
Its M10 taps use a 128 mm square, matching the plate's actual corner mounting
holes. M10 × 30 screws seat 16 mm above the plate bottom. Four M6 × 35 table
screws at X ±85, Y ±80 mm engage slot nuts in the table's ±80 mm T-slots.

Supplier references:

- [AR155-I](https://zppcncvise.com/products/zpp-125mm-self-centering-vise-zero-point-workholding)
- [TS96-155](https://zppcncvise.com/products/zpp-square-zero-point-quick-change-plate-cnc-zero-point-system)
- [TS96 pull studs](https://zppcncvise.com/products/ts96-pullstuds-pull-studs-series-m10-d20-29mm)
- [Catalogs](https://zppcncvise.com/pages/service), 2026 printed page 24 for AR155-I.

The catalog specifies a 155 × 155 × 126 mm vise, 130 mm jaws, 9 kg assembly mass,
and 8–10 kN clamping force at 6–7 bar. These are supplier ratings, not measured
or simulated guarantees. Model materials use nominal aluminium and steel density;
the single imported base solid includes multiple physical materials.

The hard jaws in the supplied CAD have an approximately 59.5 mm tooth opening.
For our 60 mm blank, each carrier moves outward 0.25 mm. Aluminium blocks move
with their carriers, and the hard jaws move with the blocks. The block tops at
126 mm above the vise base support the blank, with about **2.5 mm serrated jaw
engagement**. This replaces the manual vise's separate parallels. Serration
indentation, stamping and preload are not modeled. The reviewed jaw preset accepts
59.5–61.5 mm blanks; the catalog's wider capacity requires another jaw setup and
is not represented by extending pneumatic travel.

The stock bottom is **176 mm above the table**: 20 mm adapter + 30 mm plate +
126 mm jaw blocks. Its top is 196 mm above the table, about 17.5 mm above the hard
jaws. G54 and fixture bounds derive from these actual assembly poses. Self-centering
locates only the closing direction; lateral positioning/probing remains necessary.

### Height and Z capacity audit

The supplier's nominal 126 mm vise height is the aluminium jaw-block support
level in the imported STEP. Serrated hard-jaw teeth extend to about **128.502 mm**;
the installed hardware therefore reaches **178.502 mm above the table**, while
the blank seat remains 176 mm. Pull studs enter the adjoining parts and do not
add another layer of stack height. The adapter's 20 mm thickness is our design.

With the currently fitted **70 mm gauge-to-tool-tip length**, assembly-derived
limits are:

| Dimension (mm) | Small VMC | Compact VMC |
| --- | ---: | ---: |
| Table surface above model origin | 448 | 574 |
| Rendered assembly height at default pose, including feet | 1321.4 | 1718.4 |
| Table above lowest modeled foot surface | 498 | 624 |
| Spindle gauge above table, lower–upper Z stop | 70–320 | 150–530 |
| Tool tip above table, lower–upper Z stop | 0–250 | 80–460 |
| Blank support above table | 176 | 176 |
| Current blank top above table | 196 | 196 |
| Tool-tip clearance over current blank at upper stop | 54 | 264 |
| Maximum blank height on this seat with 5 mm tip clearance | 69 | 279 |

Both variants reach the current blank and complete the bearing-block program.
The tall vise consumes much of the Small VMC's available height; the current
20 mm blank fits, but taller work requires checking the tool length and loading
clearance. These are axial reach limits, not permission to lower the tool into
the fixture or proof of holder/head clearance for arbitrary workpieces. The
complete mechanical Z range includes positions that would collide with this
setup. Overall machine height alone does not determine usable work height.
The rendered bounds run from Z = −50 to 1271.4 mm (Small) and −50 to 1668.4 mm
(Compact) in the default pose, excluding the unmounted machining-target preview.
These are current mesh bounds, not an enclosure/service envelope.
The model origin is not a surveyed floor datum, and these machine dimensions
describe our designed assemblies rather than a supplier-built machine.

TS96 studs locate on a 96 mm square. Their 5.08 mm collars enter the vise's 16 mm
counterbores, while their shoulders seat on the vise bottom. The 24.003 mm
projecting bodies enter the plate. Nominal M10 × 20 connector envelopes join the
studs to the vise. Thread geometry and fastening details remain assumptions;
penetration checks cover the solid stud interfaces, excluding nominal thread fits.

## Actuation scope

This is a **supplier pneumatic hardware model with geometric jaw motion**.
`airA` and `airB` are exposed pneumatic service ports at the two CAD inlet bores.
Their thread/fitting specification and open/close assignment are not verified.
No supply, valve, pressure law, cylinder bore or pressure-loss behavior is invented.
The implementation does not yet simulate air-driven actuation.

Opening provides an explicitly assumed **2 mm clearance per jaw** (4 mm total),
within the CAD carrier envelope. This is a packaging preview, not a manufacturer
stroke specification. Joint limits cover the current stock preset and this release
clearance. Position switches report `clamped-position` and `released-position`;
neither measures pressure or grip force. Machining checks half-opening against
30 mm within 0.05 mm and stops if the jaws leave that preset.

A valve-driven clamp cycle requires verified stroke, port/control and pressure
behavior before binding this supplier model to the pneumatic runtime. Clamping
force, retention and safety under cutting loads remain unqualified.

The [actuation evidence review](../../../tools/suppliers/zppcnc-actuation-review.md)
records the public-document search, the supplier's unconfirmed spring-close
claim, runtime implications, and a technical request ready for supplier review.

## Reproducible supplier assets

`tools/suppliers/zppcnc-reference.json` pins original archive, STEP and PDF hashes,
alongside the reviewed frame and mounting dimensions. Run:

```
python3 tools/suppliers/zppcnc.py --prepare
```

Add `--offline` to verify existing files. Downloads remain in ignored
`build/suppliers/zppcnc`; normal component generation never accesses the network.
`MATERIA_ZPP_ROOT` selects another prepared asset directory.

## Manual alternative

`new ZppWorkholding(60, 60, false)` retains the CV15075 on TS52-108 arrangement:
actual supplier geometry, 12 mm parallels, a 180 × 190 × 20 mm adapter with M10
holes on an 82 mm square, and M8-linked TS52 studs. Stock bottom is about
92.001 mm above the table. The CV10075 download is inventoried but not split into
moving members. `Rke92Vise` also remains available as a drawing-based reference.

## Verification

- `haxeon run --project machinekit/examples/bench-mill/haxeon.workholding.json`:
  geometry, seating, carrier motion, stud clearance, service ports and persistence
  for pneumatic and manual setups.
- `haxeon run --project machinekit/examples/bench-mill/haxeon.machining.json`:
  saved coupled state, setup conditions, datums and travel on both machines.
- `haxeon run --project app/tests/mill-missions/haxeon.json`:
  complete cuts, setup mismatch/reset, feed hold and 50/100/200% speed. Set
  `MILL_MISSION_BACKEND=mujoco` to repeat with MuJoCo.

Execution tests disable physical collision response and check CNC planning and
stock removal. They do not qualify fixture contact, friction or cutting loads.
