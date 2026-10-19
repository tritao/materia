# Supported SFU2005 / HG20 axis

`SupportedSfu20Axis` assembles the Vallder standard SFU2005 ends, BK15 fixed
support, BF15 floating support, DSG20H nut housing, two HGR20 rails and HGH20CA
blocks. The default layout has 300 mm travel, a 600 mm screw, 450 mm rails and
180 mm rail spacing. `Sfu20AxisLayout` validates travel, spacing and support limits;
it does not scale purchased components.

```sh
export MATERIA_ROOT="$PWD"
./haxeon/scripts/haxeon run --project machinekit/examples/linear-rail-reference/haxeon.sfu20-seats.json
./haxeon/scripts/haxeon run --project machinekit/examples/supplier-fasteners/haxeon.sfu20-axis.json
./app/run-built.sh --project="$PWD/build/suppliers/sfu20-axis/sfu20-axis.mtrg" --perspective
```

Prepare the Vallder support library and HG references as described in
[the motion-family instructions](../linear-rail-reference/FAMILIES.md).
The ordinary MachineKit registry includes all fabrication and purchased recipes.
The `.mtrg`, `.assembly.json`, `.step`, `.bom.json`, `.sourcing.json`, `checks.json`
and `clearances.json` files are exported under `build/suppliers/sfu20-axis`.
The Machine travel joint is marked driven; the preferred camera faces the components.

## Axial stack

The pinned BK15 and BF15 compounds include bearings and seal sleeves. These stay
inside their support BOM items. `Vallder15SupportSeats` adds named seating datums
only for the two audited asset hashes. `Sfu20SeatAudit` checks the actual inner-ring
and sleeve solid bounds before exporting the source evidence.

| Interface | Shaft-local Z, mm | Assembly Z, mm |
| --- | --- | --- |
| Coupling end | 0..20 | 40..60 |
| M15x1 root envelope | 20..30 | 60..70 |
| Relief (included in the drawing's 12 mm dimension) | 30..32 | 70..72 |
| Plain fixed journal, D15 | 32..60 | 72..100 |
| BK15 front sleeve | 26..35.5 | 66..75.5 |
| BK15 two bearing inner rings | 35.5..53.5 | 75.5..93.5 |
| BK15 rear sleeve | 53.5..60 | 93.5..100 |
| BF15 bearing, D15 x 9 | 587..596 | 627..636 |
| Output ring groove, D14.3 x 1.15 | 596..597.15 | 636..637.15 |

BK15 faces the motor after a 180 degree turn about Y. Its shaft origin is at
assembly Z=86; the standard race shoulder bears on the included rear sleeve at
Z=100. An M15x1 retaining nut seats against the included front sleeve at Z=66.
The nut model uses the BK15 table's SW22 and an explicit assumed height of 6 mm,
occupying shaft Z=20..26 within the published thread. It remains an unsourced
reference component; confirm the supplied nut and its anti-loosening/preload
procedure when specifying procurement. No keyway or shaft machining change is
introduced. A DIN471-15 reference ring occupies the published floating-end groove.

The SFU25 drawing uses the same dimension convention: 17 mm includes its relief,
so that journal starts at 23+17=40 mm. Both corrections have family regressions.

## Installation and reusable parts

Only the base, two risers, carriage and NEMA23 bracket are fabricated. DSG20H is
flipped under the carriage, with the SFU nut turned 270 degrees about Z to align
its flats and all six holes. The housing attaches directly to the carriage.
A StepperOnline 23HS22-2804S motor drives the D12 input through the existing
6.35/12 sleeve generator. The coupling and radial screws rotate with the shaft.
Bare M4 heads are used on the motor's flange reliefs; no incompatible washers.

All mounting screws have installed geometry and independent BOM entries. Base
and riser threads receive their screws; the purchased supports retain clearance
bores. Carriage counterbores leave 6 mm grip for M5x12 into the HGH20's published
6 mm tapped depth. The motor bracket foot is below the rotating clamp-head envelope.

Clearance validation checks every component pair at start/mid/end, near the end,
and eight shaft phases across one revolution. Relative-pose memoization avoids
repeating identical rigid-pair booleans. Only each screw's declared threaded
receiver and the two radial screw/shaft contact pairs are exempted. The output
records every exemption. This is a sampled installation check, not continuous
collision proof or a structural/preload qualification.

DSG base/flange thread depths (12/10 mm), race root (16 mm) and retaining-nut
height (6 mm) remain explicit, persisted modelling assumptions. The supplier's
site establishes the intended component family; geometry checks do not invent
missing tolerances or material/load properties.

Sourcing is optional. The export selects exact Vallder recipe identities for
the screw, nut, housing, BK/BF supports, rails and blocks. It carries no invented
stock status or piece prices; stock length/end-option pricing requires a quote.
Motor, coupling, retaining hardware, fasteners and fabrication remain visible
engineering requirements even without selected offers.
