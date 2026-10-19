# GE-class moving-column hobber

This is a pure vertical hobber. The direct-drive C table is fixed; X moves the
column towards the table on twin bed guides, Z moves the head on the column,
A swivels directly on the Z saddle, and Y shifts the hob on twin rails.
Turning belongs to the separate lathe plan. This checked design is not a released
manufacturing drawing set: stiffness, guide preload, fits and drive friction
remain assumed inputs.

## Coordinates and load path

`MACHINE_FROM_WORK` is an explicit +90° rotation about C. Work-frame radial X
therefore becomes global Y; the untilted hob axis becomes global X. All cutting,
synchronisation and force calculations retain their work-frame conventions.
The frame regression compares actual assembled poses, cut boundaries and loads
with the analytic reference chain, rather than changing the cutting definition.

The bed is 1000 × 650 × 210 mm, with a work well and drain below the table.
Its table seat, two column rail seats, drive seat and lifting points use shared
`machinekit.structural` casting recipes and separately listed steel inserts.
Precision mounts land on inserts, never on mineral cast. The recessed column is
400 × 200 × 500 mm, on a steel carriage and twin preloaded HGR25 bed rails.
Z uses twin 480 mm HGR15 rails with four blocks. A bolts directly to its saddle;
Y uses twin HGR15 rails and four blocks. Ground screws and linear scales serve
all three translations; screw manufacturing grades and preload need procurement
confirmation.

| Quantity | Previous overhead bridge | Moving column |
|---|---:|---:|
| X guide-plane to hob centre | 347 mm | 121 mm |
| Z guide-plane to hob centre | 325 mm | 125 mm |
| Y guide-plane to hob centre | 137 mm | 137 mm |
| X moving mass | 81.71 kg | See generated layout report |
| Z moving mass | 106.79 kg | See generated layout report |
| Y moving mass | 18.01 kg | See generated layout report |
| Machine mass | 351.61 kg | See generated layout report |

Offsets are distances to the nearest guide plane, not predictions of angular
stiffness. Masses include generated bodies at their stated material densities;
`layout-metrics` reports the current masses and continuous-drive thrust margins.
Drive demand includes 0.5 m/s² acceleration, Z gravity, assumed 100 N process load
and 100 N friction, with 20% continuous-thrust reserve. These assumptions do not
replace the HB10 measured-load report or modal analysis.

## Travel, fixture and support

The target envelope is flexspline outside diameter 25–140 mm, face width up to
20 mm. X retracts to 250 mm for top loading; Z spans -10..190 mm, Y ±10 mm,
and A ±15°. Guide checks include block length, end margins and overtravel.
An integral 40 mm flange and mandrel base replaces the stacked riser assembly;
it preserves tooth-band height and tilt clearance. The mandrel sleeve continues
to support the toothed rim.

The spindle locates the arbor axially. The outboard 15 × 32 × 9 mm radial bearing
is explicitly axially floating (±1 mm assumed allowance); its 22 mm journal
provides the required float margin. It is not a second locating bearing pair.
Fits, retention hardware, lubrication and demonstrated radial stiffness remain
manufacturing-design work.

## Chips and enclosure

A sloped annular table guard drains into the tray below the work. A drilled
coolant-manifold placeholder reserves flushing provision. Four rigid rail-end
shields cover non-travelled guide segments; moving bellows are a reserved
provision, not a validated fabricated guard. The enclosure recipe reserves
the machine envelope and top loading opening; its panels are currently omitted
from the assembly at the user’s request for unobstructed inspection. Door interlocks,
fluid containment and a complete conveyor are not implemented.

## Acceptance and remaining work

Mechanical checks cover mounted-solid contact, insert-only precision mounts,
rail margins, connected fabricated saddles and persistence. Exact-solid audits
check the initial/cutting poses and the declared X/Y/Z/A grid, permitting only
the intentional hob/blank cutting intersection. A grid is sampled clearance
evidence, not a continuous swept-volume proof.

HB6 now concerns this moving-column assembly, integrated fixture, twin Y rails,
floating outboard support and chip/loading provisions. HB9's app smoke check is
a coarse physical cut; it cannot establish reference ISO grade or HB12's full
reference cycle-time acceptance. Frozen HB4/HB11 cuts use their original layout
and remain separate evidence. Structural/modal analysis, spindle/mandrel sag,
thermal behaviour and detailed manufacturing release remain open plan items.
