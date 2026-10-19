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
The bed has two continuous longitudinal casting beams under the
50 mm rail-seat inserts. Bed guides are spaced 370 mm apart (centres ±185 mm),
and the carriage is widened to suit. Lightening cores stay outside 110 mm beam
bands; the chip well leaves at least 90 mm beam width at the guide seats.
Each complete bed rail-seat footprint is backed by solid casting down to the sole;
the work well remains 300 mm wide throughout for top-down chip-tray installation.
The steel top faces deliberately stand 10 mm above the casting for rail mounting.
Full-footprint solid checks verify the bed load beams and at least 10 mm of backing
behind both column rail inserts. No rail mounting face overhangs a casting cavity.

Its table seat, two column rail seats, drive seat and lifting points use shared
`machinekit.structural` casting recipes and separately listed steel inserts.
Precision mounts land on inserts, never on mineral cast. The recessed column is
400 × 200 × 500 mm, on a steel carriage and twin preloaded HGR25 bed rails.
Z uses twin 480 mm HGR15 rails with four blocks. Its vertical saddle carries
a 25 mm deep swivel housing on its front face. A 180 × 150 × 16 mm crossed-roller
bearing surrounds an assumed 150 × 80 × 20 mm torque motor; a 200 × 180 × 10 mm
clamp shares the outer radial bay. The rotating output and vertical swivel plate
carry twin 220 mm Y rails on their front face, with four blocks and 120 mm
vertical separation. The spindle and floating outboard bracket share the vertical
Y saddle. An open C bracket supports the rear spindle housing; there is no
overhead roof. These compact motor and bearing dimensions and ratings require
supplier confirmation; geometry checks do not establish a thermal duty rating.
All twelve carriages use pinned HGH25CA (X) or HGH15CA (Z/Y) supplier CAD,
including wraparound sides and lubrication fittings. X uses matching supplier-derived
HGR25R rails; Z/Y retain the verified analytical HGR15R running profile. These
portable recipes require explicit local supplier-asset preparation. The source
CAD does not qualify load ratings or mixed-material carriage mass.

The X screw runs centrally between the bed rails, at Z=180 mm, in a 110 mm wide
recessed casting channel. Its nut carrier drops from the carriage into the channel;
a steel deck mounts to a separate ground-after-casting steel insert. The fixed
support and motor sit at the rear; the screw axis points towards the table.
The screw transmission sign follows that reversed axis. The insert starts beyond
the chip guard, with 2.5 mm nominal radial clearance. A channel cover/bellows remains
a design provision; the exposed mechanism is an inspection model.

Ground screws and linear scales serve
all three translations; screw manufacturing grades and preload need procurement
confirmation.

| Quantity | Previous overhead bridge | Moving column |
|---|---:|---:|
| X guide-plane to hob centre | 347 mm | 121 mm |
| Z guide-plane to hob centre | 325 mm | 161 mm |
| Y guide-plane to hob centre | 137 mm | 70 mm |
| X moving mass | 81.71 kg | 132.06 kg |
| Z moving mass | 106.79 kg | 41.60 kg |
| Y moving mass | 18.01 kg | 14.57 kg |
| Machine mass | 351.61 kg | 425.30 kg |
| Bed footprint (radial × tangential) | 650 × 750 mm | 1000 × 650 mm |
| Installed-pose envelope, including separate rack | 1225.63 × 750 mm | 1640 × 650 mm |
| X continuous thrust margin, same assumed demand basis | 911.15 N | 886.51 N |
| Y continuous thrust margin, same assumed demand basis | 897.29 N | 911.02 N |
| Z continuous thrust margin, same assumed demand basis | 859.36 N | 1542.40 N |

The initial moving-column revision had a 125 mm Z guide offset but retained the
old underhung head. The front-mounted head needs 36 mm more radial space: its
Z guide offset is now 161 mm, slightly beyond the approximately 150 mm target.
The swivel-bearing front face is 114 mm behind the hob, its rotating output face
100 mm, and the Z saddle/swivel mounting face 135 mm. The bearing is physically
in front of the saddle. X remains 121 mm.

[LAYOUT_SECTION.svg](LAYOUT_SECTION.svg) is a dimensioned radial section schematic;
[LAYOUT_REPORT.json](LAYOUT_REPORT.json) records measured masses, datums, demand
basis and validation. Y-offset guides and spindle housing are projected into the
section. It is not a manufacturing drawing.

Offsets are distances to the nearest guide plane, not predictions of angular
stiffness. Masses include generated bodies at their stated material densities, with the
separate electrical rack included and the enclosure omitted;
`layout-metrics` reports the current masses and continuous-drive thrust margins.
Drive demand includes 0.5 m/s² acceleration, Z gravity, assumed 100 N process load
and 100 N friction, with 20% continuous-thrust reserve. Y includes its gravity
component at the worst declared ±15° tilt; Z includes full gravity. These assumptions do not
replace the HB10 measured-load report or modal analysis.

## Travel, fixture and support

The target envelope is flexspline outside diameter 25–140 mm, face width up to
20 mm. X retracts to 250 mm for top loading; Z spans -10..190 mm, Y ±10 mm,
and A ±15°. Guide checks include block length, end margins and overtravel.
An integral 40 mm flange and mandrel base replaces the stacked riser assembly;
it preserves tooth-band height (Z=363 mm) and tilt clearance. The mandrel sleeve continues
to support the toothed rim.

The spindle locates the arbor axially. The outboard 15 × 32 × 9 mm radial bearing
is explicitly axially floating (±1 mm assumed allowance); its 22 mm journal
provides the required float margin. It is not a second locating bearing pair.
Fits, retention hardware, lubrication and demonstrated radial stiffness remain
manufacturing-design work.

## Chips and enclosure

A sloped annular table guard drains into the tray below the work. A drilled
coolant-manifold placeholder reserves flushing provision. The stationary rail-end
shields have been removed at the user’s request. Rails, blocks and the central
screw remain exposed; bellows with suitable chip protection and attached end
frames are an open design item, including compressed-length checks at both
travel limits. The enclosure recipe reserves
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

The continuous bed beams add 79.51 kg to the fixed casting, and the wider carriage
adds 1.43 kg to X. Total calculated machine mass is now 424.21 kg (previously
343.27 kg). X moving mass is 130.98 kg and its assumed-demand continuous thrust
margin remains 886.51 N. Y/Z masses and margins are unchanged. Structural/modal
qualification remains open; these are geometry and mass checks.

## Foundation assembly sequence

Cast the bed with the steel inserts located as child parts, then grind or replicate
the steel mounting faces after cure; do not machine datum surfaces into mineral cast.
Inspect the rail-seat planes and backing before fitting the guides. Lower the chip
tray into the open well before the moving column/head are fitted; its insertion
path is checked with the bed guides, screw supports and table present. Fit and align
the bed rails with the column carriage removed, preserving access to every rail
mounting bore. Align the master guide first, then indicate the second guide parallel
before tightening. Install blocks, carriage and column afterwards; fit and align the
column guides before adding the Z saddle and head. The central screw deck remains
separate and accessible through its channel. Mounting threads, retention details,
fastener selections and tightening procedures still need manufacturing drawings;
this layout check is not structural or mould-process qualification.


### Hob spindle cartridge (2026-10-19)

The former arbor-at-an-empty-gauge-line model is replaced with a cartridge load
path: hob → removable arbor → taper and face → hollow spindle shaft → opposed
angular-contact pair → stepped housing → Y saddle. The outboard bearing remains
radially supporting and axially floating. This is an **assumed custom TF20**
interface, not an HSK-E25 claim. All dimensions below are assumed, in millimetres,
measured along the spindle from the housing rear datum; the tool face stays at
180, so the cutting geometry and work-frame kinematics are unchanged.

| Feature | Axial interval / dimensions |
| --- | --- |
| Motor envelope | 20–120; Ø25 bore on spindle shaft |
| Opposed bearing pair | 129–146 and 146–163; Ø25 journals, Ø62 housing seats |
| Outer-ring retention | Housing shoulder at 129; removable threaded gland at 163–180 |
| Inner-ring retention | Shaft shoulder at 163; shaft locknut at 123–129 |
| Spindle nose | Ø40, 170–180; removable gland clearance bore Ø42 |
| Taper engagement | 160–180; Ø16 small end, Ø20 large end |
| Arbor face flange | Ø40 × 4, starting at unchanged gauge face 180 |
| Torque transmission | Two Ø4 axial pins on Ø28 PCD; 2 mm engagement in flange |
| Drawbar | Ø6 shank through hollow shaft, head on rear face; 12 mm engagement into arbor |

The taper locates radially, face contact locates axially, the drawbar holds both
contacts closed, and the pins transmit torque. Assembly sequence: fit the motor in the rear housing bore; slide the matched
bearing pair onto the shaft from its rear against the shaft shoulder, then fit
and tighten the rear locknut. Fit the shaft/pair from the front against the housing shoulder, slide the Ø42-bore front gland over the Ø40 nose and secure it, then insert
the arbor and tighten the drawbar from the rear. Access to the rear drawbar must
remain available. The BOM lists shaft, drawbar, gland and locknut separately.

Bearing envelopes do not resolve individual races. Thread helices, fits, preload,
drawbar tightening force, pin strength, balance, sealing and grinding tolerances
remain manufacturing design requirements; the CAD does not qualify them. In
particular, simultaneous taper/face contact must be specified and verified during
manufacture. The existing runout assumptions are retained, not requalified.

The cartridge adds 1.086 kg: total mass 425.297 kg; moving X/Y/Z masses
132.064 / 14.574 / 41.603 kg. Continuous thrust margins under the existing assumed
demand are X 885.968 N, Y 907.722 N, Z 1531.207 N. Guide-to-hob offsets and bed
footprint are unchanged.
