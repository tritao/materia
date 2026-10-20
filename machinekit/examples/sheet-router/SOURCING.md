<!-- The DIY route decision below supersedes older whole-base machining assumptions. -->
# Guide-first redesign: catalogue basis

This selects sourcing requirements for the next prototype. Generic catalogue-class
CAD envelopes are not supplier drawings or evidence that arbitrary bought parts
have these ratings.

## Approved structure and motion decision (2026-10-20)

Retain the raised Z/table/rack geometry (`71f330894`) and add the anchored leg
pair at machine Y=1575 (`c71e0d17d`), in addition to the original middle pair.
The gross architecture screen estimated +25.4 kg fixed; actual union CAD adds
22.18 kg on the short bed and 22.34 kg on the long bed. Moving mass is unchanged.
Do not add the deep 300×200 side girders or change plates, guide size or spacing.

The user replaced the 40 Hz gate with static X/Y ≥2 N/µm plus ≤0.05 mm
motion-induced contour error and rapid residual. Evidence: `1ba79b551`.
Required MotionKit OM-R1 supplies jerk-limited path timing and optional ZV/ZVD;
per-move ZVD is the router fallback. SR6-C commissioning must measure modes and
damping, tune the shaper and re-check the motion budget before cutting acceptance.
The full raised-datum plus leg-pair rerun passes the revised model requirement;
freeze the overall structure for SR6. See PLAN SR-D3, `fem/sr-d3-results.json`,
and the current rerun record in `fem/README.md`. Prepared seam graphs and
132/132 reaction-qualified weld groups per bed pass the 100-million-reversal
screen. WPS/detail, material/relief, epoxy and shop-machining acceptances still
gate fabrication release in `fabrication/README.md`.

## DIY fabrication route (user-approved 2026-10-20)

No full-base machining centre. Use low-distortion 5 mm stitch welding after full
jig tacking; commercial thermal relief (quoted bare-frame envelope/mass) preferred,
VSR fallback with measured before/after/rest evidence. Epoxy-levelled rail seats
are baseline, adjustable jack-screw/shim mounts the alternative. The 12 mm seat
envelope is preserved with 8 mm nominal steel plus 2–6 mm pour (4 mm nominal).
Resin application approval and measured flatness/straightness are release gates.
Dual-Y squareness comes from measured diagonals and rail/pad adjustment.

Welded feet remain: the machined bolted/dowelled contact screen gives 1.909 N/µm
long-bed worst Y versus 2.044 welded, failing SR-D3 despite the DIY advantages.
Use an ordinary shop for the separate 2550.5 mm beam/feet, checking bed/travel and
indexed datum transfer. Full method and numerical tolerances:
[fabrication/DIY_ROUTE.md](fabrication/DIY_ROUTE.md).

SR6 weld duty is **100 million full-load reversals** at 3 m/s² plus the cutting
allowance. Use the FAT36 variable-amplitude cut-off, 12.669 MPa with γMf=1.15;
do not retain the provisional ten-million-cycle basis. The reaction-detail
corrections are full post/stringer runs, 6 mm legs at the middle post joints
(full-size tabbed ends on the short bed), and two accessible 92 mm, 8 mm-leg
fillets per beam foot with full-size tabbed ends. No all-around beam-end fillet
is credited. See [qualification basis](fabrication/WELD_QUALIFICATION.md) and
the full relieved-unit machining envelope/faces in the DIY checklist.

## Guides

Use two front-face X rails, 150 mm apart vertically, with two blocks per rail
300 mm apart along X. Z has two moving rails 180 mm apart and four stationary
blocks on the X carriage, 200 mm apart vertically. Y has two blocks per side
280 mm apart. The beam remains steel RHS 200 × 100 × 6.3, 200 mm side vertical.

[HIWIN's guideway catalogue, table 3.5](https://www.hiwin.de/medias/sys_master/hiwinDocumentMedia/h69/h7f/14750790811678/HIWIN%20CATALOGUE%20LINEAR%20GUIDEWAYS/HIWIN-CATALOGUE-LINEAR-GUIDEWAYS.PDF?attachment=true)
lists radial rigidity for HG20C ZA as 460 N/µm and HG25C ZA as 539 N/µm.
These are the selected preload/class requirements for the radial spring input.
The screen still assumes 100 N/µm for the other transverse direction; directional
supplier data must replace that assumption before qualification. Each block is
placed at its exported rail and mounting connectors. No local guide rotational
spring is used: resistance to rotation follows from the spatial block pattern.

## Racks

Do not resize on the old assumed 700 MPa Hertz limit. The
[manufacturer's helical rack rating table](https://www.atlantadrives.com/pdf/ads_rackratings-h.pdf)
lists 6.0 kN maximum feed force for module 1.5, 20 teeth, HPR quality 6, with a
case-hardened 16MnCr5 pinion and the specified hardened rack. Its reference conditions
include grease lubrication, 1.5 m/s, uniform tooth loading and safety factor 1.
The study compares the reserved demand and most loaded preload flank to this
rating, retaining a conservative 50% mounting derating for short rack segments.
Mounting screws, duty/application factors, lubrication and actual material/heat
treatment must still be qualified.

[Standard HPR rack dimensions](https://www.atlantadrives.com/pdf/atlanta.pdf)
include 19 mm nominal width for module 1.5 and 24 mm for module 2. The 19 mm
module-1.5 family is the sourcing candidate. The generic assembly retains a
conservative 20 mm external envelope until the supplier's exact drawing is
selected. There is no 25 mm upsizing decision. The assumed Lewis/Hertz calculation
remains visible as a diagnostic and is not the acceptance criterion for the
specified hardened family.

If a larger module is eventually needed, module 2 with 15 teeth preserves the
100 mm/revolution travel at the same helix angle. The pinion constructor now checks
virtual tooth count rather than imposing an unrelated minimum of 18 physical teeth.
The candidate still needs matching supplier availability and a mounting drawing.

## Z travel and speed

The prototype uses 200 mm travel and a 1610 screw. Screw speed, inertia, holding
brake demand and torque are calculated from the selected lead, not a hard-coded
5 mm lead. The 13 mm race root remains an assumption to replace with the selected
1610 drawing. The outboard screw uses its own 95 mm end allowances rather than
the longer Y guide margins, giving a 390 mm fixed/free shaft. The motor/bearing
stack must clear the moving plate throughout its stroke; the assembly clearance
gate checks this independently.

The revised load needs a 750 W braked Z servo. The heavier X carriage also needs
a 750 W X servo to preserve the 1.5 duty-torque reserve; changing the X reducer's
input bore follows that motor shaft. Reduction and 100 mm rack output travel per
revolution are retained.

At 3 m/s², a 30 mm rapid move takes approximately 0.236 s at 10 m/min, compared
with 0.388 s at 5 m/min, including ramps. This comparison is for rapid moves;
cutting/plunge feeds still follow the cutting process. The eight tool-rack gauge positions
and approach poses fit within the reduced Z travel. The 200 mm stroke is provisional
for the authored 120 mm longest-tool envelope: shorter-tool reach and the real tool-length
range still require qualification with the tool table.

## Raised-datum CAD selection

The clearance revision raises the complete Z guide/ram datum 60 mm rather than
moving the spindle relative to its plate. It preserves the 149 mm ram gap and
full moving-rail coverage while reducing beam-to-tool distance from 427.75 to
367.75 mm. The X plate remains 40 mm thick, with 60 mm more height for the fixed
Z blocks. Table, tool rack, fork seats and setter rise together. Beam underside
clearance remains 268 mm; no tested architecture variant was adopted.

Front end-brace upper joints are lowered from Z700 to Z430; the complete 1600 mm
width × 40 mm sheet mouth passes a native swept-volume check. Both beds pass the
full 426-assertion assembly sweep, guide coverage and 82 sizing/planner assertions.
The historical four midpoint spot checks retained ≥2 N/µm. The subsequent full
raised-datum/extra-leg pose/mesh rerun and new-mode dynamic checks pass the
revised SR-D3 requirement; see the current freeze evidence above. Moving Y mass
is 196.65 kg on the long bed.

## Motion-error and architecture decisions

The user replaced SR-D3's 40 Hz rule with the 0.05 mm contour/residual
motion-error allocation, jerk-limited timing and ZVD fallback. This allocation
comes from the review; it is not a universal wood/plastics tolerance. The actual CNC path backend does
not certify the shared 20 m/s³ jerk default. Native jerk-limited motion or per-move
ZVD is promising in the frozen-pose modal screen; neither has been installed or
qualified on the machine. Static cutting deflection, assumed 0.054 mm backlash,
servo tracking and damping still prevent a finished-part accuracy claim.

Do not source the 309 kg deep-side-girder change or the top/front X rail bridge from
this screen: neither improves the governing mode. One extra anchored leg pair
under the governing gantry position gains 1.43 Hz for 25.4 kg fixed mass; the frame
already has a middle pair, so this is a fourth pair, not a missing existing one.
Keep the 40 mm X plate and aluminium Z. The clearance-only CAD pass above raises
the entire Z-guide/ram datum 60 mm with the table/rack; raising just the spindle
would reduce the 149 mm ram clearance to 89 mm. Guide travel, the loading envelope
and assembly have passed; structural and fabrication gates remain.

## Fabrication and access decisions after the braced-frame screen

The installation baseline is anchored to a structural slab at all eight support
stations, using separate anchor brackets/base plates, steel levelling/packing and
grouted bearing seats. Do not rely on a rubber levelling pad to transmit anchor
shear. The structural screen assumes 50,000 N/mm in X, Y and Z at each station,
with no rotational clamp; these are provisional foundation stiffnesses requiring
anchor/slab qualification. Free-standing levelling feet are a sensitivity with
50,000 N/mm vertical at all eight, and minimal 3-2-1 lateral springs of the same
stiffness (X at foot 0; Y at feet 0 and 3). This assumes friction without slip;
it does not establish sliding or overturning safety.

The sixteen-brace layout retains the lowered Z430 front endpoints and the
full-width sheet-loading mouth. The raised-datum full rerun has passed. No
load-carrying brace is removable during operation.

Service the plenum from above with the spoilboard removed. The lower vacuum
ports shift 120 mm in Y to clear crossmembers. Six valves sit on external floor
stands, with the middle row shifted 300 mm in Y to avoid the extra posts. The
headers, pump, rear crossover and fourteen explicitly routed hoses are detailed
in the fabrication packet. Validate valve-withdrawal and top-inspection paths
against the actual purchased components. Y chain stays outside the left rail;
X chain stays above/behind the beam. No hose crosses the front loading mouth.

Select bolted and dowelled aluminium Z ribs, with machined channels and bolted
closing strips, rather than welding a 6082-T6 assembly. The current two closed
side sections represent the fully composite stiffness envelope; fastener pitch,
interface slip and closing-strip compliance still require a joint design and
sensitivity before that stiffness can be credited for fabrication. Machine the
assembled rail face and guide datums after fabrication. Do not apply parent T6
strength to a welded heat-affected zone: loss is process dependent, as shown by
[this 6082-T6 welding study](https://doi.org/10.1007/s00170-025-15395-0).

Retain welded steel beam feet after both screened bolted/dowelled candidates
failed the static Y requirement. Stress-relieve the separate beam/feet weldment,
then machine its Y-carriage and X-rail faces at an ordinary shop using confirmed
travel or indexed setups with measured datum transfer. Do not machine the full
base: epoxy-levelled rail seats or adjustable pads establish its datum. Dual-Y
parallelism and squaring follow measured diagonals and pad/rail adjustment;
servo homing reproduces the measured square. See the DIY procedure for tolerances,
relief evidence and the fail-closed release checklist.

The following lever discussion records the earlier coupled redesign, now
implemented and validated by the raised-datum/extra-leg freeze above.

Recover the 60 mm spindle drop through a fork/riser redesign, not by accepting a
428 mm beam-centre-to-tool-tip lever permanently. Rack height alone cannot change
the plate-to-fork clearance at the same holder engagement: both move together at
pickup. Move the obstructing fork support/riser behind or outside the projected
Z-plate/rail ends, then raise the complete Z-guide/ram datum 60 mm (taller X plate), carrying the
spindle with it. Raise the work surface and
setter datum by the same amount to preserve the 200 mm stroke and full-depth
cutting reach. Recompute rack heights, all pickup/approach poses, shorter-tool
reach, shoe lift and plenum supports in that CAD pass; do not claim it is already
clear. The lever target is about 368 mm. Keep the present verified pickup geometry
until this coupled change passes assembly and structural checks.

## Structural decisions

The full anchored screen gives worst X/Y 2.044/2.135 N/µm, a 24.177 Hz global
X frame-sway mode and 26.054 Hz gantry Y-sway/beam-roll mode. Keep X at 40 mm and
Z aluminium: 25 mm X loses the static target for only 0.390 Hz, while steel Z
adds 22.95 kg and lowers the mode. Do not buy HGR30 or longer Y feet on this mode's
behalf: their screened gains are negligible or negative. The next geometry pass
recovers the 60 mm lever and clears sheet loading as described above. Further
modal stiffness work belongs in the frame/beam/Y-drive path. See the complete
mode-energy and sensitivity tables in [fem/README.md](fem/README.md).


The working follow-up uses a 40 mm flat X plate, 30 mm steel feet continuously
welded to the beam footprint (the historical 6 mm fillet is superseded by the
SR6 tabbed 8 mm detail above), and two closed 25 × 120 × 6 mm
side sections on the 20 mm Z web. The frame receives sixteen RHS 80 × 80 × 4
braces. Built-up aluminium joints, heat treatment and subsequent rail-seat
machining need manufacturing qualification. The router compiles against HEAD in an isolated detached worktree and both
beds export fresh FEM CAD packets. Assembly/clearance passes 426 assertions,
sizing/planner passes 82, and guide geometry/travel checks pass. The full
earlier anchored rerun passed static X/Y but failed the former 40 Hz criterion. Mode shapes,
energy shares, sensitivities and the corrected-baseline evidence are in
[fem/README.md](fem/README.md); the results below describe the previous candidate.

Start with flat carriage plates and the spatial guide pattern. A box bridge has
not been selected. The first guide-only run moved the dominant Y contribution to
the 12 mm aluminium gantry feet; the settled candidate uses 20 mm steel feet.
The Z attachment is positioned to keep the longest tool about 430 mm below the
beam while retaining approximately 150 mm between the moving plate and spoilboard
at full Z. The lower gauge datum clears the moving plate and rail ends over the tool-rack
forks, including the approach path; the earlier closer datum collided at full Z.
The structural breakdown determines whether further plate changes are
needed. A direct two-level split-clamp saddle mounts to the Z plate, eliminating
the long horizontal shelf. The screen treats the saddle body as rigid with four
spatial translational mounting springs; clamp, spindle-bearing and tool compliance
still require qualification. The revised shoe uses a 150 mm body, 40 mm brush
and front-directed short hose neck to clear the close-coupled saddle and side screw.
These remain layout envelopes rather than supplier/manufacturing drawings.

Track the actual gantry mass and the Y drive-only frequency in every sizing report.
With the current 17.1 MN/m combined Y drive stiffness, extra mass reduces the
ideal drive-only frequency below the old approximately 54 Hz estimate. The preload
hinge contributes about 65% of the current series drive compliance. Neither an
ideal drive-only estimate nor a beam-only twist estimate qualifies the full machine.

The settled reduced screen still fails SR-D3: worst X/Y stiffness is 1.005/0.480
N/µm and the lowest classified gantry-dominant mode is 27.249 Hz. Z plate/rail
compliance and frame flexibility now lead the static budget. Both drive branches
pass the eccentric supplier-based screen, including 935.77 N maximum preload-flank
demand against the conditional 3000 N mounting-derated rack rating. See
[fem/README.md](fem/README.md) for convergence, classification limits and next work.

## Populated ATC selection (2026-10-20)

Use eight nominal ISO30/ER32 tools at 220 mm rack pitch with top-loading U forks.
The 110/150 mm layouts collide with the Z ram/offset screw when tools are parked.
The rack case is 1760 mm long over the existing 880 mm plinth; pocket centres
X=30..1570 remain reachable. The rack datum is Y=workY+280 mm. The setter moves ahead of the case
to Y=workY+60 mm (240 mm extra-travel margin), leaving the widened rack case
unobstructed and keeping XY transfers in the front approach lane. The cover is centred on the rack
rather than 20 mm forward of it, clearing the Z rails at the rear cutting edge. A 1780 x 300 x 3 mm steel cover has 30 mm upward
returns. See [ATC sizing and release conditions](atc/cylinder-sizing.json).
The 25 x 600 mm cylinder needs a separate guided slide: actual guides, mounts,
cushions, flow controls and their filled-rack sweep remain procurement/release
gates. The guide/adaptor mass allowance is 3 kg. The fixed Y-chain bracket arm
is below the Y preload swept envelope; its outboard post still supports the
same chain end. Independent stopped sensing and local pressure/clamp feedback
must be selected before hardware acceptance. Nominal simulation delays are not
hardware stopping-time ratings.

## Vacuum planning basis (SR8, 2026-10-20)

Keep six zones (2 x 3), independent 50 mm valves, the service-access routing and
nominal 75 mm trunk. The model reads the CAD spoilboard thickness (25 mm) and
permeability (1e-6 mm² = 1e-12 m²); 18 mm is a hydraulic sensitivity only.
The recipe's 300 m³/h / 25 kPa / 5.5 kW blower remains assumed. A linear curve,
Darcy leakage, declared hose/filter losses, friction 0.5, effective area 0.8 and
factor of two are planning inputs, not supplier ratings or measured hold.

At the new-board baseline the 50 mm freed part has 17.06/15.80 N lateral hold
on the two beds against 600 N required. The 18 mm sensitivity reduces that to
15.19/13.79 N. Retain small parts with onion skin/tabs or a separately qualified
fixture; their connection strength and actual final-pass force belong to SR13.
Measure the installed pressure/flow and material friction before cutting
acceptance. See [the native report and zoning](vacuum/README.md).

Do not treat hydraulic board-thickness sensitivity as approval to skim the
machine datum: the 120 mm reference tool only reaches the new board plane at
full Z. Actual tool/holder gauge lengths, sacrificial overcut and surfaced-board
reach remain open for SR13; the structure stays frozen while these are resolved.
