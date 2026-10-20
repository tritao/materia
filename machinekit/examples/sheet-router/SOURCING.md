# Guide-first redesign: catalogue basis

This selects sourcing requirements for the next prototype. Generic catalogue-class
CAD envelopes are not supplier drawings or evidence that arbitrary bought parts
have these ratings.

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

## Fabrication and access decisions after the braced-frame screen

The installation baseline is anchored to a structural slab at all six support
stations, using separate anchor brackets/base plates, steel levelling/packing and
grouted bearing seats. Do not rely on a rubber levelling pad to transmit anchor
shear. The structural screen assumes 50,000 N/mm in X, Y and Z at each station,
with no rotational clamp; these are provisional foundation stiffnesses requiring
anchor/slab qualification. Free-standing levelling feet are a sensitivity with
50,000 N/mm vertical at all six, and minimal 3-2-1 lateral springs of the same
stiffness (X at foot 0; Y at feet 0 and 3). This assumes friction without slip;
it does not establish sliding or overturning safety.

The sixteen-brace layout is a structural candidate, not an accepted service layout.
The front end X braces currently cross the 554.25 mm sheet-loading plane: straight
front loading is blocked. In the next CAD pass, lower the front pair's upper joints
to at most 430 mm, below the 479.25 mm plenum underside, retaining the lower-plan
triangulation and side/rear braces. Recheck frame modes before adopting that change.
Keep the front approach free across the full 1600 mm sheet width. Do not make a
load-carrying brace removable during operation.

Service the plenum from above with the spoilboard removed; zone valves must be
removable through side service openings. Put the vacuum header, pump and isolation
valves outside the right frame, with fixed hoses clamped along the longitudinal
members and routed around brace joints, not through the lower X crossings. Reserve
side openings for valve/tool removal and hose bend radii. Y cable chain stays
outside the left rail; X chain remains above/behind the beam. Existing assembly
clearance establishes chain envelopes, not flexible hose bend/service access.
The next CAD pass must route actual hoses and demonstrate each valve removal path.

Select bolted and dowelled aluminium Z ribs, with machined channels and bolted
closing strips, rather than welding a 6082-T6 assembly. The current two closed
side sections represent the fully composite stiffness envelope; fastener pitch,
interface slip and closing-strip compliance still require a joint design and
sensitivity before that stiffness can be credited for fabrication. Machine the
assembled rail face and guide datums after fabrication. Do not apply parent T6
strength to a welded heat-affected zone: loss is process dependent, as shown by
[this 6082-T6 welding study](https://doi.org/10.1007/s00170-025-15395-0).

Weld the steel beam feet before stress relief. Machine the Y-carriage mounting
faces and both X-rail seats in one fixture/setup after stress relief, using a
common datum scheme with the beam. Dual-Y squaring must follow those machined
datums, with final rail alignment and servo homing only trimming residual error.
Specify the stress-relief cycle with the weldment/material supplier; qualify local
tube-wall and weld stresses separately from the beam screen.

Recover the 60 mm spindle drop through a fork/riser redesign, not by accepting a
428 mm beam-centre-to-tool-tip lever permanently. Rack height alone cannot change
the plate-to-fork clearance at the same holder engagement: both move together at
pickup. Move the obstructing fork support/riser behind or outside the projected
Z-plate/rail ends, then raise the spindle datum 60 mm. Raise the work surface and
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
welded to the beam footprint (6 mm nominal fillet), and two closed 25 × 120 × 6 mm
side sections on the 20 mm Z web. The frame receives sixteen RHS 80 × 80 × 4
braces. Built-up aluminium joints, heat treatment and subsequent rail-seat
machining need manufacturing qualification. The router compiles against HEAD in an isolated detached worktree and both
beds export fresh FEM CAD packets. Assembly/clearance passes 426 assertions,
sizing/planner passes 82, and guide geometry/travel checks pass. The full
anchored structural rerun passes static X/Y targets but still fails 40 Hz. Mode shapes,
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
