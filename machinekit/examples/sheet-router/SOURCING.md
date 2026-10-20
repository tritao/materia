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

## Structural decisions

The working follow-up uses a 40 mm flat X plate, 30 mm steel feet continuously
welded to the beam footprint (6 mm nominal fillet), and two closed 25 × 120 × 6 mm
side sections on the 20 mm Z web. The frame receives sixteen RHS 80 × 80 × 4
braces. Built-up aluminium joints, heat treatment and subsequent rail-seat
machining need manufacturing qualification. The router compiles against HEAD in an isolated detached worktree and both
beds export fresh FEM CAD packets. Assembly/clearance passes 426 assertions,
sizing/planner passes 82, and guide geometry/travel checks pass. The full
structural rerun remains pending. The diagnostic and corrected-baseline evidence is in
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
