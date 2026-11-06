# Physical rack mount implementation

Qualification in progress, 2026-10-28. The implementation is authorized by the
approval of `LIGHTWEIGHT_PROPOSAL.md` (eeb6ff4e0). Commit 2760c26cf landed the
mount and strict mount-required `driveRack` migration together while validation
was ongoing; the following cleat and acceptance fixes are a follow-up. This file
does not release fabrication or close SR5/SR6.

## Finished layout

The rack remains ATLANTA 29-15-055, in whole 500 mm sections, with its own four
mount connectors and two 5.7 mm pilot connectors per section. Receiver drilling
uses `MountedElement` and `MountHoleDerivation`, retaining the installed rack
occurrence, connector and fingerprint. No supplier hole positions are invented.
Pitch is set with a rack joining gauge. The integral adjustment bar stands clear
of the rack; M4 push screws set the side position before the mounting screws are
locked. Pinion centre distance is adjusted at the carrier, not fixed by a lip.

Each receiver is C45, 498 x 26 x 18 mm, with a 6 mm integral adjustment flange
36.6 mm deep. Receivers have 2 mm gaps on the 500 mm section pitch. Two sourced
1.6 mm washers at each rack bolt provide stand-off; their added thickness is
matched by the 12.6 mm outward shaft-layout shift, preserving the pitch height.
Nominal pinion-face clearance becomes 2.7 mm. Both Y racks are below the pinions.
Measure/select each washer pair to 3.20 +/-0.10 mm; the tolerance allocation is
an assembly target, not a claim about unverified washer thickness tolerances.

Five C45 cleats per section sit at -220, -125, 0, 125, 220 mm. Each finished
base is 28 x 64 x 13 mm, with an 11 mm web and geometry-dependent height. Base
screws are two recessed M6 class 10.9 through bolts at +/-18 mm; web screws are
two M6x20 class 10.9 at +/-14 mm, entering the receiver 10 mm. The wider web pair
leaves at least 4 mm finished steel between the source locating bores and cleat
threads. Every axial receiver bore pair is checked against this net-ligament
rule. Four millimetres of replicated bedding separates each cleat and host.
The connection screen includes net section, bearing, prying, slip, bolt
interaction, washer bearing, counterbore-floor punching and base-edge tear-out.
Its 1000/400/400 N envelope is an engineering input, not final J7 qualification.

After pitch and side adjustment, match-ream the source pilots and receiver to
6 H7. Two ground 6 h6 x 30 mm C45 locating pins per section engage the receiver
7.8 mm, with 3.2 mm blind-tip clearance. `GearRackRun.finishPinDiameter` records
this shop operation separately from the supplier's 5.7 mm pilot dimensions;
the run mass subtracts the additional drilling. Pins locate a remounted section;
no cutting-load capacity is credited to them. They are removed with the receiver
for bench disassembly; qualify the extraction method before shop release.

## Captured nuts and service access

Each cleat reuses `LinearMountNutCassette` in its compact centre-service layout:
58 x 18 x 10 mm, two actual M6 nuts and washers, a 2 mm retaining cover and two
M3x6 cover screws. Nuts and washers bear directly on the RHS underside; the cage
and cover only prevent rotation and loss. The cover's explicit 2 mm ligament
allowance is checked for a 100 N handling load and is not a structural thread
or bolt allowance. Specify class-10 nuts and suitable hardened washers when
buying hardware; the current nominal catalogue recipe does not verify a supplier.

One 24 mm insertion opening per 500 mm section is on the Y frame's inboard face
or the X beam's front face, at the tube's neutral height. The caller declares
that face explicitly. This avoids underside openings over legs (notably Y≈1575)
and removes the two underside socket holes per cleat. The ordinary clearance-hole
edge rule is retained; no access-hole ligament override is needed.

Feed the assembled cassette end-first through the opening:
sqrt(18^2+15^2) < 24 mm. Its turn envelope fits inside the 87.4/88 mm cavity.
Move it at neutral height with a magnetic pickup/retrieval tool, at most 220 mm
along the tube, then lift and align with temporary guide rods from the base-bolt
holes. Withdraw both base bolts before retrieving a cassette. Other installed
cassettes remain above this transfer corridor. Cover screws are serviced on the
bench; base heads are torqued from above with the carriage parked away. The
native check must qualify the outside approach against braces, table structure
and services. Net host sections still enter the compliance/fatigue screen.

The hex-key check uses a conservative L-key envelope and both handle directions.
The confined web screws need a 5 mm key with a shortened 12 mm overall short
leg and at most a 5 mm centreline bend radius. Recessed rack/base and other cap
screws use a 16 mm short leg; larger root keys use the recorded projection. Confirm the actual tool's bend and dimensions,
and keep a hardened key cool while shortening it. A long straight shaft is not
a valid access test for the recessed web screws beside a fixed rail.

## Functional radial pivot

A bounded passive lever carries the actual motor, reducer and pinion. It has
no parallel rigid motor-support path. Its hinge axis is the shaft direction;
an 80 mm lever moves the pinion radially. A 56 mm rear spring arm, adjustable
disc pack, positive stroke stops, 10 mm retained pivot pin and bronze bush are
physical parts. The pinion datum is at its axial centre. The bounded stroke is
-0.1/+0.5 mm radially. The disc-pack force/rate and loaded-height range remain
assumptions pending a supplier curve and commissioning evidence.
Bronze thrust shims nominally 0.99 mm per side replace the otherwise empty
1 mm clevis gaps. Finish/select them against the measured gap for 0.01--0.02 mm
total axial endplay. The sleeve/pin clearance and residual axial endplay remain
mechanical lost-motion inputs for the local screen and measured commissioning;
they must not be silently treated as an ideal rigid bearing.

The anchor is machined C45 with a closed-section reach arm. Y anchors mount to
the 30 mm steel gantry feet; X mounts to the 40 mm aluminium carriage. Four blind
M8 roots replace underside nuts that crossed guides and cleats: 17.4 mm steel
engagement with M8x25, and 25.4 mm aluminium engagement with M8x35, both class 8.8.
Their explicit
material policies use 24/31 mm drill depth and 21.5/28.5 mm usable thread
(steel/aluminium), with one pitch (1.25 mm) free usable thread beyond the tip
and 6 mm minimum back wall. Drill-bottom clearance is reported separately. A 9 kN installation target, 15% tightening scatter and 6 kN external
increment are provisional stripping-screen inputs; model reactions and tightening
qualification must replace them. X screw columns sit outside the reach-arm walls
on a 94 mm root flange so washers and sockets are accessible.

## Native acceptance and tolerance allocation

`RadialPinionTests` checks motor ownership, the axial-centre datum, single-solid
parts, bounded radial displacement and stroke-stop clearance. `RackMountClearanceChecks`
checks same-body conflicts too. A thread fit requires coaxial matching diameter
and pitch, full engagement, plain-shank and blind-tip checks, plus native clearance
outside that one declared thread's major-diameter envelope.

`RackMountTravelChecks` certifies continuous relative-translation intervals using
native distances and the 1-Lipschitz bound. It includes full axis overtravel,
accessory slides, conservative shaft-rotation envelopes and bounded pivot motion.
Unresolved intervals fail. Fixed/pivot fits and tooth mesh need their separate
checks; this translational certificate does not exempt them.

Pinion rotation uses a tip-radius cylinder whose containment of the actual
lofted teeth is checked natively. For the passive pivot, the pinion centre
follows an 80 mm radius arc. An enclosing disk about that arc's midpoint has
radius `2 R sin((upper-lower)/4)`; adding the rotor radius encloses every angle,
with 0.001 mm numerical padding. Native forward-kinematics containment at both
ends and zero checks the frame/sign implementation. The actual enclosure adds
0.3010015 mm about a centre displaced about 0.2000014 mm, rather than inflating
in every direction by the old 0.6185 mm bound. The original diagonal-of-box
rotor and isotropic pivot diagnostics remain diagnostic failures, not geometry
defects or acceptance evidence. Coaxial bush/thrust faces retain their selected
positive axial clearance; a reducer journal may start at its output plane.
Both interfaces have explicit geometry checks and pivot regressions.

The proposed independent-body clearance allocation is 1.5 mm: 0.50 mm weldment,
0.15 mm datum setting, 0.40 mm catalogue rack face width, 0.20 mm shaft/axial
setting, 0.10 mm machined parts, 0.10 mm hardware stack and 0.05 mm reserve.
These are manufacturing/assembly targets, not measured capabilities. Mating
threads, locating fits, bearing fits and hard stops use explicit fit checks
rather than a 1.5 mm separation rule. The 3150 mm bed passes fixed clearance
(5,842 native pairs, 572 qualified interfaces, zero intersections or tolerance
failures), service access (636 trials, zero clashes), and continuous travel
(1,239,253 certified pairs, 30,610 native distances, zero failures). Updated pivot
regressions pass 582 assertions across both beds. The 2600 mm bed also passes
fixed clearance (4,727 native pairs, 460 qualified interfaces, zero intersections
or tolerance failures), service access (520 trials, zero clashes), and continuous
travel (961,085 certified pairs, 28,157 native distances, zero failures).
The shared-caller and existing router regression suites are still pending.
Service trials reroute both cable chains at every actual parked coordinate;
the 520/636-trial reruns also pass. They cover both ends and the middle service
park, rather than testing the chains' static zero-position mesh.

## Remaining model screen and release gates

Use actual built moving/fixed mass and inertia, including the X rack supports,
cassettes, pivot hardware, removed motor plates and host drilling. The local
solve must include receiver bending/torsion and net holes, cleat and host-pad
flexibility, bolt bearing/prying, bedding, carrier, anchor and pin/bush. Condense
work at the pitch point, including the motor-housing torque reaction; the old
end-anchor stiffness and old radial-only projection do not represent this mount.
Use the verified existing J6 histories for a nominal-stop-only prediction.

After reduced-stop executed histories arrive, one final final-geometry campaign
closes SR5 braking error (<=0.05 mm on both beds) and SR6 executed-J7 dynamic weld
stress/fatigue together. Do not qualify SR6 on the obsolete frozen load path.
Cycle time is outside frame acceptance. No structural acceptance is claimed here.

## Motion input integrity

The existing nominal-stop J6 handoff was reverified on 2026-10-28: all 186
histories and 755 unique output, manifest, immutable producer and loaded-library
files matched their recorded SHA-256 values. The [compact receipt](results/j6-input-hashes-20261028.json)
retains the three index hashes. This is input integrity only; the final mount
model has not yet been replayed, and these histories do not substitute for the
required executed reduced-stop final campaign.

The first final-source short-bed fixed check found zero solid intersections in
4,849 native pairs, but rejected the 1.4 mm cleat-to-Y-rail gap and tangent
positive-limit posts. The cleat toe is reduced by 1 mm (the connection screen
retains counterbore-floor tear-out), and each positive-limit post steps 3 mm
away from the pad before rising above it. These revisions require a fresh
clearance pass; the earlier zero-intersection result is diagnostic only.

Preload service also requires a 20 mm open spanner within a 14.2 mm head radius
and 3 mm thickness, plus a 16 mm ISO 4032 locknut spanner within a 12.2 mm head radius
and 5 mm thickness; handles are checked to 90 mm. Verify purchased tool bounds
or make the specified thin plate tools before commissioning. Their occupied
volumes are checked independently from the socket keys.

The run BOM expands to the supplier section part number and section quantity;
finished 6 H7 pilots remain a shop operation on that purchase. The run continues
to own its declared physical mass once. Procurement aggregation and the absence
of a second mass multiplication have separate regression checks.

The relocated switch checks use the existing assumed `GENERIC-INDUCTIVE-M8`
barrel and bored-holder geometry. Supplier-specific sensor retention is not
modelled by that baseline: its M8x1 threaded region starts after the connector,
and the holder must ultimately grip the threaded region with compatible
fixings. For example, SICK's [accessory-family drawing](https://www.sick.com/media/pdf/0/50/450/dataSheet_BEF-MU-M12N_2103188_en.pdf)
lists an M8x1 mounting nut, 13 mm across flats and 4 mm thick; the [sensor sheet](https://www.sick.com/media/pdf/7/17/517/dataSheet_IMB08-04NPOVT0K_1072695_en.pdf)
identifies the matching two-nut accessory. These are sourcing references, not
an adopted sensor or a purchase specification for this machine. Final sensor
selection, holder clamping geometry, fasteners and their clearance remain an
SR17/SR18 fabrication-release dependency. The rack-mount clearance result must
state this model boundary and must not release the switch holders for manufacture.

The long-bed fixed check found four guide/web-head gaps of 0.9 mm, despite
zero solid intersections. The web washer recess is now 2.6 mm deep, moving the
unchanged fully loadable M6 socket heads inward by 1 mm. Cleat length increases
to 64 mm to retain the conservative 235 MPa / 1.5 net-section screen, without
credit for a higher C45 yield strength. The two end cleats and bedding shift
their stock centres 3 mm inward, preserving their outer bounds at +/-249 mm
and the receiver joint gaps. Bolt stations and all axis/rail datums are unchanged.
This revision passes fresh fixed, service and full-travel acceptance on both beds,
as recorded above. The local section/connection fixture passes 64 assertions and
has 4.636902 kg of gross mount members per 500 mm fixture. That fixture value is
not the machine's added moving mass: the model screen must include all installed
parts, removed supports and host drilling before reporting the net change.
