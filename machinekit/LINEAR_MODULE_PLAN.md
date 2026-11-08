# Precision screw linear module

## Decisions

- **Structure:** castings provide mass; precision belongs in bought ground plate and flat bar.
  Bars provide only derived height spacing. Derive heights from the catalogue/stock stack.
  Locate rails at reference edges, never with dowels. Fixture-replicate or indicate screw supports, then match-pin;
  adjust scales at the head. Structural joints are bolted, dowelled for assembly, bonded only
  where needed. Buy ground stock first; replicate failing faces before custom grinding.
- No welding, legacy alternatives or parts merged just for count. Bolts, holes and pins come
  from BoltedFace/PinnedFace and JointFasteners; declare fittedBefore and check load slip.
- **Lean execution (user,2026-11-07):** about90 minutes per step. Model checks only: integrity,
  derived fasteners, collisions, service paths and hobber suite. No new FE scripts or cases
  unless stiffness load paths change; then one run. No new library components/fields unless
  named by the step. Commit what works at the timebox and report remaining work.
- **XC is three commits:** edge check; X ground plate and ribs; column sole pads. Keep the
  passing whole-machine sweep; rerun only hobber after trimming. Existing hobbing,
  joint-fasteners and assembly-integrity passes remain the gates. No FE service framework.
- **Edge check:** made clearance1.5d; steel tapped d plus0.5d wall beyond major radius;
  counterbores fully on the face. Bought own holes excluded by kind. Enlarge made parts or
  report findings; no certified-edge exceptions. See BOM_PLAN F1. Report other machines in
  one Log line; do not alter their hardware.
- **X carriage:** ground20 mm deck, bright-bar ribs bolted from below and dowelled; bond may
  be used but gets no stiffness credit. Keep the small SawnBar manufacturing route. The X
  rail height is unchanged. Stock flatness0.440 mm ASSUMED exceeds HIWIN S1 mounting
  allowance0.0748 mm and matched class-C height variation0.020 mm. Replicate all four block
  pads against one master set from the installed blocks to0.010 mm. Nominal layer0.8 mm,
  worst-case0.23..1.37 mm versus ASSUMED compound range0.2..2.0 mm. Preload/accuracy remain
  procurement notes, not LinearRailBlock recipe fields.
- **Rib faying process:** blast-clean both bare steel lands, mask precision block-pad
  datums, and keep paint, oil and adhesive off the contact. RCSC 2020 sections5.4/8.4
  ([source](https://www.boltcouncil.org/files/2020RCSCSpecification.pdf)) gives0.50 for its
  Class B surfaces. This model uses0.40 as an ASSUMED acceptance floor for our small
  metric joints, pending a shop slip coupon; it is not a vendor rating or a claim of RCSC
  qualification. Check normal AND crash factors>=3 without bond/pin credit. Retain the
  six-fixing pattern and its original FE geometry; the seventh-fixing draft is deleted.
- **X stiffness:** one script,fem/carriage_fem.py, and one current result,fem/results/carriage.json.
  Bolts-only43.633 N/um front tie and17.077 MN m/rad sole tilt, C3D10 size10 mm, pads held z.
  Use17.077 in the accuracy budget. LM0 authoritative reference20.232/10.345; fresh unchanged
  STEP19.529/9.672. Plate approximation28.90/11.75 is diagnostic; old263.9 tilt was invalid.
- **Column sole:** four ground C45 pads span380x180 mm; front pair clear the casting recess
  and actuator path. Pads are80 mm deep, widths65/60/90/90 mm, with two M8 each. Reserve
  the same joint-declared source drilling before cast-anchor sizing. Catalogue stock first;
  inspect after cure. A6 alignment and assembly match-pins belong to XA. The former full
  rectangular sole collided with the actuator and is deleted. No dowel credit in sole slip.
- **Order:** LM0,XC edge/carriage/sole,XA,LM1,LM2,LM3,LM4,LM5,ZB,LM6. Merge main after XC
  and before XA (authorized); preserve submodule pointers. Main carries electrical-rack,
  readable-name/foundation-group changes and ALIGNMENT.md; follow its datums/methods.
- **XA:** keep the X motor inline and low. Declare alignment per precision joint: reference
  edge / indicate then match-pin / shim or adjust / drawing dowels / fixture-replicated;
  reject a missing declaration. Indicate the nut block at both travel ends, then match-pin.
  Check assembly order and tools: bed/inserts, rails/blocks, supports, screw/nut, carriage,
  nut block, scale/head, guards/skirt, tray. Every fastener needs clear access at its step.
- **XA drive seats:** delete X/Z ground drive pads and nutGroove. One fixture-replicated seat
  per support on cast inserts; an arbor and carriage-nut-bore height reference hold supports
  coaxial and parallel to rails during cure. Derive layer thickness/tolerance and check an
  ASSUMED product range until chosen. Bolt and match-pin for service return. Move guard feet
  off precision seats. Nut travel/overtravel must clear the open channel. Update RAIL_MOUNTING.
- **XA motor bay:** tray ends at one straight edge ahead of motor; one removable lid exposes
  motor/mount/coupling. Remove rear bridge, its seals and two casting pockets. Keep rearward
  screw withdrawal. Remove/reposition clamps10/11, deriving seats from named cover/clamp widths;
  clamps and bolts clear every machine part except the bolt's own threaded seat (M6 major
  thread/tap-drill representation51.84 mm3 over6 mm). All tray blanks/lid clear removal paths.
  Pass enclosure end to end after merging main. Head/wipers accessible with guards removed;
  blind/captive fasteners need a reason. Record motor heat and cooling only if budget requires.
- **LM1:** a library ScrewLinearModule, own test project, two or three sizes. Spec: rail family,
  spacing, blocks/spacing, stroke/overtravel, carriage and host pattern, SupportedScrewDrive,
  own/host ground base plate, derived ground riser bars. HZMotion mount/BF sit between rails.
  Derived fasteners/pins, hard stops before support contact, alignment, BOM and removals checked.
- **LM2:** integrate sealed scale, smallest standard ML for stroke+2overtravel. Seat referenced
  to rail edges; head adjustment covers JFT-40 housing parallelism0.1, head parallelism0.05,
  lateral0.2 and gap1+/-0.3 mm. Check reading window, Abbe,75 mm cable bend, air access,
  dual-loop scale-position/motor-velocity feedback ports and independent scale service.
- **Module scale:** JFT-40 beside the guides, ground40x20 seat derived from M8 engagement,
  separate fitted head carrier; head slides out along travel and housing sideways after it,
  leaving the carriage. Short modules return the R75 cable toward the guides beyond their end; long modules keep
  the normal bend so the cable never crosses the longer housing during travel. Fit head normal face within
  +/-0.5 mm; head/attachment clearances give +/-0.55 gap adjustment. Stack0.24/0.22 mm is
  ASSUMED. Base/riser/block stock screen0.45/0.55/0.76 mm already fails0.01; inspect all
  stock faces (including carriage), then replicate failing faces. Qualification remains open.
- **LM3:** cover chosen from stroke: short-stroke wipers/fixed top or telescoping, longer bellows.
  Keep chip ingress, travel, fasteners and removal checks in the module test project.
- **Module cover:** up to20 mm stroke, split formed-sheet guards and separate clip-in PU
  wipers leave a62x192 mm default payload window,10 mm overlap through overtravel and6 mm
  roof gap. The short cable exits the open end below the roof, with derived corner relief
  for2 mm running clearance through overtravel; the tall hood is deleted. Short covers
  withdraw longitudinally: remove the head first at the front, motor/shaft first at the
  rear. A containing machine may supply the carriage interface. Longer strokes use the existing lamella
  bellows library with through-tapped flange plates and side-clamped feet. The full R75
  cable bend/tail drives the large envelope:396.8x186.25 /424.2x179.95 mm for80/200 stroke.
  Geometry masses about1.273/8.775/8.018 kg, rigid moving guards0/4.135/3.710 kg; bought cover
  density/geometry, clip retention and end sealing remain ASSUMED/unqualified. No stiffness credit.
- **LM4:** use the module for hobber Y, screw centred, ground base and derived rail risers.
  Delete L brackets, old nut seat and scale seat bars. Apply XA alignment/assembly/service
  treatment. Preserve swivel/head interfaces and the dual-loop feedback wiring.
- **Y host guard follow-up:** seal on the ground carriage back face, with the roof
  derived below it from the nut/head stack and raised end hoods over the fixed supports.
  The rail-top-derived lower profile includes Y/Z overtravel and A tilt; the rear lower
  M8 head fixing moves inward one diameter to preserve the made clearance edge land.
  Stock/PU retention and fixed-port sealing remain ASSUMED. The longitudinal rear
  withdrawal still traps the retainer at the fixed mount; redesign the split/service
  route rather than making precision-support removal a guard-service prerequisite.
  Stop unbolting access beneath the end hood is also an OPEN service gate.
  The deck keepout must cover its complete vertical passage, not only its lowest-Z snapshot.
- **LM5:** verify hobber model gates; report before/after Y/head mass/CoM, A moment/torque,
  accuracy, Abbe, clearances and scale tolerance stack. Supplier-unknown values stay ASSUMED.
- **ZB:** way-cover-library bellows on Z, no structural redesign; check full travel and removal.
  Four rail bellows use an ASSUMED custom thin PU RFQ envelope:210 mm span including
  overtravel,30 mm compressed space,H24 cropped steel end flanges. No catalogue/vendor
  rating is implied. Keep nested-cloth clearances internal to each purchase only after
  actual-solid zero-overlap checks at both overtravel limits and zero. Bench-fit the
  moving brackets before lowering the saddle; park Z high for lower flange screws and
  low for upper screws. For service support released ends on temporary straps, remove
  the saddle first, then withdraw whole covers16 mm forward,100 mm outward,500 mm forward.
  Offset-screw ingress and complete service/tool corridors remain release gates.
- **LM6:** update FRAME_PLAN,PARTS,MACHINE_DESIGN and regenerate/check machined supplier package.
  Assembly tools get separate BOM rows, ASSUMED prices until quoted, and checked sequence refs:
  X/Z arbors/height references, replication epoxy/release/dams, C test mandrel, square/granite,
  level/straightedge/ground spindle arbor, indicators/bases, AF4 key>=400 mm and added fixtures.
- UI capture is unavailable in this session; do not access the user's protected editor worktree.

## Log

- 2026-11-07 LM0 (`fedf4ff0c`): before edits,hobber2345/hobbing1161; joint-fasteners,assembly-integrity,way-cover PASS; requested frames0 errors,13 services,269 members,68 joints246 screws26 pins. X carriage24.586824 kg; Y28.132885 kg; head39.16 kg; A tilt48.39 Nm, max gravity35.10 versus87 rated204 peak (brake87 ASSUMED). FE baseline/Abbe/budget details: examples/flexspline-hobber/LINEAR_MODULE_BASELINE.json.
- 2026-11-07 XC edge complete: plain made-only rule; bought own holes excluded. Bench unchanged232 screws16 pins,60 findings (saddle22,table12,X/Z nut brackets6 each,motor plates4 each,wall2); other examples have no derived joints. Post-trim hobber2453 PASS;50 existing hobber findings elsewhere, none on the new carriage/sole joints; no material exceptions.
- 2026-11-07 XC carriage complete:20 mm ground deck +bright-bar ribs,26.242 kg versus24.587; bolts-only43.633 N/um,17.077 MN m/rad; accuracy PASS29 s, predicted class4/target6. Post-trim hobber2453 PASS; retained pre-trim hobbing1194,joint-fasteners,assembly-integrity,way-cover and frames0 errors1728 s/13 services gates as directed. Single FE result; service framework and unrelated head changes removed.
- 2026-11-07 XC sole complete:four80 mm deep ground-stock corner pads,8 M8; contact-only tilt screen2.225x before, per-pad whole-shear slip38.1 normal/7.2 limited crash. Source anchor demand includes joint service threads; central actuator path stays open. Trimmed hobber2453 PASS; split commits689a2fbb4 (edge),f28c59f45 (carriage),25f7ca127 (sole),3.696 kg pads.
- 2026-11-07 XA partial (lean timebox): bridge,6 seals,2 pockets and2 clamp/socket pairs removed;10 clamps,70 mm cover clearance; lid3/67.715/100 mm path at X45.215.21 joint methods,9 steps,6 tool corridors PASS (AF3/AF5.5 profiles ASSUMED); rib friction screen4 PASS, minimum1.545 normal/1.368 crash (mu0.15,2x sharing ASSUMED,no bond/pin credit). Hobbing1186,joint-fasteners,assembly-integrity,enclosure48 states/66 pairs/30 lid samples PASS; hobber2633 and focused31 checks PASS, including adjacent fasteners in the offset-spanner corridor. X/Z replicated seats,guard-foot relocation,A6 pins and complete Z/wiper service treatment remain open; motor40 W ASSUMED, no cooling part until the 1 K budget is measured.
- 2026-11-07 LM1 nominal module: own/host base, derived ground risers, M60D-BK12/BF12/SFU1605, four guide blocks and joint-derived stops; HG15 strokes20/80 and HG25 stroke200 PASS60 checks including strict integrity, three service paths, end-stop contact, slip, roundtrip and alignment for every joint. Screws170/240/370 mm, rails220/300/440, risers25x25/25x25/25x15;58/62/70 screws and18 pins. Hobber2633,hobbing1186,joint-fasteners and assembly-integrity PASS. Stock flatness0.10/0.05 mm ASSUMED versus0.01 acceptance remains unqualified; host pattern/torque-limit qualification and physical corrective replication remain open. No FE added.
- 2026-11-07 LM2 model gates: JFT-40 ML70/120/220,208/258/358 mm housings on ground40x20 seats;66/70/78 screws,18 pins. Own102 checks PASS (physical made edges, five services, key/air corridors, overtravel window, roundtrip, independent position/native-motor feedback); hobber2633 PASS. Head normal/gap stacks0.24/0.22 versus+/-0.5/0.55 mm settings, R75 cable, key D3.5x8/90 and air socket D14x35 ASSUMED. Abbe from module carriage interface111.2/36 mm HG15 and133.2/38 HG25; minimum slip4.482 normal/1.494 momentary, scale89.411/44.706 at ASSUMED20/40 N. Base/riser/block stock screens0.45/0.55/0.76 versus0.01 mm fail; inspect/replicate and qualify fitted head face before release. Fixed M6/pin annulus by66 mm bolt spacing; no FE. 2026-11-08 short cable return toward +module Y passes286 library assertions and actual-solid head-against-machine probes at three low-Z tilt states (zero overlaps), removing the former721.23 mm3 cable/X-carriage clash.
- 2026-11-07 LM3 model gates: short split cover/retainers/PU lips plus bolted cable hood; long80/200 use two/four-fold lamella bellows with22/38 mm compressed,30/50 space,120/260 extended. Own276 assertions PASS across all three sizes (integrity, physical made edges, slip, contact, access, service, seal contact/overtravel, part refs, roundtrip/feedback); hobber2633 PASS.86/122/130 screws,18 pins; minimum joint factors4.482/1.494. Fixed/moving flange contacts nominal672/736 mm2 versus625; short payload62x192,overlap10,roof gap6 mm. Guard masses1.881/8.775/8.018 kg (rigid moving0/4.135/3.710), cover geometry/density ASSUMED; clip/end-seal/vendor qualification open. No precision stiffness path changed, no FE.

- 2026-11-08 LM4 fit follow-up (main merge de3795a23; earlier e8794ed99): underside roof58 mm / carriage back69, end hoods91; ground base9.319693 kg, carriage7.309937. All6 full wiper-footprint probes and fastener fit0 errors PASS; actual-solid Y-part probes against all machine parts at three low-Z tilt states report0 overlaps. Enclosure48 states/112 exact pairs PASS (before the final11.18 mm3 internal mount relief). Guard seam holes moved off the split; guards/retainers bench order declared. Whole hobber FAIL7 service-only findings: scale-head/stop17.529 mm3, rear cover/retainer/lip against fixed mount60/83.826/41.182 mm3, and three screw-withdrawal/guard obstructions. Nominal solid fit0; full requested sweep FAIL10 in2185 s (7 service plus base/deck42.118 mm3 and two guard/deck5.265 mm3 overlaps at Z-10);96 joints345 screws46 pins; do not disturb precision supports to service guards. 2026-11-08 deck keepout now sweeps the full210 mm relative Z passage; focused six wiper checks and nine Z/A deck-pair poses PASS with0 overlaps (previous42.118/5.265/5.265 mm3). Full sweep not rerun. Screw removal now requires guards off; host scale-head removal requires the front-left stop off. Focused actual-solid screw/head service audit PASS0 errors (four former findings resolved); three-size module regression PASS289 assertions; three rear guard/mount findings, stop tool access, final service split and valid Y FE remain OPEN.

- 2026-11-08 LM5 model numbers after LM4 fit follow-up: X/Y/Z moving203.73/30.88/65.15 kg versus191.48/28.13/51.42; all drive sizing PASS, Z RMS0.822/1.27 rated N m, thrust reserve278 N. Rotating head52.54 versus39.16 kg; COM(-74.9,33.0,81.3) mm from output, bearing demand51.01 Nm UNKNOWN rating, A gravity27.29 Nm versus87 rated/204 peak (brake87 ASSUMED,3.2x). Y Abbe87.8 versus76.2 mm; scale stacks0.24/0.22 versus+/-0.5/0.55 after accepted/replicated stock; raw bound0.91 versus0.01 fails. Six-fixing rib factors about3.9 normal/3.5 crash at ASSUMED coupon-qualified mu0.40; no bond/pin credit. Current library286/hobbing1185 PASS; retained core suites PASS. Current Y physical fit improves; service/global gates and valid Y stiffness/accuracy remain OPEN. No accepted new FE result.

- 2026-11-08 ZB partial (20afedf1f,9f7ca8351,b8fbbeb7c):4 rail bellows/6 made mounts replace2 Z sheets;98 joints346 screws46 pins,210 mm span/30 mm compressed/H24 flanges (thin PU RFQ ASSUMED). X/Y/Z moving202.23/29.80/63.63 kg, drives PASS; guard slip>=169.7 under ASSUMED10 N handling. All32 flange tool corridors and1260 internal pair/state probes PASS; module286,hobbing1185,joint-fasteners (7 machining-identity checks),assembly-integrity,way-cover PASS. Physical machining BOM keys fix repeat-bracket aggregation; recipe/geometry keys stay exact. Nominal strict integrity0 errors; full hobber3234 PASS after correcting stale Y joint/coupling test IDs. Revised17.2 mm opening/cropped flange lands retain complete head bearing;24 travel/overtravel poses now0 ZB errors (was8),46 remaining whole-machine Y findings. Enclosure reaches Y-base/tray-clamp clash827.497 mm3; direct socket-head bearing on3 mm steel eliminates four washer/skin clashes (0.311 mm3 each); final fastener fit0 errors and32 driver corridors PASS. 29 other hobber made-part edge findings remain reported/open (Z saddle2, cradle7, X/Z nut receivers12, X guards8). Y corner fit and offset-screw ingress remain OPEN. No stiffness path changed or FE run.

- 2026-11-08 LM6 model/RFQ checkpoint:27 separate tooling BOM rows (EUR2166 total ASSUMED planning allowance),89 checked sequence references; X/Y/Z plus datum/lifting procedures validate. Routed inventory replaces stale hand list;46 routed designs/50 made instances/90.05 kg regenerate after the Y fit follow-up with all package checks PASS, including SawnBar blanks and profiled ground-stock allowances. Module286,joint-fasteners,assembly-integrity,way-cover PASS; retained hobbing1185 PASS. Hobber3325 PASS before the Y host-guard follow-up; current hobber FAIL7 service-only findings, fastener fit0 and enclosure48/112 PASS. XA replicated X/Z seats/A6,29 made-edge findings,Y corner fit/valid FE accuracy and Z screw ingress remain OPEN; UI capture unavailable.
