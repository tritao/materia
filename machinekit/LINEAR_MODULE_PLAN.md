# Precision screw linear module

## Decisions

- **Structure:** castings provide mass; precision belongs in bought ground plate and flat bar.
  Bars supply only derived stack height. Locate rails at reference edges, never dowels;
  indicate screw supports then match-pin; indicate nut blocks at both travel ends then
  match-pin; adjust scales at the head; structural assembly dowels follow drawings.
  Joints declare reference edge / indicate then match-pin / shim or adjust / drawing dowels.
  Replication is reserved for large casting surfaces that need it. Inspect/select bought
  ground stock, then supplier-finish failing small faces. No small epoxy support layers.
- No welding, compatibility alternatives or parts merged just for count. Bolts, holes and
  dowels come from BoltedFace/PinnedFace and JointFasteners. Check load slip and fittedBefore.
  Castings, structural members and deliberate adjustment interfaces remain separate parts.
- **Wrap-up scope (user,2026-11-08):** replace the replicated drive-seat work with simple
  ground bars, freeze the module at the hobber Y use and already-tested configurations,
  run LM5, finish LM6 and record unfinished LM4/ZB as OPEN. No further generalization,
  long-stroke host mounts, new host modes, service FE batches or new qualification work.
  This closes the implementation goal with the open qualifications below, not a machine release.
- **Execution/order:** LM0, XC(edge/carriage/sole), XA, LM1, LM2, LM3, LM4, LM5, ZB, LM6.
  Main was merged before XA and LM4. Preserve submodule pointers. Lean model checks and
  one FE case only when the stiffness path changes; no added FE framework. Commit each step.
- **Made-hole edges:** clearance holes in load plates>=1.5d; steel tapped holes>=d and
  >=0.5d wall beyond the thread major radius; counterbores fully on face. Bought own holes
  excluded by part kind. Enlarge made receivers or report findings, no certified exceptions.
  Sources/assumptions are in BOM_PLAN F1. Other machines are report-only.
- **XC:** ground20 mm X deck, four bolted bright-bar ribs and small SawnBar route; no bond
  stiffness credit. Four ground sole pads span380x180 mm,80 mm deep, widths65/60/90/90,
  two M8 each; front pads clear recess/actuator. Column joint contact tilt screen2.225x
  before; no dowel credit in slip. A6 squareness/pins remain OPEN. One existing
  fem/carriage_fem.py and one fem/results/carriage.json; FE held-guide boundary conditions
  govern the accuracy budget. LM0 authoritative20.232 N/um and10.345 MN m/rad; the old
 263.9 MN m/rad plate-model value was invalid. Current FE45.201 N/um and24.616 MN m/rad (direct ground guide contacts) governs
  the accuracy budget; residuals6.14e-11/7.49e-11. The previous43.633/17.077 result
  included small replica films and is replaced, not retained as an alternative.
- **XC stock/faying process:** stock flatness0.440 mm ASSUMED exceeds HIWIN S1 allowance
 0.0748 and class-C height variation0.020 mm ASSUMED. Select accepted stock or finish-grind four
  block lands in one setup to0.010 mm; no0.8 mm replica pads. Blast-clean rib lands,
  no paint/oil/adhesive in contact; friction0.40 ASSUMED pending shop coupon. RCSC2020
  sections5.4/8.4 gives0.50 for its Class B surfaces
  ([source](https://www.boltcouncil.org/files/2020RCSCSpecification.pdf)); that is not a
  rating for these metric joints. Check normal/crash>=3 without bond/pin strength credit.
- **XA ground drive bars:** two short bars per X/Z, one HZMotion and one BF12, matched
  thickness X15/Z40 mm derived from existing insert/support planes. Finished70x75 and
 65x35 mm footprints are sawn from80 mm bought ground parent stock. Inspect faces and
  match each pair within0.01 mm. Support screws/pins engage bars, bar clamps engage
  inserts. Indicate to rails and actual screw/nut at both travel ends then pin for service
  return. Delete continuous pads, nutGroove, spacers, drive epoxy, alignment arbors,
  height-reference fixtures, special reaming kit and compound-friction screen/tool rows.
  Nut traverses open channel through overtravel. Guard feet stay off bars on bed inserts;
  four AF2.5 D3.5x100 mm corridors are ASSUMED tool envelopes. Covered bar clamps tighten
  before supports; rare bar replacement deliberately removes supports, motor service does not.
- **XA assembly/service:** keep motor inline and low. Bed/inserts, ground bars, rails to
  edges, blocks, supports, screw/nut, carriage, indicated/pinned nut block, scale/head,
  guards/skirt, tray. Declared methods/tools and tightening corridors are checked.
  Tray ends at a straight edge before motor, one removable bay lid; rear bridge, seals,
  two casting pockets and clamps10/11 removed. Named cover70 mm half-width/clamp width
  drive seat placement. Keep rear screw withdrawal and tray/lid paths clear. Motor40 W
  heat ASSUMED,1 K loop allocation; measure before adding cooling plate or isolation spacer.
- **LM1–LM3 library:** ScrewLinearModule owns ground base/derived risers, centred
  M60D-BK12/BF12/SFU1605, carriage/stops, scale and covers. Retain tested HG15 strokes20/80
  and HG25 stroke200 only; no added hosts/modes. Screws170/240/370,rails220/300/440 mm.
  JFT-40 ML70/120/220,40x20 ground seats and fitted head carriers; normal/gap stacks
 0.24/0.22 versus+/-0.5/0.55 mm adjustment are ASSUMED. R75 cable, air/key access and
  dual-loop feedback remain checked. Raw stock/block bounds0.45/0.55/0.76 exceed0.01 mm:
  inspect/select/finish small faces. Short split guards/clip-in PU lips have62x192 payload
  window,10 overlap/6 roof gap; existing longer lamella examples remain tested, frozen.
- **LM4 hobber Y:** profiled ground base/carriage directly on actuator; derived rail
  risers, centred screw and integrated sealed scale. Delete L brackets, nut/scale seat
  bars, swivel plate and adapter. Upper/lower guards attach directly to base. At Xretract,
  Zhigh,Y=A0: upper135 mm up; lower202 down after clearing mandrel, then108 forward.
  Remove guards before stops, front-left stop before head. Precision supports/X skirt stay.
  Rigid hardware sweeps do not qualify elastic PU peel/retention or fixed-port seals.
- **LM5:** report LM0/current masses/CoM, A moment/torque versus eRob, conditional accuracy
  rows/Abbe, model integrity/fasteners/collisions/service, member and BOM counts. Supplier
  facts without public data remain ASSUMED. No valid new Y stiffness is credited.
- **ZB:** four library rail bellows/six made mounts replace two Z sheets. Custom thin PU
  RFQ210 mm span including overtravel,30 compressed,H24 cropped flanges is ASSUMED.
  Bench-fit brackets; service uses support straps, saddle removed,16 forward/100 outward/
 500 forward cover withdrawal. Retain tested geometry; remaining service/ingress is OPEN.
- **LM6:** keep FRAME_PLAN, MACHINE_DESIGN, PARTS and ALIGNMENT consistent; regenerate and
  check the machined RFQ package. Tooling is a separate BOM category with ASSUMED EUR
  prices and checked step references: C mandrel, spindle test arbor, square/straightedge,
  level, indicators/bases, AF4>=400 mm, declared hand tools/hoist/slings/support straps.
- **OPEN at wrap-up:** final extended sweep has one running-gap finding: eRob
  housing/ZdriveFloatingBar0.8 mm versus2 mm required at X45.215,Y-10,Z-10,A-15deg;
  correct that gap and rerun the full sweep/service phase (not reached after the finding).
  Also valid Y stiffness and resulting accuracy release (prior Y solve
  rejected); bought-ground-stock/finished-land inspection; rib friction coupon; A6 column
  squareness and assembly pins; PU wiper peel/retention/elastic release and port/end seals;
  residual made-edge findings outside new carriage/sole/Y/bar joints; complete Z/wiper
  service/tool treatment and offset-screw chip ingress; thin Z bellows supplier acceptance;
  eRob bearing moment rating (unknown) and brake87 Nm (ASSUMED); motor thermal commissioning
  and conditional cooling; casting-package refresh/large-face qualification; other unpublished supplier tolerances. These are recorded,
  not pursued in this timeboxed close-out. UI capture unavailable; use model tree evidence.

## v2: module as one body (2026-11-09)

**Why.** The first module reads as a kit of separate parts on a flat plate, and fitted to the hobber's Y axis (+-15 mm) it was
sized like a long-stroke module: base 780 x 450, carriage 600 x 350, screw 410 mm, guards 782 mm wide, four U stops, seal lands,
retainers and PU lips. The swivel head went from 39.2 to 52.7 kg on the eRob and the Y Abbe offset from 76 to 88 mm (LM5).

**Reference (user, 2026-11-09).** An HZM200-style open module: a wide base plate with clamp flanges along its sides; two tall
full-length side walls with the guide rails on top; an open channel between them with the ball screw centred on the floor; the
motor end block (coupling window) at one end and the floating support at the other; a plain carriage plate spanning both rails.
Optional flat top covers from the ends to the carriage.

**Parts, all from the Structure rule (ground flat stock, location by part type).**
- Base: ground plate; clamp or bolt flanges along both sides to the host.
- Walls: two full-length ground bars, bolted and dowelled to the base; their tops carry the rails (reference edge on the inner
  side); they replace the riser bars and form the body's sides.
- Drive: HZMotion M60D-BK12 as the motor end block and BF12 at the other end, both on the base floor in the channel, screw
  centred, indicated then pinned; hard stops on the floor at the channel ends (no separate U stops).
- Carriage: plain ground plate spanning both rails, tapped host pattern; no seal lands.
- Cover (optional): two flat top sheets from the ends to the carriage, overlapped by it, plus the block wipers. No retainers, no
  PU lips.
- Scale: housing on the outside face of one wall (ground, so parallel to the rail on top by construction); head on a small
  bracket on the carriage; cable out at the motor end.

**Hobber Y at compact proportions.** Carriage only as large as the spindle seat needs; body length = carriage + stroke +
overtravel + the two supports. Hard gates: head (A rotating) mass <= 39.2 kg; no part wider than the old swivel plate; hobber,
hobbing, module, joint-fastener, assembly-integrity and enclosure suites pass; the Z floating bar / eRob clearance >= 2 mm.

**Implementation choices (v2-a).** Keep both displayed variants open: omit the optional sheets rather
than add supports or seals. Walls are full-length ground steel bars; their height follows the bought drive/nut/guide stack,
and their width also clears the ASSUMED 8 mm AF3 key leg at the outside scale. Two plain rectangular floor stops contact the
nut carrier at overtravel, clear of the centred screw. The scale head/bracket leads the carriage datum by 20 mm toward the
motor (layout allowance, not a supplier dimension). The hobber adapter is one profiled ground-steel base: continuous floor,
wall lands and 16 mm side clamp flanges, with an integral actuator mounting lobe. Its plain carriage uses ground aluminium
plate and washers at the head interfaces; no steel-thread engagement is credited in aluminium. The high, wide host placement
is an intermediate fit for v2-a; compact proportions and all three hard gates remain v2-b work. Stock/finished-face acceptance
and Y stiffness remain unqualified; no FE is added. LM0's swivel plate is 230 mm along work Y and 190 mm vertically; record
actual part dimensions when applying the width gate.

**Compact refit (v2-b).** Independent nut, guide and scale stations remove the empty end bays. The hobber uses
30 mm nominal travel (+/-15) plus 5 mm overtravel each end, a 180 mm screw, 220 mm rails, 140 mm rail spacing and 64 mm block
spacing. The 230 x 310.5 x 12 mm profiled steel base keeps a continuous floor and an integral actuator lobe; the plain
140 x 290 x 10 mm aluminium carriage spans the rails and reaches down to the head. Dimensions are travel x across-guides x
thickness. Both walls are 230 x 32 x 30 mm. The 230 mm rigid-part width gate is along work Y (LM0 plate 230 x 190 mm); the
310.5 mm transverse base extent and the scale's 251 mm flexible hose-routing envelope are reported separately. No cable
length is credited as rigid plate width. The scale housing is centred on the guide span; its head leads the carriage datum
by 52.5 mm. Its single bracket bolts outside the lower guide block, behind the spindle flange, and is bench-fitted before
the spindle seat/cartridge. The low-profile AF3 key's exposed 5.5 mm leg, D3.5 and 90 mm reach are ASSUMED tool dimensions.
The spindle seat's four M8 now use Y95/115 rows; the existing outboard support has two integral arm extensions to its Y30
carriage fixing, without a separate adapter. Two d4 drawing pins locate each head interface; clamped bolts carry head loads.
The base's M6 receivers are through tapped in 12 mm stock. Floor stops retain two M6 and two d4 pins each; their pins carry
the declared crash load. The existing Z floating bar is relieved by the eRob envelope swept continuously through Z
-15..195 mm with 2 mm clearance; its fastener support is checked with Y parts. Optional sheets remain omitted. Stock/face
acceptance, the selected key, the new Y/outboard stiffness and supplier/brake ratings remain unqualified; no FE is added.

**User follow-up (2026-11-10): flat floor and narrower rail walls.** Replace the nut-carrier floor relief with two
matched 2 mm packing pads under the HZMotion and BF12 supports. Finish the pads from 3 mm ground stock; the existing
support screws and match-pins pass through them. Pad widths retain the actual 1.5d clearance-hole lands. The BF12 pad extends
toward the free end for its bearing seat, away from nut travel. Packed M60D match-pins use installer-selected sites
at x=+/-8, z=70 mm in the solid rear block, below the bearing bore, with 6 mm blind bores in the foot. Packed BF12 pin
sites move farther from the bolt lands and use 6 mm blind bores too. These are ASSUMED installation details, checked
against the supplier solids; unpadded axes keep their prior sites. Exclude packing from the separate support pin-bearing
strength check rather than crediting it as bought-foot engagement. Stand 32 x 25 mm ground bar on edge for both full-length
walls: 25 mm across the rails and 32 mm high, versus 32 x 30 mm before. This raises the screw and guides together by2 mm
and leaves3 mm below the carrier on the plain12 mm floor. The hobber's module pose preserves the working component datums;
the integral actuator interface uses shorter M4 screws to retain the original5 mm engagement and a full-depth head recess.
Both25 mm walls require two7 mm scale-housing spacers for the existing ASSUMED AF3 key. These are two instances of one
40 x30 x7 mm ground-stock part, finished from8 mm stock, clamped by the existing housing screws. Four small pieces are
added in total; no extra brackets, stop parts or independent spacer screws. The two travel stops remain. Separate packing
is included in clamp grip and persists with the supports/housing; it is excluded from bought-part identity and never
credited as thread in the base/wall. The reading-head carrier clears the housing by2.2 mm. Allow an additional0.02 mm
ASSUMED finished-packing contribution in both scale-setting stacks. Both displayed variants now derive internal module
machining and fastener BOM lines before generating their scene. Stock, finishing and stiffness remain unqualified; no FE.

**Steps** (lean: model checks only, no new FE, no new library features beyond this list, about 1.5 h each, one commit each):
- **v2-a.** Rebuild `ScrewLinearModule` to this layout; delete the riser bars, U stops, seal lands, retainers and lips.
  Module checks for the standard and the hobber sizes.
- **v2-b.** Refit the hobber's Y axis at compact size; fix the Z floating bar clearance.
- **v2-c.** Compare with LM0 (head mass and CoM, A tilt and gravity torque, Y Abbe, member and BOM counts) in one Log line;
  update `machinekit/examples/linear-module` so both variants show the result.

## Log

- 2026-11-07 LM0 (`fedf4ff0c`): before edits,hobber2345/hobbing1161; joint-fasteners,assembly-integrity,way-cover PASS; requested frames0 errors,13 services,269 members,68 joints246 screws26 pins. X carriage24.586824 kg; Y28.132885 kg; head39.16 kg; A tilt48.39 Nm, max gravity35.10 versus87 rated204 peak (brake87 ASSUMED). FE baseline/Abbe/budget details: examples/flexspline-hobber/LINEAR_MODULE_BASELINE.json.
- 2026-11-07 XC edge complete: plain made-only rule; bought own holes excluded. Bench unchanged232 screws16 pins,60 findings, unchanged and report-only; other examples have no derived joints. Post-trim hobber2453 PASS;50 existing hobber findings elsewhere, none on the new carriage/sole joints; no material exceptions.
- 2026-11-07 XC carriage complete:20 mm ground deck +bright-bar ribs,26.242 kg versus24.587; bolts-only43.633 N/um,17.077 MN m/rad; accuracy PASS29 s, predicted class4/target6. Post-trim hobber2453 PASS; retained pre-trim hobbing1194,joint-fasteners,assembly-integrity,way-cover and frames0 errors1728 s/13 services gates as directed. Single FE result; service framework and unrelated head changes removed.
- 2026-11-07 XC sole complete:four80 mm deep ground-stock corner pads,8 M8; contact-only tilt screen2.225x before, per-pad whole-shear slip38.1 normal/7.2 limited crash. Source anchor demand includes joint service threads; central actuator path stays open. Trimmed hobber2453 PASS; split commits689a2fbb4 (edge),f28c59f45 (carriage),25f7ca127 (sole),3.696 kg pads.
- 2026-11-08 XA wrap-up (06d5bf157;5264c687d): motor bay/lid, straight tray edge, guard insert feet and ten checked assembly steps retained; continuous X/Z pads, nutGroove, drive replication/spacers/arbors/height fixture/reaming kit and layer-aware fastener derivation removed. Four matched ground bars X15/Z40 mm, fixed70x75/BF65x35 from80 mm stock; no channel groove. Support height unchanged; fixed-bar slip X7.4/3.8 and Z3.4/1.3 normal/crash, default steel-joint mu0.15 (ASSUMED). Four AF2.5 guard-foot corridors; A6 and complete Z/wiper service remain OPEN. Hobber3361,hobbing1185,module289,joint-fasteners/core integrity/way-cover,enclosure48/128 and Z-drive42 PASS. Direct-ground carriage FE45.201/24.616 replaces film result43.633/17.077; remaining LM4/ZB qualifications stay OPEN.
- 2026-11-07 LM1 (7fb0f9938) nominal module: own/host base, derived ground risers, M60D-BK12/BF12/SFU1605, four guide blocks and joint-derived stops; HG15 strokes20/80 and HG25 stroke200 PASS60 checks including strict integrity, three service paths, end-stop contact, slip, roundtrip and alignment for every joint. Screws170/240/370 mm, rails220/300/440, risers25x25/25x25/25x15;58/62/70 screws and18 pins. Hobber2633,hobbing1186,joint-fasteners and assembly-integrity PASS. Stock flatness0.10/0.05 mm ASSUMED versus0.01 acceptance remains unqualified; host pattern/torque-limit qualification and supplier face finishing remains open. No FE added.
- 2026-11-07 LM2 (19f8bb6ad) model gates: JFT-40 ML70/120/220,208/258/358 mm housings on ground40x20 seats;66/70/78 screws,18 pins. Own102 checks PASS (physical made edges, five services, key/air corridors, overtravel window, roundtrip, independent position/native-motor feedback); hobber2633 PASS. Head normal/gap stacks0.24/0.22 versus+/-0.5/0.55 mm settings, R75 cable, key D3.5x8/90 and air socket D14x35 ASSUMED. Abbe from module carriage interface111.2/36 mm HG15 and133.2/38 HG25; minimum slip4.482 normal/1.494 momentary, scale89.411/44.706 at ASSUMED20/40 N. Base/riser/block stock screens0.45/0.55/0.76 versus0.01 mm fail; inspect/select/finish and qualify fitted head face before release. Fixed M6/pin annulus by66 mm bolt spacing; no FE. 2026-11-08 short cable return toward +module Y passes286 library assertions and actual-solid head-against-machine probes at three low-Z tilt states (zero overlaps), removing the former721.23 mm3 cable/X-carriage clash.
- 2026-11-07 LM3 (16e5da031) model gates: short split cover/retainers/PU lips plus bolted cable hood; long80/200 use two/four-fold lamella bellows with22/38 mm compressed,30/50 space,120/260 extended. Own276 assertions PASS across all three sizes (integrity, physical made edges, slip, contact, access, service, seal contact/overtravel, part refs, roundtrip/feedback); hobber2633 PASS.86/122/130 screws,18 pins; minimum joint factors4.482/1.494. Fixed/moving flange contacts nominal672/736 mm2 versus625; short payload62x192,overlap10,roof gap6 mm. Guard masses1.881/8.775/8.018 kg (rigid moving0/4.135/3.710), cover geometry/density ASSUMED; clip/end-seal/vendor qualification open. No precision stiffness path changed, no FE.
- 2026-11-08 LM4 (f3005413a,e8794ed99; service5beed7560) service follow-up: upper/lower host guards, roof58/carriage back69/end hoods91 mm, direct upper3 M4/lower2 M5 (45 mm pitch) base joints; base9.526003/carriage7.309937 kg. 210 mm deck keepout; six wiper probes,32 retained retainer head/nut sweeps, fastener fit0 and0 Y made-edge findings PASS; library289,hobbing1185,core fastener/integrity/way-cover PASS; enclosure48/128 PASS. Paths135 up/202 down then108 forward at X retract/Z high/Y=A0, mandrel cleared only for lower half; precision supports/X skirt retained. Guards off before stops; front-left stop off before head. Lower moment factors5.6/1.7 at ASSUMED30/100 N,390 mm lever. RFQ46 designs/50 instances90.22 kg PASS. Hobber3357 and full requested sweep0 errors in2217 s/18 root removals clear PASS; valid Y FE and PU peel/clip/port-seal qualification OPEN.
- 2026-11-08 ZB partial (20afedf1f,9f7ca8351,b8fbbeb7c):4 rail bellows/6 made mounts replace2 Z sheets;98 joints346 screws46 pins,210 mm span/30 mm compressed/H24 flanges (thin PU RFQ ASSUMED). X/Y/Z moving202.23/29.80/63.63 kg, drives PASS; guard slip>=169.7 under ASSUMED10 N handling. All32 flange tool corridors and1260 internal pair/state probes PASS; module286,hobbing1185,joint-fasteners (7 machining-identity checks),assembly-integrity,way-cover PASS. Physical machining BOM keys fix repeat-bracket aggregation; recipe/geometry keys stay exact. Nominal strict integrity0 errors; full hobber3234 PASS after correcting stale Y joint/coupling test IDs. Revised17.2 mm opening/cropped flange lands retain complete head bearing;24 travel/overtravel poses now0 ZB errors (was8),46 remaining whole-machine Y findings. Enclosure reaches Y-base/tray-clamp clash827.497 mm3; direct socket-head bearing on3 mm steel eliminates four washer/skin clashes (0.311 mm3 each); final fastener fit0 errors and32 driver corridors PASS. 29 other hobber made-part edge findings remain reported/open (Z saddle2, cradle7, X/Z nut receivers12, X guards8). Y corner fit and offset-screw ingress remain OPEN. No stiffness path changed or FE run.

- 2026-11-08 LM5 wrap-up (76383b88d): LM0/current X/Y/Z moving191.48/28.13/51.42 ->196.25/30.88/65.30 kg; head39.16 ->52.687 kg, A-relative COM(-93.7,84.9,36.1) ->(-74.7,32.9,80.7) mm. A tilt48.39 ->51.0 Nm (rating UNKNOWN), max gravity35.10 ->27.24 versus87 rated/204 peak; brake87 ASSUMED. Y Abbe76.2 ->87.77 mm; X209.05/Z235.9 unchanged. Full COM/BOM and budget-row comparison in MACHINE_DESIGN; conditional RSS class4/target6, new Y stiffness unqualified. Direct-ground carriage45.201 N/um,24.616 MN m/rad (2.23x/2.38x LM0), residuals<8e-11,106.81 s/3730764 KiB. Hobber3361,hobbing1185,module289,core suites,enclosure48/128 and Z-drive42 PASS (race48.5/saddle10/coupling12.5 mm).426 members/163 global BOM lines,347 screws/46 pins; LM0 global BOM unrecorded, Y/head subtree comparisons retained.21 residual made-edge findings (Z saddle2,cradle7,X/Z nut receivers12) remain OPEN; bench60 unchanged. Extended frame/service sweep final result belongs to LM6.

- 2026-11-08 LM6 close-out (prior tooling/package d8354ce60): ground-bar cleanup5264c687d and LM5 comparison76383b88d retained;21 separate tooling rows/85 checked references/EUR1436 ASSUMED. Clean routed RFQ48 designs/52 instances/80.741357 kg passes all STEP/hole/drawing checks. Final hobber3361,hobbing1185,module289,joint-fasteners,assembly-integrity,way-cover,enclosure48 states/128 near pairs and Z-drive42 PASS. Extended requested frame sweep completed with1 running-gap finding in2863 s: swivel/housing versus ZdriveFloatingBar0.8 mm instead of2 at X45.215,Y-10,Z-10,A-15deg; service phase not reached. This correction/rerun, remaining LM4/Y stiffness and PU qualifications, A6,21 residual made edges, ZB offset-screw ingress/complete service, casting refresh, stock/supplier/brake/heat qualifications remain OPEN per timeboxed wrap-up; no further engineering pursued. Tree426 members/68 sets/no ungrouped; editor screenshots unavailable. Implementation wrap-up scope closed; not a machine release.

- 2026-11-09 v2-a: full-length ground walls with rails on their inner reference edges, centred HZMotion/BF12 screw, two plain floor stops, outside scale and plain carriage replace risers, four U stops, scale seat, guards/bellows, seal lands, retainers and PU lips. Optional sheets omitted. Stock-sized bracket, 20 mm head lead and profiled steel base/Al carriage pass native fit with0 errors and0 Y made-edge findings; no aluminium base-thread credit. All seven gates PASS: module172 (standard and hobber sizes), hobber3284, hobbing1185, joint-fasteners/core integrity, enclosure48/66 and both example scenes. Intermediate host base400x328.5/carriage340x420 mm, head51.46 kg, output-relative COM(-69.3,48.2,94.6) mm; A tilt47.36/max gravity35.82 Nm; Y Abbe87.2 mm. Compact proportions, <=39.2 kg, <=230 mm work-Y rigid-part width and the unchanged prior0.8 mm Z-bar gap remain v2-b gates, not achieved here. Stock/finished faces and Y stiffness remain unqualified; no FE.

- 2026-11-10 v2-b: compact 230 x 310.5 x 12 mm steel base, 140 x 290 x 10 mm Al carriage, 230 x 32 x 30 mm walls, 180 mm screw/220 mm rails and +/-15 mm nominal Y plus5 mm overtravel each end. Refitted seat/outboard joints and a single outside-wall scale bracket pass native fastener fit0 errors and0 Y/Z-floating-bar made edges (21 pre-existing findings elsewhere). Modelled A head38.08 kg, output-relative COM(-85.0,89.1,68.6) mm; A tilt44.14/max gravity38.78 Nm; Y Abbe93.2 mm. Rigid work-Y width230 mm <= LM0 plate230 (transverse base310.5, flexible scale-hose envelope251); continuous Z-15..195 eRob/bar gap2.000 mm. Module168, hobbing1185, joint-fasteners/core integrity, enclosure48/66 and both example scenes PASS; hobber3286 PASS; all seven gates PASS. Optional sheets omitted; stock/face, selected key, supplier/brake and Y/outboard stiffness qualifications remain open; no FE.

- 2026-11-10 v2-c LM0 -> final: modelled A head39.16 ->38.08 kg, output-face COM(-93.7,84.9,36.1) ->(-85.0,89.1,68.6) mm; A tilt including100 N cutting48.39 ->44.14 Nm, maximum gravity over A+/-15 at nominal Y0 35.10 ->38.78 Nm (87 rated/204 peak; brake87 ASSUMED, bearing moment rating unknown); Y Abbe76.2 ->93.2 mm (increased). A-head members49 ->50. Joint-descendant inventory uses components plus hardware on clamped descendants, with pins only when both ends are inside: LM0 blockYLeft ->Ymodule/blockLeftFront moving Y34 ->34 members,28.132885 ->25.316819 kg,29 ->29 component BOM lines,42 ->45 with clamped fasteners,52 ->52 screws/8 ->10 pins; LM0 swivelPlate ->Ymodule/base trees47 ->49 members,41 ->43 component BOM/60 ->66 with clamped fasteners,82 ->96 screws/12 ->22 pins (new base includes its integral actuator interface). Global269 ->417 members, BOM total unrecorded ->150 including inventory/tooling,246 ->328 screws/26 ->42 pins; intervening XC/ZB work is included, so global differences are not attributed to v2 alone. Hard gates retain38.08 kg, rigid work-Y width230 mm (transverse base310.5; flexible hose251) and continuous Z-bar gap2.000 mm. Both example configurations now identify the v2 open body and compact fitted dimensions; all seven gates PASS: module168, hobber3286, hobbing1185, joint-fasteners/core integrity, enclosure48 samples/66 exact pairs and both preview builds. Native report: `run-fe.sh machines/flexspline-hobber/tests/hobber/haxeon.frames.json swivel scales v2 v2inventory`. No geometry changed in this step. Stock/face, selected key, supplier/brake and Y/outboard stiffness qualifications and21 unrelated made-edge findings remain open; the legacy Y FE script still uses old riser inputs and is not v2-qualified. No FE.

- 2026-11-10 v2 follow-up: two2 mm screw-support pads remove the nut-carrier floor pocket; both rail walls narrow32 ->25 mm and stand32 mm high. Two identical7 mm scale-housing spacers preserve assembled AF3 access, four added pieces total, with existing fasteners and the two floor stops retained. Plain channel floor12 mm, carrier clearance3 mm; work-Y rigid span230 mm, transverse base303.5 mm, carriage140 x290 mm, screw180/rail220 mm. Supplier-finished packing, actual pad bolt lands, support pin-bearing material and persisted clamp grips are checked; preview machining/BOM generation corrected. Hard gates PASS: head37.40 kg <=39.2, rigid work-Y width230 mm, continuous Z-bar/eRob gap2.000 mm. LM0 -> current: head39.16 ->37.40 kg, output-relative COM(-93.7,84.9,36.1) ->(-85.9,89.8,67.3) mm, A tilt48.39 ->43.89/max gravity35.10 ->38.19 Nm, Y Abbe76.2 ->93.2 mm. A-head members49 ->54. Global421 members/154 BOM lines/328 screws42 pins; Y-base subtree53 members/46 component BOM lines68 with clamped hardware/96 screws22 pins; moving Y34 members25.314398 kg/29 component BOM lines45 with clamped hardware/52 screws10 pins. All seven gates PASS: module230 (both sizes), hobber3302, hobbing1185, joint-fasteners, core integrity, enclosure48/66 and both example scenes. Packed support pins have3 mm actual foot engagement, separately checked with packing excluded: rated/peak thrust bearing factors4.51/1.50 against1.5/1.2 required (strengths ASSUMED). Stock/face, selected key, supplier/brake and Y/outboard stiffness qualifications and21 unrelated made-edge findings remain open. No FE.
