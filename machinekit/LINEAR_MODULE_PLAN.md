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
- **OPEN at wrap-up:** valid Y stiffness and resulting accuracy release (prior Y solve
  rejected); bought-ground-stock/finished-land inspection; rib friction coupon; A6 column
  squareness and assembly pins; PU wiper peel/retention/elastic release and port/end seals;
  residual made-edge findings outside new carriage/sole/Y/bar joints; complete Z/wiper
  service/tool treatment and offset-screw chip ingress; thin Z bellows supplier acceptance;
  eRob bearing moment rating (unknown) and brake87 Nm (ASSUMED); motor thermal commissioning
  and conditional cooling; casting-package refresh/large-face qualification; other unpublished supplier tolerances. These are recorded,
  not pursued in this timeboxed close-out. UI capture unavailable; use model tree evidence.

## Log

- 2026-11-07 LM0 (`fedf4ff0c`): before edits,hobber2345/hobbing1161; joint-fasteners,assembly-integrity,way-cover PASS; requested frames0 errors,13 services,269 members,68 joints246 screws26 pins. X carriage24.586824 kg; Y28.132885 kg; head39.16 kg; A tilt48.39 Nm, max gravity35.10 versus87 rated204 peak (brake87 ASSUMED). FE baseline/Abbe/budget details: examples/flexspline-hobber/LINEAR_MODULE_BASELINE.json.
- 2026-11-07 XC edge complete: plain made-only rule; bought own holes excluded. Bench unchanged232 screws16 pins,60 findings, unchanged and report-only; other examples have no derived joints. Post-trim hobber2453 PASS;50 existing hobber findings elsewhere, none on the new carriage/sole joints; no material exceptions.
- 2026-11-07 XC carriage complete:20 mm ground deck +bright-bar ribs,26.242 kg versus24.587; bolts-only43.633 N/um,17.077 MN m/rad; accuracy PASS29 s, predicted class4/target6. Post-trim hobber2453 PASS; retained pre-trim hobbing1194,joint-fasteners,assembly-integrity,way-cover and frames0 errors1728 s/13 services gates as directed. Single FE result; service framework and unrelated head changes removed.
- 2026-11-07 XC sole complete:four80 mm deep ground-stock corner pads,8 M8; contact-only tilt screen2.225x before, per-pad whole-shear slip38.1 normal/7.2 limited crash. Source anchor demand includes joint service threads; central actuator path stays open. Trimmed hobber2453 PASS; split commits689a2fbb4 (edge),f28c59f45 (carriage),25f7ca127 (sole),3.696 kg pads.
- 2026-11-08 XA wrap-up: motor bay/lid, straight tray edge, guard insert feet and ten checked assembly steps retained; continuous X/Z pads, nutGroove, drive replication/spacers/arbors/height fixture/reaming kit and layer-aware fastener derivation removed. Four matched ground bars X15/Z40 mm, fixed70x75/BF65x35 from80 mm stock; no channel groove. Support height unchanged; fixed-bar slip X7.4/3.8 and Z3.4/1.3 normal/crash, default steel-joint mu0.15 (ASSUMED). Four AF2.5 guard-foot corridors; A6 and complete Z/wiper service remain OPEN. Hobber3361,hobbing1185,module289,joint-fasteners/core integrity/way-cover,enclosure48/128 and Z-drive42 PASS. Direct-ground carriage FE45.201/24.616 replaces film result43.633/17.077; remaining LM4/ZB qualifications stay OPEN.
- 2026-11-07 LM1 nominal module: own/host base, derived ground risers, M60D-BK12/BF12/SFU1605, four guide blocks and joint-derived stops; HG15 strokes20/80 and HG25 stroke200 PASS60 checks including strict integrity, three service paths, end-stop contact, slip, roundtrip and alignment for every joint. Screws170/240/370 mm, rails220/300/440, risers25x25/25x25/25x15;58/62/70 screws and18 pins. Hobber2633,hobbing1186,joint-fasteners and assembly-integrity PASS. Stock flatness0.10/0.05 mm ASSUMED versus0.01 acceptance remains unqualified; host pattern/torque-limit qualification and supplier face finishing remains open. No FE added.
- 2026-11-07 LM2 model gates: JFT-40 ML70/120/220,208/258/358 mm housings on ground40x20 seats;66/70/78 screws,18 pins. Own102 checks PASS (physical made edges, five services, key/air corridors, overtravel window, roundtrip, independent position/native-motor feedback); hobber2633 PASS. Head normal/gap stacks0.24/0.22 versus+/-0.5/0.55 mm settings, R75 cable, key D3.5x8/90 and air socket D14x35 ASSUMED. Abbe from module carriage interface111.2/36 mm HG15 and133.2/38 HG25; minimum slip4.482 normal/1.494 momentary, scale89.411/44.706 at ASSUMED20/40 N. Base/riser/block stock screens0.45/0.55/0.76 versus0.01 mm fail; inspect/select/finish and qualify fitted head face before release. Fixed M6/pin annulus by66 mm bolt spacing; no FE. 2026-11-08 short cable return toward +module Y passes286 library assertions and actual-solid head-against-machine probes at three low-Z tilt states (zero overlaps), removing the former721.23 mm3 cable/X-carriage clash.
- 2026-11-07 LM3 model gates: short split cover/retainers/PU lips plus bolted cable hood; long80/200 use two/four-fold lamella bellows with22/38 mm compressed,30/50 space,120/260 extended. Own276 assertions PASS across all three sizes (integrity, physical made edges, slip, contact, access, service, seal contact/overtravel, part refs, roundtrip/feedback); hobber2633 PASS.86/122/130 screws,18 pins; minimum joint factors4.482/1.494. Fixed/moving flange contacts nominal672/736 mm2 versus625; short payload62x192,overlap10,roof gap6 mm. Guard masses1.881/8.775/8.018 kg (rigid moving0/4.135/3.710), cover geometry/density ASSUMED; clip/end-seal/vendor qualification open. No precision stiffness path changed, no FE.
- 2026-11-08 LM4 service follow-up: upper/lower host guards, roof58/carriage back69/end hoods91 mm, direct upper3 M4/lower2 M5 (45 mm pitch) base joints; base9.526003/carriage7.309937 kg. 210 mm deck keepout; six wiper probes,32 retained retainer head/nut sweeps, fastener fit0 and0 Y made-edge findings PASS; library289,hobbing1185,core fastener/integrity/way-cover PASS; enclosure48/128 PASS. Paths135 up/202 down then108 forward at X retract/Z high/Y=A0, mandrel cleared only for lower half; precision supports/X skirt retained. Guards off before stops; front-left stop off before head. Lower moment factors5.6/1.7 at ASSUMED30/100 N,390 mm lever. RFQ46 designs/50 instances90.22 kg PASS. Hobber3357 and full requested sweep0 errors in2217 s/18 root removals clear PASS; valid Y FE and PU peel/clip/port-seal qualification OPEN.
- 2026-11-08 ZB partial (20afedf1f,9f7ca8351,b8fbbeb7c):4 rail bellows/6 made mounts replace2 Z sheets;98 joints346 screws46 pins,210 mm span/30 mm compressed/H24 flanges (thin PU RFQ ASSUMED). X/Y/Z moving202.23/29.80/63.63 kg, drives PASS; guard slip>=169.7 under ASSUMED10 N handling. All32 flange tool corridors and1260 internal pair/state probes PASS; module286,hobbing1185,joint-fasteners (7 machining-identity checks),assembly-integrity,way-cover PASS. Physical machining BOM keys fix repeat-bracket aggregation; recipe/geometry keys stay exact. Nominal strict integrity0 errors; full hobber3234 PASS after correcting stale Y joint/coupling test IDs. Revised17.2 mm opening/cropped flange lands retain complete head bearing;24 travel/overtravel poses now0 ZB errors (was8),46 remaining whole-machine Y findings. Enclosure reaches Y-base/tray-clamp clash827.497 mm3; direct socket-head bearing on3 mm steel eliminates four washer/skin clashes (0.311 mm3 each); final fastener fit0 errors and32 driver corridors PASS. 29 other hobber made-part edge findings remain reported/open (Z saddle2, cradle7, X/Z nut receivers12, X guards8). Y corner fit and offset-screw ingress remain OPEN. No stiffness path changed or FE run.

- 2026-11-08 LM5 wrap-up: LM0/current X/Y/Z moving191.48/28.13/51.42 ->196.25/30.88/65.30 kg; head39.16 ->52.687 kg, A-relative COM(-93.7,84.9,36.1) ->(-74.7,32.9,80.7) mm. A tilt48.39 ->51.0 Nm (rating UNKNOWN), max gravity35.10 ->27.24 versus87 rated/204 peak; brake87 ASSUMED. Y Abbe76.2 ->87.77 mm; X209.05/Z235.9 unchanged. Full COM/BOM and budget-row comparison in MACHINE_DESIGN; conditional RSS class4/target6, new Y stiffness unqualified. Direct-ground carriage45.201 N/um,24.616 MN m/rad (2.23x/2.38x LM0), residuals<8e-11,106.81 s/3730764 KiB. Hobber3361,hobbing1185,module289,core suites,enclosure48/128 and Z-drive42 PASS (race48.5/saddle10/coupling12.5 mm).426 members/163 global BOM lines,347 screws/46 pins; LM0 global BOM unrecorded, Y/head subtree comparisons retained.21 residual made-edge findings (Z saddle2,cradle7,X/Z nut receivers12) remain OPEN; bench60 unchanged. Extended frame/service sweep final result belongs to LM6.
