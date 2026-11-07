import machinekit.design.DesignRequirement;
import machinekit.hobbing.HobSpindle;
import materia.assembly.AssemblyDefinition.AssemblyJointCoordinate;
import materia.presentation.AssemblyPresentation;

/**
 * Assembly procedures for the hobber's axes (FRAME_PLAN.md H7): how the screw supports are aligned to the nut and pinned, and how
 * rails are set against an edge. Steps are notes and joint positions; they highlight the members the step works on. The
 * seat holes the pinning needs are `HobberDrives.seatHoles`; the nut block's bolts are `HobberDrives.nutBlockHoles`.
 */
class HobberAssemblyProcedures {
  static function select(id:String):PartSelector return {occurrence: id, subtree: false};

  static function note(id:String, title:String, instruction:String, members:Array<String>):ProcedureStep return {
    id: id,
    title: title,
    instruction: instruction,
    highlight: [for (member in members) select(member)],
    action: Note
  };

  /** Fit the supports loose, run the carriage to both ends so the screw centres on the nut, tighten, ream and pin. */
  public static function alignScrewSupports(axis:String, travel:{lower:Float, upper:Float}):Procedure {
    // X and Z have one ground bar under both supports; Y has a bracket under each.
    var fixedPad = axis == "Y" ? "YdriveFixed" : axis + "drivePad", floatingPad = axis == "Y" ? "YdriveFloating" : axis + "drivePad";
    var members = [axis + "fixed", axis + "floating", axis + "screw", axis + "nut", axis + "nutBlock"];
    return {
      id: axis.toLowerCase() + "-align-supports",
      label: axis + ": align the screw supports and pin them",
      kind: Assembly,
      start: Assembled,
      steps: [
        note("seat", "Fit the supports loose",
          "Bolt the motor mount to its pad and the BF12 to the floating pad with the tapped bolts only snug (no dowels). The pads are drilled and tapped, "
          + "not reamed: the supports can still move about 0.5 mm.", (fixedPad == floatingPad ? [fixedPad] : [fixedPad, floatingPad]).concat([axis + "fixed", axis + "floating"])),
        note("nut", "Bolt the nut block to the carriage",
          "Fit the nut, its block and the block's two carriage bolts, then thread the screw through the nut into the mount and the BF12.",
          [axis + "nut", axis + "nutBlock", axis + "screw"]),
        {id: "low", title: "Run the carriage to the lower end", instruction: "Turn the screw by hand until the carriage rests at the lower end of its travel; the nut must turn freely.",
          highlight: [select(axis + "nut")], action: SetJoints([{joint: axis, value: travel.lower}])},
        note("centre-low", "Centre the screw on the nut",
          "With the nut at this end, move the floating support (the end the screw sits loose in) until the screw spins without binding at the nut; snug its bolts.", [axis + "floating"]),
        {id: "high", title: "Run the carriage to the upper end", instruction: "Turn the screw by hand to the other end; the nut must still turn freely.",
          highlight: [select(axis + "nut")], action: SetJoints([{joint: axis, value: travel.upper}])},
        note("centre-high", "Check the screw is centred at both ends",
          "Repeat the free-turning check at both ends. If it binds, loosen the supports and shift them by half the difference.", members),
        note("tighten", "Tighten, then ream and pin",
          "Tighten all support bolts. Ream the two dowel holes in each support foot through the pad at "
          + "5 mm and drive the dowels. The pins keep the alignment, so the pad needs no precision machining.", (fixedPad == floatingPad ? [fixedPad] : [fixedPad, floatingPad]).concat([axis + "fixed", axis + "floating"]))
      ],
      source: Generated("hobber")
    };
  }

  /** Rails against an edge: the datum edge of the seat sets the rail's line, the push screws hold it while the bolts are tightened. */
  public static function setRails(axis:String, rails:Array<String>):Procedure {
    return {
      id: axis.toLowerCase() + "-set-rails",
      label: axis + ": set the rails against their edge",
      kind: Assembly,
      start: Assembled,
      steps: [
        note("clean", "Clean the seat and the rail", "Stone the seat face and the rail underside, and wipe the datum edge.", rails),
        note("edge", "Lay the rails against the edge",
          "Seat each rail against its datum edge and bolt it down finger-tight; push screws or side bars hold it against the edge along its whole length.", rails),
        note("bolts", "Tighten from the datum end",
          "Tighten the rail bolts from the datum end along the rail, keeping the edge contact. Check straightness with an indicator against the rail side.", rails),
        note("second", "Set the second rail parallel",
          "Tighten the second rail the same way, then check both rails' parallelism and the carriage's free running.", rails)
      ],
      source: Generated("hobber")
    };
  }

  /**
   * B belt drive (head redesign): 40T pulleys on both shafts, a whole-tooth 450-5M belt on 125 mm centres. The cradle is bolted and dowelled to the
   * seat block's top first (that joint is the one the drive's first-mode estimate depends on). The servo then slides along the belt's line in the
   * wall's vertical slots on its pilot, a jack screw in the lug under its flange raises it, and the flange bolts are tightened once the belt carries
   * its pretension (frequency of the free span, or the deflection under a stated force). Values come from `HobSpindle` and the drive check
   * `HobberFrameChecks.spindleDrive` prints them too.
   */
  public static function tensionBelt(machine:FlexsplineHobber):Procedure {
    var drive = ["spindle/motor", "spindle/motorPulley", "spindle/spindlePulley", "spindle/belt"], spindle = machine.spindle, rating = spindle.motor.rating;
    function r(v:Float, d:Int = 1):Float return Math.round(v * Math.pow(10, d)) / Math.pow(10, d);
    var t = HobSpindle.pretension(rating.peakTorque, spindle.motorPulley.pitchDiameter), f = HobSpindle.spanFrequency(t), span = HobSpindle.centres();
    var travel = HobSpindle.tensioningTravel(t, spindle.belt.length), force = 4 * t * (span / 64) / span;
    return {
      id: "b-tension-belt",
      label: "B: fit the belt and set its tension",
      kind: Assembly,
      start: Assembled,
      steps: [
        note("cradle", "Bolt the cradle to the seat block",
          "Stone the seat block's top and the cradle's underside, set the cradle on the block with its two dowels and tighten its four M8 into the block's top "
          + "(this is a preloaded, dowelled joint: tighten crosswise to the screw's full torque). The cradle's slit lies over the seat's own so the clamp still closes.",
          ["motorCradle", "spindleSeat"]),
        note("motor", "Seat the servo on its slides",
          "Offer the servo to the cradle wall with its Ø70 pilot in the wall's vertical slot (the slot is a sliding fit on the pilot) and its connectors up, the four M6 through the "
          + "flange into the wall's slots with a washer and nut on the wall's rear face, finger-tight. Run the jack screw (M8 in the lug under the flange) down to the lug so the servo "
          + "rests at the bottom of its " + r(2 * HobSpindle.TENSION_TRAVEL) + " mm of slot travel. The shaft must be flush with the pulley's rear face.",
          ["motorCradle", "spindle/motor"]),
        note("pulleys", "Fit the pulleys",
          "Set the " + spindle.motorPulley.teeth + "T pulley on the servo shaft and the " + spindle.spindlePulley.teeth + "T pulley on the cartridge stub, their front faces 16 mm behind the housing rear face (the encoder's rotor lies in front of them), belt planes aligned with a straightedge across both.",
          ["spindle/motorPulley", "spindle/spindlePulley"]),
        note("belt", "Put the belt on",
          "With the servo at the bottom of its slots the centres are " + r(span - HobSpindle.TENSION_TRAVEL) + " mm: slip the " + r(spindle.belt.length, 0) + " mm belt over both pulleys without rolling it.", drive),
        note("jack", "Set the tension with the jack screw",
          "Turn the jack screw up, raising the servo, until the free span (" + r(span, 0) + " mm) rings at " + r(f, 0) + " Hz on a sonic tension meter, i.e. " + r(t, 0) + " N per strand "
          + "(belt mass " + HobSpindle.BELT_MASS_PER_M + " kg/m assumed; set the meter to the maker's value and recompute). Alternatively, deflect the middle of the span by 1/64 of its length ("
          + r(span / 64, 2) + " mm) under " + r(force, 0) + " N. The belt needs about " + r(travel, 2) + " mm more centre distance than slack to reach this (cord stiffness "
          + HobSpindle.BELT_CORD_RIGIDITY + " N per mm of width assumed), well inside the " + r(HobSpindle.TENSION_TRAVEL) + " mm of travel. The jack bears on the flange from below, against the belt's pull.", drive),
        note("clamp", "Tighten the flange bolts",
          "Holding the jack where it is, tighten the four M6 crosswise to their full torque (the friction of the preloaded flange holds the servo; the drive check prints its margin over the belt pull), "
          + "then re-read the tension and trim with the jack and re-tighten if it moved. Lock the jack screw with thread-locker.", ["motorCradle", "spindle/motor"]),
        note("rest", "Shim the servo body onto the rest block",
          "With the tension set and the flange bolts tight, measure the gap between the rest block's ground top (it stands " + r(HobSpindle.TENSION_TRAVEL + 0.5) + " mm under the body's underside at the bottom of the slots, so the servo can still go down to fit the belt) "
          + "and the body's underside, at both ends of the block, with feeler gauges: about " + r(HobSpindle.TENSION_TRAVEL + 0.5 + travel, 1) + " mm. Make a pack of ground steel shim 0.02 mm thinner than the smaller gap (peel shim in 0.05 and 0.1 mm layers, at most "
          + "8 laminations, no wedge over 0.05 mm) and slide it in, bearing on the block's full width. The belt's pull and the servo's weight keep the body pressed on it; it carries the servo's weight and "
          + "makes the first mode of the servo group clear the tooth-pass band (the drive check prints the FE and hand-model values). Re-fit the pack whenever the tension is trimmed.",
          ["motorCradle", "spindle/motor"]),
        note("run", "Run in and re-check",
          "Turn the spindle by hand, then run it at 500 rpm for ten minutes and re-measure the tension; the belt must track on both pulleys without climbing a flange.", drive)
      ],
      source: Generated("hobber")
    };
  }

  /**
   * Commissioning: the guides' pitch and yaw over the travel, measured with an autocollimator or laser angular optics, go into the
   * controller's correction table, so the angular error times the offset to the tool (the Abbe error, see `HobberFrameChecks.scales`) is
   * compensated instead of left in the part. Roll about the travel is not measured here (it needs a level or the optics' roll option).
   */
  public static function compensateTilt(axis:String, travel:{lower:Float, upper:Float}, rails:Array<String>):Procedure {
    var scale = axis + "scale";
    return {
      id: axis.toLowerCase() + "-compensate-tilt",
      label: axis + ": measure pitch and yaw and load the correction table",
      kind: Service,
      start: Assembled,
      steps: [
        note("setup", "Set up the angular optics",
          "Put the autocollimator or laser angular reflector on the carriage (or the tool seat) and the optics on a stand beside the machine, aligned to the " + axis
          + " travel. Let the machine reach its working temperature first.", rails),
        {id: "low", title: "Carriage at the lower end", instruction: "Move " + axis + " to the lower end of its travel and zero the optics.",
          highlight: [select(scale)], action: SetJoints([{joint: axis, value: travel.lower}])},
        note("sweep", "Step through the travel",
          "Move in 20 mm steps to the upper end and back, stopping at each point to read pitch and yaw (the two angles about the axes across the travel) "
          + "from the scale reading. Repeat once in the other direction to see the hysteresis.", [scale]),
        {id: "high", title: "Carriage at the upper end", instruction: "Read the last point at the upper end of the travel.",
          highlight: [select(scale)], action: SetJoints([{joint: axis, value: travel.upper}])},
        note("table", "Load the correction table",
          "Enter pitch and yaw against scale position in the controller's tilt correction table for " + axis + ", with the Abbe offsets to the hob centre from the "
          + "machine's scale report. Re-run the sweep with the table on and keep the residual as the acceptance record.", [scale])
      ],
      source: Generated("hobber")
    };
  }

  public static function fitBt30Arbor():Procedure return {
    id: "b-fit-bt30-arbor", label: "B: seat the BT30 arbor and secure the hob", kind: Assembly, start: Assembled,
    steps: [
      note("stud", "Match the pull stud and gripper", "Use the cartridge maker's M12 45-degree pull stud; verify its head and retention force. The model's stud outline and retention force are assumed. Inspect the clean taper and confirm the supplier's drawbar clamping specification before running.", ["spindle/housing", "spindle/pullStud"]),
      note("taper", "Seat and indicate the BT30 arbor", "Clean and blue-check the BT30 taper, clamp the drawbar and indicate the ground hob seat and journal. The standard BT30 taper locates; do not force the holder flange against the spindle face. The model's 2 micrometre errors are assumed acceptance requirements.", ["spindle/shaft", "spindle/arbor"]),
      note("hob", "Engage the face key and lock the retaining nut", "Slide the plain-bore hob onto the ground diameter 16 seat with its end slot over the integral face key. Tighten the M16x1 nut against the hob to 2000 N axial preload (ASSUMED assembly target; establish shop torque by test). Secure with medium-strength thread-locking compound and witness-mark it; verify the hob is seated and cannot turn on the arbor.", ["hob", "spindle/arbor", "spindle/hobLocknut"]),
      note("service", "Withdraw the cartridge toward the tool", "Park X at retract, stop and isolate B, remove the belt and the outboard support with its bearing, loosen the split clamp, and draw the complete cartridge forward. Its front register cannot pass rearward through the clamp bore.", ["spindle/housing", "spindleSeat", "outboard"])
    ], source: Generated("hobber")
  };

  public static function alignOutboard():Procedure return {
    id: "b-align-outboard", label: "B: preload, indicate and pin the floating outboard pair", kind: Assembly, start: Assembled,
    steps: [
      note("pair", "Assemble the matched DB pair", "Use two SKF 7002 CDGA/P4A bearings back-to-back, arrows and matched faces per SKF. Seat both outer rings against the sleeve shoulder with the M36x1 gland, then clamp both inner rings between the arbor D20 shoulder and ground D20 spacer with the M15x1 AF22 nut. Their ground faces establish class-A preload (SKF: 20 N unmounted), not a torque-derived preload. Shaft fit and retaining torque require shop qualification to avoid race creep and excessive interference preload; use fillets <=0.2 mm, clean precision grease and verify free rotation and temperature. Secure both retaining threads with medium-strength threadlocker and witness marks.", ["spindle/arbor", "outboardSleeve", "outboardGland", "outboardBearingFront", "outboardBearingRear", "outboardInnerSpacer", "outboardLocknut"]),
      note("indicate", "Indicate the housing to the spindle axis", "Fit the Y scale reading head after the outboard carrier's screws are driven (the head's M4 run through its thickness and are driven from the plate side with the saddle off the plate, where the carrier's own driver axis passes the head's row). Before fitting the carrier to the Y saddle, fit the housing screws and washers in the 5.5 mm deep carrier recesses (heads finish 0.5 mm below the saddle face). Fit the carrier to its Y-saddle pins. For indication leave the housing screws snug and housing pins absent, indicate a ground test arbor at both ends of the D40 housing bore; shim the carrier face for X/tilt and use screw clearance for Z. Require <=2 micrometres radial residual (<=4 micrometres TIR), then lift the carrier off, torque the four derived M4 from its saddle side, refit it on its dowels and recheck. Ream the two derived D4 H7 housing/carrier pin bores together, fit the dowels and recheck. Worst-case radial budget: indication 2 + seat 1 + bearing 1.5 + thermal 0.5 = 5 micrometres, all project acceptance assumptions.", ["outboardCarrier", "outboard"]),
      note("float", "Verify thermal float and labyrinth clearance", "Before final inner-ring clamping verify the sleeve slides without binding in the D40 H6/g5 fit. At cold assembly centre it between the front housing stop and rear cap spigot: 1 mm travel each direction. The spindle cartridge alone locates axially; the entire preloaded pair follows the expanding arbor. The front D21 sleeve throat clears the D20 arbor shoulder by 0.5 mm radial. The 30 K thermal case, 12e-6/K steel coefficient and 0.2 mm assembly allowance are assumed; verify the retained travel and keep the labyrinth clean.", ["outboard", "outboardSleeve", "outboardCap"]),
      note("remove", "Free the retaining stack before withdrawing the support", "At X retract isolate B. Undo the M15x1 nut using a thin 22 AF socket with outside diameter <=28 mm through the D30 cap bore. Slide the nut and inner spacer off the journal first. Remove the Y scale head and carrier screws, then pull the complete carrier/housing/pair forward off the journal (use a bearing puller if the qualified shaft fit requires it). The support is located by its dowels when refitted; re-indicate after service.", ["outboardLocknut", "outboardInnerSpacer", "outboardCarrier", "outboard"])
    ], source: Generated("hobber")
  };

  public static function fitChipGuards():Procedure return {
    id:"fit-chip-guards",label:"Fit the fixed chip sheets and overlapping carriage skirt",kind:Assembly,start:Assembled,
    steps:[
      note("bench","Preassemble the fixed X guard on the bench","Bolt the low front and high rear side sheets to their front and rear lids on the bench, with the M3 heads below the side roofs. This leaves room for the nut driver before the assembly is lowered onto the casting. Do not weld the sheet corners or the lapped height transition.",["XcoverFrontLeft","XcoverFrontRight","XcoverRearLeft","XcoverRearRight","XfrontCover","XrearCover"]),
      note("feet","Lower the guard before fitting the carriage","Set the formed feet on the X pad and fit the four joint-derived M3 through the tool ports. Use a long 2.5 mm hex key. After the carriage is fitted, front ports are reached at X retract; rear ports at X minimum. Fit the NBR plugs after tightening; their procurement envelope and oil compatibility remain assumed requirements.",["XdrivePad","XfrontCover","XrearCover","XfrontPlugLeft","XfrontPlugRight","XrearPlugLeft","XrearPlugRight"]),
      note("moving","Fit the skirt and the Z shields to the carriage underside","Bolt the skirt's four tabs and each Z shield's two fixings to the carriage underside. These holes belong to the carriage's existing bottom milling setup; no side drilling or casting modification is needed. Keep the sheet's R2 bends and relief slots free, and keep its feet out of the carriage ribs. The Z shields can be fitted after the saddle using the reverse z-service-erob path.",["Xbridge","Xskirt","ZcoverLeft","ZcoverRight"]),
      note("gaps","Verify the overlaps over full travel","With screws snug, set at least 2 mm running clearance and then tighten. Check the low fixed sheets at A +/-15, X limits and minimum Z. The rear skirt return has reliefs over the high side lips. Nominal end overlaps include 5 mm linear overtravel; flat side overlap is 6 mm. The Z sheets shield the offset screw and used guide region while the saddle moves behind them. Keep the existing block wipers.",["Xskirt","XfrontCover","XrearCover","ZcoverLeft","ZcoverRight"]),
      note("clean","Keep the tool ports closed","Brush chips from the lapped roofs and refit all four port plugs. These are fixed overlapping chip shields; the enclosure contains mist. No drain is declared for the flat channels, and no new casting floor or bore is implied.",["XfrontPlugLeft","XfrontPlugRight","XrearPlugLeft","XrearPlugRight"])
    ],source:Generated("hobber")
  };

  public static function serviceAxes(machine:FlexsplineHobber):Array<Procedure> {
    var module:Array<String> = [], xStroke=0.0;
    for (r in machine.serviceRemovals()) switch r {
      case ServiceRemoval("erobWithSaddle", members, _, _, _): module = members;
      case ServiceRemoval("xScrewWithdrawal", _, steps, _, _): xStroke=steps[0].distance;
      default:
    }
    var kg = Math.round(HobberJointLoads.massOf(machine,module)*100)/100;
    return [
      {id:"x-service-screw",label:"X: withdraw screw and nut rearward",kind:Service,start:Assembled,steps:[
        note("park","Park and isolate X","Park X at its initial 45 mm position, stop and isolate the axes. Leave the X carriage and chip shields installed. Provide rear workspace for the " + xStroke + " mm withdrawal stroke.",["Xscrew","Xnut","XnutBlock"]),
        note("release","Release the rear drive and nut flange","Remove the X servo and shaft stub, BK12 motor bracket/fixed support and BF12 retaining ring; the BF12 stays in place. Support the screw. Through the rear channel opening, use a long AF4 key (at least 400 mm reach, shop tool requirement) to remove the joint-derived M5 nut-flange screws. The nut block stays bolted and pinned to the carriage.",["Xmotor","Xfixed","XfloatingRing","Xnut","XnutBlock"]),
        note("withdraw","Slide the screw, coupling and nut rearward","Draw the screw, coupling and nut " + xStroke + " mm toward the rear. The motor-side flange leaves directly, while the nut barrel passes through the stationary block. Keep the nut on the screw or fit its transfer tube; do not spill the recirculating balls. The declared swept path leaves all chip shields installed.",["Xscrew","Xcoupling","Xnut"]),
        note("refit","Refit and recheck the axis","Reverse the path, refit the BF ring and pinned support, tighten the nut flange and drive fasteners, then check free running, end reserves and encoder zero before restoring motion.",["Xscrew","Xnut","Xfixed"])
      ],source:Generated("hobber")},
      {id:"z-service-erob",label:"Z: remove the saddle, eRob and head as a module",kind:Service,start:Assembled,steps:[
        {id:"park",title:"Park for module removal",instruction:"Park X at retract, centre Y and A, and set Z to joint zero. Stop and isolate drives; retain the Z brake. Clear the work area.",highlight:[select("Zplate")],action:SetJoints([{joint:"X",value:110.0},{joint:"Y",value:0.0},{joint:"Z",value:0.0},{joint:FlexsplineHobber.A,value:0.0}])},
        note("ymotor","Remove the Y servo first","Withdraw the Y servo and shaft stub along their already declared axial path. Its connector envelope blocks the right Z shield's first forward move; keep the servo off for module removal.",["Ymotor","Ymotor-shaft","ZcoverRight"]),
        note("guards","Remove the two Z shields","Unbolt each shield's two underside foot screws and support the loose sheet. Move it 19 mm forward, 24 mm upward, 4 mm back, then 200 mm outwards to its own side. This frees the under-deck foot and clears the head before sideways withdrawal. Fit by reversing this path after the Z saddle is assembled.",["ZcoverLeft","ZcoverRight"]),
        note("support","Support the complete module","The selected module weighs " + kg + " kg from the installed parts. Support the saddle and head with a rated hoist and qualified shop slings/fixture before releasing it. Lifting arrangement and load balance remain shop assumptions to qualify; no unmodelled lifting holes are implied. Disconnect A/B/Y cables, feedback and coolant services.",module),
        note("release","Leave the nut block and guide blocks on the machine","Support the Z scale reading head on the fixed scale and unbolt its arm from the saddle's back face (the head's own M4 are driven from the channel wall's side, closed once the saddle is on the rails: the head stays on its arm), and secure it without loading its cable. Release the Z nut block's base screws and dowels from the saddle, leaving nut and block on the braked screw. Remove the saddle-to-guide-block screws; all four blocks stay on the rails. Keep the eRob, adapter and complete head bolted to the saddle: the adapter output screws are buried behind the swivel plate.",["Zplate","ZnutBlock","ZscaleArm","ZscaleHead"]),
        note("withdraw","Draw the saddle and head forward together","With the module supported, move the saddle, eRob and complete head 60 mm forward. The declared path retains the column, rails, nut block, fixed scale and reading head. Service the actuator on the bench, reversing the head assembly order as needed. Refit and indicate the saddle, reconnect the scale head, refit the shields along their reverse paths and recheck axis zero before operation.",module)
      ],source:Generated("hobber")}
    ];
  }

  public static function build(machine:FlexsplineHobber):Array<Procedure> {
    var result:Array<Procedure> = [];
    result.push({id:"x-assembly", label:"X: checked assembly and alignment", kind:Assembly, start:Assembled,
      steps:[for (i => step in HobberXAssembly.steps()) note("x"+i, step.id,
        step.measurement + " Tools: " + step.tools.join(", ") + ".", step.parts)], source:Generated("hobber")});
    for (travel in machine.travel) if (travel.id != "X") result.push(alignScrewSupports(travel.id, {lower: travel.lower, upper: travel.upper}));
    result.push(tensionBelt(machine));
    result.push(fitBt30Arbor());
    result.push(alignOutboard());
    result.push(fitChipGuards());
    result = result.concat(serviceAxes(machine));
    result.push(setRails("Z", ["railZLeft", "railZRight"]));
    result.push(setRails("Y", ["railYLeft", "railYRight"]));
    var rails = ["X" => ["railXLeft", "railXRight"], "Y" => ["railYLeft", "railYRight"], "Z" => ["railZLeft", "railZRight"]];
    for (travel in machine.travel) if (rails.exists(travel.id)) result.push(compensateTilt(travel.id, {lower: travel.lower, upper: travel.upper}, rails.get(travel.id)));
    return result;
  }
}
