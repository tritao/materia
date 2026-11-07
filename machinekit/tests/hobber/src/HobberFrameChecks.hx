import machinekit.hobbing.HobberDrives;
import machinekit.assembly.AssemblyIntegrity;
import machinekit.assembly.PosedParts;
import machinekit.design.DesignRequirement;
import machinekit.assembly.MachineAssembly;
import machinekit.component.MachineKitComponents;
import machinekit.hobbing.FoundationLayout;
import machinekit.hobbing.HobSpindle;
import machinekit.hobbing.HobberFeResults;
import machinekit.hobbing.HobberCastingPockets;
import machinekit.hobbing.HobberSaddle;
import machinekit.hobbing.HobberSpindleRelief;
import machinekit.hobbing.HobberXLayout;
import machinekit.hobbing.HobberScales;
import machinekit.hobbing.SpindleHousing;
import machinekit.hobbing.HobberSwivelLayout;
import machinekit.hobbing.HobberZLayout;
import materia.assembly.AssemblyFrames as F;

/** Frame, drive and integrity checks on the assembled hobber (FRAME_PLAN.md H1-H6). */
class HobberFrameChecks {
  /** `AssemblyIntegrity` in strict mode on the whole machine; `sweep` false checks the rest state only. */
  static function integrityOptions(machine:FlexsplineHobber, sweep:Bool, gap:Float = 2.0):IntegrityOptions {
    // The column is lowered onto the carriage after the carriage's block and nut-block screws are fitted: those screws are reached before it is in place
    // (the assembly order, per fastener: the column still blocks every other screw's driver and counts for holes and heads).
    var options:IntegrityOptions = {strict: true, processContacts: machine.processContacts(), runningGap: gap,
      fittedBefore: [
        // Fit the belt after fastening the servo flange; the stationary loop covers its wrench access.
        {member: "spindle/belt", fastened: ["spindle/motor"]},
        {member: "column/", fastened: ["Xbridge"], faces: ["blockL1", "blockL2", "blockR1", "blockR2", "nut-block"]},
        {member: "XscaleHead", fastened: ["Xrib-scale"], faces: ["plate"]},
        // Tighten the sole after lowering the column, before fitting its removable skirt.
        {member: "Xskirt", fastened: ["Xbridge"], faces: [for (pad in machinekit.hobbing.FoundationLayout.columnSolePads()) "column-sole-"+pad.id]},
        // Indicate the housing on its carrier before mounting the carriage.
        {member:"Ymodule/carriage",fastened:["outboard"]},
        // The servo goes on the cradle after the cradle is bolted to the seat (the floor bolts' heads are under the servo).
        {member: "spindle/motor", fastened: ["spindleSeat"]},
        // The Z scale head's screws are driven from the arm's inner face, where the eRob housing will stand: the head is screwed on before the actuator is fitted.
        {member: "swivel/housing", fastened: ["ZscaleArm"]}]};
    // Assemble the fixed shield/lid joints on the bench, lower that assembly
    // onto the bed and fit its foot screws through capped tool ports, then fit
    // the carriage. The casting otherwise obstructs the lid-nut driver's axis.
    options.fittedBefore = options.fittedBefore.concat([
      // XA: supports are tightened before the carriage and guard sheets go on.
      {member:"Xbridge",fastened:["Xfixed","Xfloating"]},
      {member:"XrearCover",fastened:["Xfixed","Xfloating"]},
      {member:"bed/casting",fastened:["XfrontCover","XrearCover"]},
      {member:"Xbridge",fastened:["XcoverFrontLeft","XcoverFrontRight","XcoverRearLeft","XcoverRearRight"]}
    ]);
    // The scale heads' M4 enter from the seat side (their slots run through the head), so each head is fitted last on its carrier, with the carrier's neighbours already
    // bolted on: the Y head after the outboard support's screws (its access runs where the head stands), the Z head before the guide blocks that stand beside its arm (a driver
    // from the wall side passes the blocks' row), and, once the carrier is a separate part on the bench, the X head with the carriage off the rails.
    options.fittedBefore = options.fittedBefore.concat([{member: "Ymodule/scaleHead", fastened: FlexsplineHobber.outboardMembers()},
      {member: "blockZLeft", fastened: ["ZscaleArm"]}, {member: "blockZLeftUpper", fastened: ["ZscaleArm"]}]);
    var order=options.fittedBefore;if(order==null)throw "Missing assembly order";
    for(rule in machine.yModule.fittingOrder()) {
      var mapped:machinekit.assembly.AssemblyIntegrity.FastenerFitOrder={member:"Ymodule/"+rule.member,fastened:[for(id in rule.fastened)"Ymodule/"+id]};
      var faces=rule.faces;if(faces!=null)mapped.faces=faces.copy();order.push(mapped);
    }
    for(member in ["scaleSeat","scaleHousing","riserLeft","riserRight"])order.push({member:"Ymodule/"+member,fastened:["Ymodule/base"],faces:["actuator-output"]});
    for(member in ["spindleSeat","outboardCarrier"])order.push({member:member,fastened:["Ymodule/carriage"],faces:["block-1-1","block-11","block1-1","block11","nut-carrier"]});
    order.push({member:"Ymodule/scaleHead",fastened:["Ymodule/carriage"],faces:["seat","outboard-carrier"]});
    if (!sweep) options.sweepStates = [];
    return options;
  }

  public static function integrity(machine:FlexsplineHobber, sweep:Bool, gap:Float = 2.0):Array<String> {
    var report = AssemblyIntegrity.check(machine, integrityOptions(machine, sweep, gap));
    for (item in report.diagnostics.items) if (item.code == "fasteners.edge-support") Sys.println(item.code + " " + item.message);
    return [for (item in report.errors()) item.code + " " + item.message];
  }

  /** Every declared service path is checked against the parts left installed.
   * Reuse main's completed strict check only for that same unchanged instance. */
  public static function service(machine:FlexsplineHobber, ?checkedIntegrity:Array<String>):Array<String> {
    var lines = [];
    var declared = [for (r in machine.serviceRemovals()) switch r {
      case ServiceRemoval(id, _, _, _, _): id;
      default: "";
    }];
    var errors = checkedIntegrity ?? [for (e in AssemblyIntegrity.service(machine).errors()) e.code + " " + e.message];
    for (id in declared) {
      var mine = [for (e in errors) if (e.indexOf("integrity.service." + id + " ") == 0) e];
      lines.push(mine.length == 0 ? 'service $id: clear' : 'service $id: BLOCKED ${mine.join("; ")}');
    }
    if (errors.length > 0) throw "Declared service removals are blocked: " + errors.join("; ");
    return lines;
  }

  /** Thickness of a cover end wall and of a carrier on the moving member, mm (the bench mill's `COVER_PLATE`). */
  static inline final COVER_PLATE = 14.0;
  /** Gap kept between a cover's plates and the supports and nut around them, mm (the running gap of the integrity check). */
  static inline final COVER_GAP = 2.0;

  /** The least free length, mm, at which a box cover of the lamella row folds and reaches `stroke`, with its opening `width` x `height`; also the half width of its fixed flange there. */
  static function leastCoverSpace(stroke:Float, width:Float, height:Float):{space:Float, folds:Int, half:Float} {
    var space = 20.0;
    while (space < 400) {
      try {
        var plan = new machinekit.cover.WayCoverPlan(machinekit.cover.WayCoverCatalog.get("BELLOWS-LAMELLA-8x8"), width, height, stroke, space, machinekit.cover.WayCoverPlan.boxCover());
        return {space: space, folds: plan.folds, half: plan.fullPlateHalf()};
      } catch (e:Dynamic) space += 1;
    }
    throw "no cover space found";
  }

  /**
   * Way covers on the X and Z screws (FRAME_PLAN, "Chip protection and service"): the free length each cover would have between its end wall (behind the support, a `COVER_PLATE` plate
   * and a gap) and its carrier on the nut block's side of the moving member, measured on the posed machine at the end stop where they are closest, and the least length a box
   * cover of the lamella row needs for that stroke round the bare screw (opening 40 x 50). Where the free length is short, the line says by how much: the geometry change that would make
   * the cover fit is that much more screw between the support and the nut's end stop (`HobberXLayout`, `HobberZLayout`). Returns one line per cover.
   */
  public static function coverFit(machine:FlexsplineHobber):Array<String> {
    var poses = machine.solvedPoses(), lines = [];
    PosedParts.scope(posed -> {
      function box(id:String) {
        var member = [for (c in machine.components()) if (c.id == id) c.component][0];
        var p = posed.posed(member, poses.get(id)), b = PosedParts.boxOf(p);
        p.close();
        return b;
      }
      function row(name:String, free:Float, stroke:Float, channelHalf:Float):Void {
        var need = leastCoverSpace(stroke, 40, 50);
        var verdict = free >= need.space ? 'fits' : 'SHORT by ${Math.round((need.space - free) * 10) / 10} mm';
        lines.push('cover $name: stroke ${stroke} mm, free ${Math.round(free * 10) / 10} mm, needs ${need.space} mm (${need.folds} folds), flange half width ${Math.round(need.half * 10) / 10} of ${channelHalf} mm: $verdict');
      }
      // X: work x is world y. The nut and its block move with the carriage by the joint value from its initial one.
      var x = machine.travel.filter(t -> t.id == "X")[0], z = machine.travel.filter(t -> t.id == "Z")[0];
      var nut = box("Xnut"), floating = box("Xfloating"), fixedSupport = box("Xfixed");
      var xStroke = x.upper - x.lower;
      var nutFront = nut.minY + (x.lower - x.initial), nutBack = nut.maxY + (x.upper - x.initial);
      row("X front (floating support to nut)", (nutFront - COVER_GAP - COVER_PLATE) - (floating.maxY + COVER_GAP + COVER_PLATE), xStroke, FoundationLayout.DRIVE_CHANNEL_WIDTH / 2);
      row("X rear (nut to fixed support)", (fixedSupport.minY - COVER_GAP - COVER_PLATE) - (nutBack + COVER_GAP + COVER_PLATE), xStroke, FoundationLayout.DRIVE_CHANNEL_WIDTH / 2);
      // Z: work z is world z. The nut moves with the saddle.
      var znut = box("Znut"), zfloating = box("Zfloating"), zfixed = box("Zfixed");
      var zStroke = z.upper - z.lower;
      var nutLow = znut.minZ + (z.lower - z.initial), nutHigh = znut.maxZ + (z.upper - z.initial);
      row("Z bottom (floating support to nut)", (nutLow - COVER_GAP - COVER_PLATE) - (zfloating.maxZ + COVER_GAP + COVER_PLATE), zStroke, 60);
      row("Z top (nut to fixed support)", (zfixed.minZ - COVER_GAP - COVER_PLATE) - (nutHigh + COVER_GAP + COVER_PLATE), zStroke, 60);
    });
    return lines;
  }

  /** Assumed guide pitch/yaw error over the travel, arcseconds (no measurement yet): sets the uncompensated Abbe error printed by `scales`. */
  static inline final GUIDE_ANGLE_ERROR_ARCSEC = 5.0;
  static inline final GUIDE_ANGLE_ERROR_RAD = GUIDE_ANGLE_ERROR_ARCSEC * Math.PI / 648000;

  /**
   * The members whose strict-integrity findings are open, with the reason for each. A finding on any other member fails the check, so the
   * frames, drives, guides, casting and anchors stay covered.
   * Empty: the last one, the spindle servo against the bed casting, is fixed by the relief in the casting (`HobberSpindleRelief`).
   */
  static final OPEN_MEMBERS:Array<String> = [];

  /** Integrity errors that do not touch an open member (see `OPEN_MEMBERS`). */
  public static function newErrors(machine:FlexsplineHobber, sweep:Bool):Array<String> {
    var found = integrity(machine, sweep);
    function open(error:String):Bool {
      for (member in OPEN_MEMBERS) if (error.indexOf(member) >= 0) return true;
      return false;
    }
    return [for (error in found) if (!open(error)) error];
  }

  /** The joint's positive direction in the work frame, per axis. */
  static function jointDirection(id:String):Array<Float> return id == "X" ? [1.0, 0, 0] : id == "Y" ? [0.0, -1, 0] : [0.0, 0, 1];

  /**
   * The sealed scales (FRAME_PLAN H4): the measuring length is the smallest standard one for the stroke, the head's centre stays inside
   * the measuring window over the whole stroke plus overtravel, and the Abbe offsets are reported. The Abbe offset of an axis is the
   * distance from the housing's line (the face toward the head, along the travel) to the hob centre, across the travel, in the work
   * frame at the initial joint values: two components, along the two work axes that are not the travel, and their length. Returns
   * one line per axis; throws when the head can leave the window or the scale is not where the machine says.
   */
  public static function scales(machine:FlexsplineHobber):Array<String> {
    var poses = machine.solvedPoses(), lines = [];
    var back = F.inverse(FlexsplineHobber.MACHINE_FROM_WORK);
    var tool = F.compose(poses.get("hob"), machine.hob.connector("centre").frame);
    var toolWork = F.transformPoint(back, tool.x, tool.y, tool.z);
    for (a in HobberScales.all()) {
      var travel = machine.travel.filter(t -> t.id == a.id)[0];
      var housing = poses.get(HobberScales.member(a.id)), head = poses.get(HobberScales.member(a.id,true));
      if (housing == null || head == null) throw 'axis ${a.id} has no scale members';
      var origin = F.transformPoint(back, housing.x, housing.y, housing.z);
      var worldAlong = F.transformVector(housing, 0, 0, 1), along = F.transformVector(back, worldAlong.x, worldAlong.y, worldAlong.z);
      var centre = F.transformPoint(back, head.x, head.y, head.z);
      // From the measuring line (the housing's `axis` connector, under the reading face's groove floor), not the hole line the housing is bolted on.
      var ax = F.compose(housing, a.housing.connector("axis").frame), mline = F.transformPoint(back, ax.x, ax.y, ax.z);
      var d = [toolWork.x - mline.x, toolWork.y - mline.y, toolWork.z - mline.z];
      var u = [along.x, along.y, along.z], along0 = d[0] * u[0] + d[1] * u[1] + d[2] * u[2];
      var across = [for (i in 0...3) d[i] - along0 * u[i]];
      var sense = jointDirection(a.id), sign = sense[0] * u[0] + sense[1] * u[1] + sense[2] * u[2];
      // Head position along the scale: at the initial joint value, then over the stroke with the guide overtravel (5 mm each end).
      var here = (centre.x - origin.x) * u[0] + (centre.y - origin.y) * u[1] + (centre.z - origin.z) * u[2];
      var from = here + sign * (travel.lower - 5 - travel.initial), to = here + sign * (travel.upper + 5 - travel.initial);
      var low = Math.min(from, to), high = Math.max(from, to), start = a.housing.startOffset(), end = start + a.housing.measuringLength;
      var names = ["x", "y", "z"], parts = [for (i in 0...3) if (Math.abs(u[i]) < 0.5) '${names[i]} ${Math.round(Math.abs(across[i]) * 10) / 10}'];
      var total = Math.sqrt(across[0] * across[0] + across[1] * across[1] + across[2] * across[2]);
      var line = '${a.id} scale ${a.housing.designation}: Abbe offset ${parts.join(", ")} mm (${Math.round(total * 10) / 10} mm), uncompensated Abbe error ${Math.round(total * GUIDE_ANGLE_ERROR_RAD * 1e4) / 10} um at an assumed ${GUIDE_ANGLE_ERROR_ARCSEC}" guide angular error (assumed; compensated by the tilt table, see the compensate-tilt procedures), head ' +
        'window ${Math.round(low * 10) / 10}..${Math.round(high * 10) / 10} of ${Math.round(start * 10) / 10}..${Math.round(end * 10) / 10}';
      lines.push(line);
      if (low < start - 1e-6 || high > end + 1e-6) throw "scale head leaves its measuring window: " + line;
    }
    return lines;
  }

  /**
   * The head on the A axis (FRAME_PLAN R3): the mass that turns with the actuator's output flange and its centre of mass, and what that
   * puts on the output bearing and the brake, at the middle position and at A +-15 deg. Work frame, N m. The actuator's bearing moment
   * rating and brake holding torque are not published (asked from ZeroErr); a missing one is reported as unknown, and an assumed one
   * is labelled. Gravity is -z; the output bearing's tilting moment is taken about the output face (the internal bearing location is unpublished; the reported demand is at the output face). The cutting force is 100 N at the hob, in the direction that loads the bearing most.
   * Returns one line per quantity; throws when the demand exceeds a rating that is known.
   */
  public static function swivelLoad(machine:FlexsplineHobber, ?worldPoses:Map<String, materia.assembly.AssemblyRecord.AssemblyFrame>):Array<String> {
    var def = machine.definition(), poses = worldPoses ?? machine.solvedPoses(), back = F.inverse(FlexsplineHobber.MACHINE_FROM_WORK);
    var turning = new Map<String, Bool>();
    turning.set("swivel/output", true);
    var changed = true;
    while (changed) {
      changed = false;
      for (j in def.joints) if (turning.exists(j.parent) && !turning.exists(j.child)) {
        turning.set(j.child, true);
        changed = true;
      }
    }
    var mass = 0.0, mx = 0.0, my = 0.0, mz = 0.0, count = 0;
    for (m in machine.components()) if (turning.exists(m.id)) {
      var props = m.component.massProperties(), at = F.transformPoint(F.compose(back, poses.get(m.id)), props.centreOfMass.x, props.centreOfMass.y, props.centreOfMass.z);
      mass += props.mass;
      mx += props.mass * at.x;
      my += props.mass * at.y;
      mz += props.mass * at.z;
      count++;
    }
    var outputFace = F.compose(back, poses.get("swivel/output"));
    var com = {x: mx / mass - outputFace.x, y: my / mass - outputFace.y, z: mz / mass - outputFace.z};
    var g = 9.80665, face = outputFace.x, overhang = -com.x / 1000;
    var tilt = mass * g * overhang;
    // Gravity torque about A: the centre of mass is (dy, dz) from the axis; turned by theta its lever on the vertical force is dy cos - dz sin.
    function gravityTorque(theta:Float):Float return mass * g * (com.y * Math.cos(theta) - com.z * Math.sin(theta)) / 1000;
    var worst = 0.0, at0 = gravityTorque(0), limit = HobberSwivelLayout.RANGE;
    for (i in -15...16) worst = Math.max(worst, Math.abs(gravityTorque(i * Math.PI / 180)));
    var tool = F.transformPoint(F.compose(back, F.compose(poses.get("hob"), machine.hob.connector("centre").frame)), 0, 0, 0);
    var hobOverhang = (face - tool.x) / 1000, hobLever = Math.sqrt(Math.pow(tool.y - outputFace.y, 2) + Math.pow(tool.z - outputFace.z, 2)) / 1000;
    var cutting = 100.0, cutTilt = cutting * hobOverhang, cutTorque = cutting * hobLever;
    var demandTorque = worst + cutTorque, bearingMoment = tilt + cutTilt;
    var row = machine.swivel.housing.ratioRow, moment = machine.swivel.housing.momentRating(), holding = machine.swivel.housing.brakeHoldingTorque();
    function r(v:Float, digits:Int = 2):Float return Math.round(v * Math.pow(10, digits)) / Math.pow(10, digits);
    var old = new machinekit.hobbing.SwivelHead(new machinekit.motion.TorqueMotor(new machinekit.component.ComponentValues().setNumber("height", 20)
      .setNumber("ratedSpeed", Math.PI / 12).setNumber("maxSpeed", Math.PI / 6).setNumber("torqueConstant", 2)
      .setNumber("backEmfConstant", 2 / Math.sqrt(3))), null, Math.PI / 12);
    var oldMass = 0.0;
    for (m in old.components()) oldMass += m.component.massProperties().mass;
    var newMass = 0.0;
    for (id in ["swivel/housing", "swivel/output", "Ymodule/base"]) for (m in machine.components()) if (m.id == id) newMass += m.component.massProperties().mass;
    var lines = [
      'A head: ${r(mass)} kg on the output flange (${count} members), centre of mass x ${r(com.x, 1)} y ${r(com.y, 1)} z ${r(com.z, 1)} mm relative to the output face; ' +
      'overhang ${r(overhang * 1000, 1)} mm in front of the output face',
      'A head change: swivel stack was ${r(oldMass)} kg (torque motor, crossed roller, clamp, housing, platen), now actuator ${r(machine.swivel.housing.massProperties().mass + machine.swivel.output.massProperties().mass)} kg ' +
      '+ profiled ground module base, ${r(newMass - machine.swivel.housing.massProperties().mass - machine.swivel.output.massProperties().mass)} kg',
      'A output bearing: gravity tilting moment ${r(tilt)} N m, cutting ${cutting} N at the hob (${r(hobOverhang * 1000, 1)} mm out) ${r(cutTilt)} N m, together ${r(bearingMoment)} N m; ' +
      (moment == null ? "moment rating: unknown (asked ZeroErr)" : 'moment rating ${moment} N m (assumed), margin ${r(moment / bearingMoment)}x'),
      'A torque about the axis: gravity ${r(at0)} N m at 0, ${r(worst)} N m at most over +-15 deg; cutting ${r(cutTorque)} N m (${cutting} N at a ${r(hobLever * 1000, 1)} mm lever); ' +
      'demand ${r(demandTorque)} N m against rated ${row.ratedTorque} / peak ${row.peakTorque} N m (${r(row.ratedTorque / demandTorque, 1)}x rated)',
      'A brake: must hold ${r(demandTorque)} N m with the power off; ' +
      (holding == null ? "holding torque: unknown (asked ZeroErr)" : 'holding torque ${holding} N m (ASSUMED, not vendor data), ${r(holding / demandTorque, 1)}x the demand')
    ];
    if (demandTorque > row.ratedTorque) throw "A head demand exceeds the actuator's rated torque: " + lines.join("; ");
    if (moment != null && bearingMoment > moment) throw "A head moment exceeds the output bearing rating: " + lines.join("; ");
    if (holding != null && demandTorque > holding) throw "A brake cannot hold the head: " + lines.join("; ");
    return lines;
  }


  /** Worst-case radial budget; all terms are radial, not TIR or diametral. */
  public static function outboard(machine:FlexsplineHobber):Array<String> {
    var poses = machine.solvedPoses(), arbor = machine.spindle.arbor;
    var inv = F.inverse(poses.get("spindle/arbor"));
    var radial = machinekit.hobbing.HobberOutboardLayout.INDICATED_RADIAL + machinekit.hobbing.HobberOutboardLayout.SEAT_RADIAL
      + machinekit.hobbing.HobberOutboardLayout.BEARING_RADIAL + machinekit.hobbing.HobberOutboardLayout.THERMAL_RADIAL;
    for (id in ["outboard", "outboardSleeve", "outboardBearingFront", "outboardBearingRear"]) {
      var p = F.compose(inv, poses.get(id)), axis = F.transformVector(p, 0, 0, 1);
      if (Math.sqrt(p.x * p.x + p.y * p.y) > 1e-7 || Math.abs(Math.abs(axis.z) - 1) > 1e-8) throw "outboard is not on spindle axis: " + id;
    }
    var front = F.compose(inv, poses.get("outboardBearingFront")), rear = F.compose(inv, poses.get("outboardBearingRear"));
    if (Math.abs(front.z - (arbor.parameterNumber("height") - arbor.parameterNumber("supportLength"))) > 1e-8
      || Math.abs(rear.z - front.z - 18) > 1e-8
      || F.transformVector(F.compose(F.inverse(poses.get("outboardBearingFront")), poses.get("outboardBearingRear")), 0, 0, 1).z > -0.999999) throw "outboard DB pair does not seat against journal shoulder";
    if (radial > machinekit.hobbing.HobberOutboardLayout.RADIAL_BUDGET + 1e-12) throw "outboard coaxiality budget exceeded";
    var thermal = machinekit.hobbing.HobberOutboardLayout.STEEL_EXPANSION * arbor.parameterNumber("height") * machinekit.hobbing.HobberOutboardLayout.TEMPERATURE_RISE;
    var floatMargin = 1 - thermal - machinekit.hobbing.HobberOutboardLayout.ASSEMBLY_AXIAL;
    var nutClearance = (30 - 22 / Math.cos(Math.PI / 6)) / 2;
    if (floatMargin <= 0 || nutClearance < 2) throw "outboard thermal travel or nut access insufficient";
    return ['Outboard: SKF 7002 CDGA/P4A DB pair, 15x32x9 each, class-A 20 N unmounted preload; installed radial budget 5 um '
      + '(2 indicated + 1 seats + 1.5 bearing + 0.5 thermal, all ASSUMED requirements); thermal growth ${Math.round(thermal * 10000) / 10000} mm at assumed 30 K, '
      + 'float +/-1 mm, remaining margin ${Math.round(floatMargin * 10000) / 10000} mm; nut/cap radial clearance ${Math.round(nutClearance * 1000) / 1000} mm.'];
  }

  /** BT30 retention/load interface. Material allowables and drawbar force are
   * explicit assumptions, not ratings inferred from the cartridge listing. */
  public static function bt30(machine:FlexsplineHobber):Array<String> {
    var keyBearing = 240.0 * 3 * 2 * 9.5 / 1000, keyRoot = 600.0 / Math.sqrt(3) * 3 * 5 * 9.5 / 1000;
    var capacity = Math.min(keyBearing, keyRoot), designTorque = 1.6, jamTorque = machine.spindle.motor.rating.peakTorque;
    if (capacity / designTorque < 1.5 || capacity / jamTorque < 1.2) throw "BT30 arbor face key fails torque reserve";
    if (machine.spindle.maxSpeed() * 60 / (2 * Math.PI) > machinekit.hobbing.Bt30SpindleData.MAX_RPM) throw "B exceeds bought BT30 speed limit";
    if (machine.hob.parameterNumber("faceKeyWidth") < 5.2 || machine.hob.parameterNumber("faceKeyDepth") < 2.2) throw "hob end slot does not clear arbor key";
    if (machine.billOfMaterials().quantity(machinekit.hobbing.Bt30SpindleData.DESIGNATION) != 1) throw "BT30 purchase is not unique";
    return ['BT30 arbor: face-key bearing capacity ${Math.round(keyBearing * 100) / 100} N m, root ${Math.round(keyRoot * 100) / 100} N m '
      + '(240 MPa bearing, 600/sqrt(3) MPa shear ASSUMED), factors ${Math.round(capacity / designTorque * 10) / 10} design / ${Math.round(capacity / jamTorque * 100) / 100} motor peak; '
      + 'MAS taper-only location, M12 pull stud; bought gripper retention force and stud-head outline require confirmation.'];
  }

  /**
   * The B spindle belt drive (head redesign): the belt's pull on the cartridge's rear bearing, and the first mode of the servo on its
   * cradle against the hob's tooth-pass frequency over the hob speed range. ESTIMATES from the model's masses and sections with stated joint
   * assumptions (preloaded, dowelled bolted joints: axial stiffness 3 x the bolt's); they say where the mode is, not that the part is released.
   * Lines: belt tangential force and pretension (slack side kept at a quarter of the peak force, an assumed rule), the radial pull and
   * what it does to the 6205 tail bearing (catalogue ratings assumed from the maker), then the servo group's mass and centre, each stiffness
   * and mode, and the tooth-pass band (hob rpm x gashes / 60, from 100 m/min to the cartridge's top speed) with +-20 %. Throws when the first
   * mode lies inside that band.
   */
  public static function spindleDrive(machine:FlexsplineHobber):{lines:Array<String>, modes:Array<{name:String, f:Float}>, modalMass:Float} {
    var spindle = machine.spindle, rating = spindle.motor.rating, poses = machine.solvedPoses(), back = F.inverse(FlexsplineHobber.MACHINE_FROM_WORK);
    function r(v:Float, digits:Int = 2):Float return Math.round(v * Math.pow(10, digits)) / Math.pow(10, digits);
    var lines = [];
    // Belt pull on the cartridge pulley.
    var rm = spindle.motorPulley.pitchDiameter / 2000, rs = spindle.spindlePulley.pitchDiameter / 2000, c = HobSpindle.centres() / 1000;
    var fRated = rating.ratedTorque / rm, fPeak = rating.peakTorque / rm, t0 = HobSpindle.pretension(rating.peakTorque, spindle.motorPulley.pitchDiameter);
    var radial = 2 * t0 * Math.cos(Math.asin((rs - rm) / c));
    lines.push('B belt: HTD5M ${spindle.motorPulley.teeth}T/${spindle.spindlePulley.teeth}T, ${r(spindle.belt.length, 1)} mm belt on ${r(c * 1000, 1)} mm centres, motor turns ${r(spindle.ratio, 3)}x the hob; tangential force ${r(fRated)} N rated / ${r(fPeak)} N motor peak, '
      + 'pretension ${r(t0)} N per strand (assumed rule), tight side ${r(t0 + fPeak / 2)} N at peak; '
      + 'free span rings at ${r(HobSpindle.spanFrequency(t0), 0)} Hz (${HobSpindle.BELT_MASS_PER_M} kg/m assumed), tension travel ${r(HobSpindle.tensioningTravel(t0, spindle.belt.length), 2)} mm; slots +-${HobSpindle.TENSION_TRAVEL} mm');
    lines.push('Purchased BT30 cartridge: ${r(radial)} N radial belt pull; require ${Math.ceil(radial * 1.5 / 50) * 50} N at the pulley. Allowable load unpublished (ASSUMED acceptance requirement); no life calculation from invented internal bearing positions/ratings.');
    var nTop = spindle.maxSpeed() * 60 / (2 * Math.PI);
    // The servo group on its cradle: members without the cradle (the FE carries the cradle's own mass in its mesh), and a third of the cradle for the load lines.
    var ids = ["spindle/motor", "spindle/motorShaft", "spindle/motorPulley", "spindle/motorEncoder"], mass = 0.0, mx = 0.0, my = 0.0, mz = 0.0, cradle = 0.0, servo:Array<{m:Float, x:Float, y:Float, z:Float}> = [];
    for (m in machine.components()) {
      var weight = ids.indexOf(m.id) >= 0 ? 1.0 : m.id == "motorCradle" ? 1.0 / 3 : 0.0;
      if (weight == 0) continue;
      var props = m.component.massProperties(), at = F.transformPoint(F.compose(back, poses.get(m.id)), props.centreOfMass.x, props.centreOfMass.y, props.centreOfMass.z);
      mass += weight * props.mass;
      mx += weight * props.mass * at.x;
      my += weight * props.mass * at.y;
      mz += weight * props.mass * (at.z - HobberZLayout.SEAT_Z);
      if (m.id == "motorCradle") cradle = props.mass;
      else servo.push({m: props.mass, x: at.x, y: at.y, z: at.z - HobberZLayout.SEAT_Z});
    }
    var sec = HobberSaddle.cradleSection(), dims = HobberSaddle.cradleDimensions(), rest = HobberSaddle.cradleRest(), faceY = HobberSaddle.cradleFaceY();
    var com = {x: mx / mass, y: my / mass, z: mz / mass}, ahead = faceY - com.y, height = com.z - sec.floorTop;
    var motor = machine.spindle.motor, face = motor.rating.bodyDiameter, bodyLength = motor.rating.bodyLength;
    // ---- Hand model of the servo group on the cradle, validated against the C3D10 FE (HobberFeResults, fem/): two coupled-pair modes.
    //  rocking  the wall bends as a cantilever (the section of `cradleSlabs` at each height: wall at full thickness, the rib at a fitted participation) from the foot's top to the upper
    //           flange bolt row, where the servo is attached; the base turns about x (floor joint, seat: FE-derived rotation stiffness); the rest block adds a vertical spring ahead of the wall.
    //           DOFs: y displacement and rotation about x of the attachment point, 2 x 2 generalised eigenproblem with the servo's mass and inertia.
    //  lateral  the same cantilever bending in x, the base rolling about y, the rest block's width as a roll spring.
    var e = 210000.0, g = e / 2.6, kappa = 5.0 / 6.0, base = sec.floorTop, flange = machine.spindle.motor.mounting();
    if (flange == null) flange = machinekit.motion.ServoMotor.assumedMounting(motor.rating.designation);
    var zq = HobSpindle.motorHeight(), zTop = zq;
    for (h in flange.holes) zTop = Math.max(zTop, zq + h.y);
    var eta = HobberFeResults.RIB_PARTICIPATION, hq = zTop - base, steps = Math.round(hq);
    function props(z:Float, withRib:Bool, axis:Int) {
      var area = 0.0, sx = 0.0, slabs = HobberSaddle.cradleSlabs(z, withRib);
      for (sl in slabs) {
        var a = (sl.x1 - sl.x0) * (sl.y1 - sl.y0);
        area += a;
        sx += a * (axis == 0 ? 0.5 * (sl.y0 + sl.y1) : 0.5 * (sl.x0 + sl.x1));
      }
      var c = sx / area, inertia = 0.0;
      for (sl in slabs) {
        var a = (sl.x1 - sl.x0) * (sl.y1 - sl.y0), d = (axis == 0 ? 0.5 * (sl.y0 + sl.y1) : 0.5 * (sl.x0 + sl.x1)) - c;
        inertia += (axis == 0 ? (sl.x1 - sl.x0) * Math.pow(sl.y1 - sl.y0, 3) : (sl.y1 - sl.y0) * Math.pow(sl.x1 - sl.x0, 3)) / 12 + a * d * d;
      }
      return {area: area, inertia: inertia};
    }
    // Flexibility of the wall cantilever at the attachment height: [[v, v-theta], [v-theta, theta]] per unit load / moment (1 mm slices).
    function wallFlex(axis:Int) {
      var f11 = 0.0, f12 = 0.0, f22 = 0.0;
      for (i in 0...steps) {
        var s = (i + 0.5) * hq / steps, z = base + s, with = props(z, true, axis), without = props(z, false, axis);
        var a = without.area + eta * (with.area - without.area), inertia = without.inertia + eta * (with.inertia - without.inertia), arm = hq - s, ds = hq / steps;
        f11 += (arm * arm / (e * inertia) + 1 / (kappa * g * a)) * ds;
        f12 += arm / (e * inertia) * ds;
        f22 += ds / (e * inertia);
      }
      return [f11, f12, f22];
    }
    // The servo's inertias about its centre of mass (the motor body as a box, the members as points), tonne mm2.
    var ixx = 0.0, iyy = 0.0, mServo = 0.0, cx = 0.0, cy = 0.0, cz = 0.0;
    for (m in servo) {
      mServo += m.m;
      cx += m.m * m.x;
      cy += m.m * m.y;
      cz += m.m * m.z;
    }
    cx /= mServo;
    cy /= mServo;
    cz /= mServo;
    for (m in servo) {
      ixx += m.m * ((m.y - cy) * (m.y - cy) + (m.z - cz) * (m.z - cz));
      iyy += m.m * ((m.x - cx) * (m.x - cx) + (m.z - cz) * (m.z - cz));
    }
    var body = servo.filter(m -> m.m > 2.0)[0];
    ixx += body.m / 12 * (face * face + bodyLength * bodyLength);
    iyy += body.m / 12 * (face * face + face * face);
    var tonne = 1e-3, mt = mServo * tonne, ry = cy - faceY, rz = cz - zTop;
    function eigen2(k:Array<Float>, m:Array<Float>):Float {
      // det(K - l M) = 0 for symmetric 2x2: a l^2 + b l + c = 0
      var a = m[0] * m[2] - m[1] * m[1], b = -(k[0] * m[2] + k[2] * m[0] - 2 * k[1] * m[1]), c = k[0] * k[2] - k[1] * k[1];
      var l = (-b - Math.sqrt(Math.max(0.0, b * b - 4 * a * c))) / (2 * a);
      return Math.sqrt(Math.max(0.0, l)) / (2 * Math.PI);
    }
    function invert(f:Array<Float>):Array<Float> {
      var d = f[0] * f[2] - f[1] * f[1];
      return [f[2] / d, -f[1] / d, f[0] / d];
    }
    var packSeries = 1 / (1 / HobberFeResults.REST_PACK_N_PER_MM + 1 / HobberFeResults.SEAT_UNDER_REST_N_PER_MM);
    // rocking: u_y(cg) = v - theta rz, u_z(cg) = theta ry
    var fr = wallFlex(0), kb = HobberFeResults.BASE_ROCKING_N_MM_PER_RAD;
    var kRock = invert([fr[0] + hq * hq / kb, fr[1] + hq / kb, fr[2] + 1 / kb]);
    var restArm = faceY - 0.5 * (rest.y0 + rest.y1), kRest = packSeries * restArm * restArm;
    kRock[2] += kRest;
    var mRock = [mt, -mt * rz, mt * (rz * rz + ry * ry) + ixx * tonne];
    var fRock = eigen2(kRock, mRock);
    // lateral: u_x(cg) = u + phi rz
    var fl = wallFlex(1), kbl = HobberFeResults.BASE_ROLL_N_MM_PER_RAD;
    var kLat = invert([fl[0] + hq * hq / kbl, fl[1] + hq / kbl, fl[2] + 1 / kbl]);
    var restWidth = rest.x1 - rest.x0, restOffset = 0.5 * (rest.x0 + rest.x1) - HobberSaddle.cradleMotorX();
    kLat[2] += packSeries * (restWidth * restWidth / 12 + restOffset * restOffset);
    var mLat = [mt, mt * rz, mt * rz * rz + iyy * tonne];
    var fLat = eigen2(kLat, mLat);
    var modes = [{name: "rocking about x (wall, base, rest block)", f: fRock}, {name: "lateral roll about y (wall, base, rest block width)", f: fLat}];
    var first = modes[0];
    for (m in modes) if (m.f < first.f) first = m;
    function hz(k:Float):Float return Math.sqrt(k * 1000 / mass) / (2 * Math.PI);
    // Tooth-pass band: hobbing runs from 100 to 300 m/min (the servo's rated speed caps it, below); the cartridge itself turns up to its own limit.
    var gashes = machine.hob.parameterCount("gashes"), diameter = machine.hob.design.outsideDiameter;
    var nLow = 100000 / (Math.PI * diameter), hobRpmTop = 3 * nLow, fLow = nLow * gashes / 60, fOp = hobRpmTop * gashes / 60, fHigh = nTop * gashes / 60;
    var lo = 0.8 * fLow, opHi = 1.2 * fOp, hi = 1.2 * fHigh;
    var feSig = HobberFeResults.CRADLE_DESIGN, signature = HobberSaddle.cradleSignature();
    var fePre = Math.min(HobberFeResults.CRADLE_PRELOADED_HZ[0], HobberFeResults.CRADLE_PRELOADED_HZ[1]), feUnpre = HobberFeResults.CRADLE_UNPRELOADED_HZ[0], feOpen = HobberFeResults.CRADLE_UNPRELOADED_HZ[1];
    lines.push('B servo group: ${r(mass)} kg (motor, shaft, pulley, encoder and a third of the ${r(cradle)} kg cradle), centre ${r(ahead, 1)} mm ahead of the flange, ${r(height, 1)} mm above the floor; '
      + 'hand-model servo ${r(mServo)} kg, inertia ${Math.round(ixx)} kg mm2 about x / ${Math.round(iyy)} about y, attachment at the upper flange bolt row, ${r(hq, 1)} mm above the foot; rest block ${r(restWidth, 0)} x ${r(rest.y1 - rest.y0, 0)} mm, '
      + '${r(restArm, 0)} mm ahead of the wall, spring ${Math.round(packSeries / 1000)} N/um (shim pack ${Math.round(HobberFeResults.REST_PACK_N_PER_MM / 1000)} in series with the seat ${Math.round(HobberFeResults.SEAT_UNDER_REST_N_PER_MM / 1000)} N/um)');
    lines.push('B servo first-mode hand model (calibrated to the FE of 15 cradle variants): ' + [for (m in modes) m.name + " " + Math.round(m.f) + " Hz"].join("; ") + '; first ${Math.round(first.f)} Hz '
      + 'against FE (C3D10, flange on its four bolts: ${HobberFeResults.CRADLE_PRELOADED_HZ.map(Math.round)} Hz preloaded (joint closed / open), ${HobberFeResults.CRADLE_UNPRELOADED_HZ.map(Math.round)} Hz unpreloaded (contact closed / open), '
      + '${Math.round(HobberFeResults.CRADLE_BONDED_HZ)} Hz bonded, ${HobberFeResults.CRADLE_FACE_FLANGE_PRELOADED_HZ.map(Math.round)} Hz with the whole flange coupled; ${HobberFeResults.CRADLE_NO_REST_PRELOADED_HZ.map(Math.round)} Hz without the rest block, '
      + '${HobberFeResults.CRADLE_JACK_SCREW_PRELOADED_HZ.map(Math.round)} Hz with jack screws instead of the shim pack): hand / FE preloaded (lowest) = ${r(first.f / fePre, 2)}');
    if (signature != feSig) throw "the cradle's geometry changed since the FE reference was run (fem/run_fem.sh): FE \"" + feSig + "\" now \"" + signature + "\"";
    if (Math.abs(first.f / fePre - 1) > 0.25) throw 'the cradle hand model (${Math.round(first.f)} Hz) and the FE (${Math.round(fePre)} Hz) disagree by more than 25 %: refit HobberFeResults constants or fix the model';
    lines.push('B servo vs the tooth-pass band (hob Ø${r(diameter, 1)}, ${gashes} gashes): operating range 100..300 m/min = ${Math.round(nLow)}..${Math.round(hobRpmTop)} rpm (the servo\'s ${Math.round(rating.ratedSpeed * 60 / (2 * Math.PI))} rpm rated speed caps it) = tooth-pass ${Math.round(fLow)}..${Math.round(fOp)} Hz, '
      + 'avoid ${Math.round(lo)}..${Math.round(opHi)} Hz (+-20 %); FE first mode ${Math.round(fePre)} Hz preloaded / ${Math.round(feUnpre)} Hz unpreloaded (contact closed) is ${r(fePre / opHi, 2)} x / ${r(feUnpre / opHi, 2)} x the top of that band; '
      + 'with the joint open (no preload, contact lost) ${Math.round(feOpen)} Hz, ${feOpen > opHi ? "still above" : "INSIDE"} the band, excited at ${Math.round(feOpen * 60 / gashes)} rpm (${Math.round(feOpen * 60 / gashes * Math.PI * diameter / 1000)} m/min): not a gate (the procedure torques the floor bolts), the reason to re-check their torque; '
      + 'the mode is excited by the tooth-pass at ${Math.round(fePre * 60 / gashes)} rpm (${Math.round(fePre * 60 / gashes * Math.PI * diameter / 1000)} m/min) and its +-20 % window begins at ${Math.round(0.8 * fePre * 60 / gashes)} rpm, ${r(0.8 * fePre * 60 / gashes / hobRpmTop, 2)} x the 300 m/min limit; '
      + 'the cartridge turns up to ${Math.round(nTop)} rpm (tooth-pass ${Math.round(fHigh)} Hz, avoid to ${Math.round(hi)} Hz): hobbing above ${Math.round(0.8 * fePre * 60 / gashes)} rpm would cross the mode');
    if (first.f <= opHi || fePre <= opHi || feUnpre <= opHi) throw "the servo's first mode lies within 20 % of the operating tooth-pass band: " + lines.join("; ");
    var margin = 1.3;
    lines.push('B servo first-mode margin against the cartridge-limit band: FE ${r(fePre / hi, 2)} x the top of the avoid band (${Math.round(hi)} Hz), required ${margin} x (${Math.round(margin * hi)} Hz): '
      + (fePre >= margin * hi ? "met" : "NOT MET (open decision: limit hobbing to the servo's rated speed, as the speed check already does, or stiffen the seat; see FRAME_PLAN 2026-11-06)"));
    // Cradle mass and the lightening.
    lines.push('Motor cradle: ${r(cradle, 3)} kg (1.321 kg before the rest block and the floor extension; 2.036 kg with a 15 mm wall, 12 mm floor and solid rib); wall ${dims.wall}, floor ${dims.floor} (${dims.floorThin} ahead of y 195, foot ${dims.foot} under the wall), windowed rib, jack lug ${dims.lug} mm above the foot, rest block top ${r(rest.top, 1)} mm');
    // The servo's speed and torque over the hob's speed range (1:1 belt).
    var ratio = spindle.ratio, motorRatedRpm = rating.ratedSpeed * 60 / (2 * Math.PI), motorRpmTop = hobRpmTop * ratio;
    var cuttingBound = 100.0, efficiency = 0.97, reserve = 1.2;
    var cuttingTorque = cuttingBound * diameter / 2000, motorTorque = cuttingTorque / ratio / efficiency, motorPower = motorTorque * motorRpmTop * 2 * Math.PI / 60;
    lines.push('B speed and torque over the hob range (Ø${r(diameter, 1)}, 100..300 m/min = ${Math.round(nLow)}..${Math.round(hobRpmTop)} rpm): servo ${Math.round(nLow * ratio)}..${Math.round(motorRpmTop)} rpm against ${Math.round(motorRatedRpm)} rpm rated (${r(100 * motorRpmTop / motorRatedRpm, 1)} %), '
      + 'the cartridge limit ${Math.round(nTop)} rpm is above the rated speed (torque falls there: not for continuous cutting); cutting torque bound ${r(cuttingTorque, 2)} N m at the hob '
      + '(tangential force ${cuttingBound} N: ASSUMED upper bound, above the repo\'s load studies: 46.7 N peak on the reference part, 72.1 N on the m0.3 z40 1 mm smoke; at 16 mm radius those are 0.75 and 1.15 N m) = ${r(motorTorque, 2)} N m at the servo through the belt '
      + '(${efficiency} efficiency, ratio ${r(ratio, 2)}); servo rated ${r(rating.ratedTorque, 2)} N m (${r(rating.ratedTorque / motorTorque, 2)} x, ${reserve} x required), peak ${r(rating.peakTorque, 2)} N m; power ${Math.round(motorPower)} W against ${Math.round(rating.ratedTorque * rating.ratedSpeed)} W');
    if (motorRpmTop > motorRatedRpm) throw "the servo would run above its rated speed at the top of the hob speed range: " + lines.join("; ");
    if (motorTorque * reserve > rating.ratedTorque || motorPower > rating.ratedTorque * rating.ratedSpeed) throw "the servo's rated torque or power does not cover the hob's cutting load: " + lines.join("; ");
    // The flange bolts hold the servo against the belt (slots in the wall): friction of four M6 8.8 at 60 % of proof (assumed), steel on steel.
    var preload = 0.6 * 580 * 20.1, grip = 4 * preload * 0.12, pull = 2 * t0;
    lines.push('B servo flange grip: 4 M6 8.8 at 60 % of proof (${Math.round(preload)} N each, assumed), friction 0.12 steel on steel (assumed) holds ${Math.round(grip)} N along the slots against the belt\'s net pull ${Math.round(pull)} N (2 x pretension): ${r(grip / pull, 1)} x; '
      + 'the jack screw also bears on the flange from below against that pull');
    if (grip < 3 * pull) throw "the flange bolts' friction does not hold the belt pull with a factor of 3: " + lines.join("; ");
    return {lines: lines, modes: modes, modalMass: mass};
  }

  /**
   * The Z saddle plate's first modes (C3D10 FE, `HobberFeResults`, the head as a rigid mass on the eRob seat: upper bounds, the eRob's bearing and housing compliance are not in the model)
   * against the tooth-pass of the hob at the cutting speeds used. The tooth-pass f = n gashes / 60 reaches a mode of frequency m at the hob speed n = 60 m / gashes; the mode is "hit" over
   * +-20 % of that speed. Lines only (no throw): the 13 mm plate is fixed by the swivel seat (x 135) and the block faces (x 148), so a stiffer plate is a redesign, not a parameter.
   */
  public static function zSaddleModes(machine:FlexsplineHobber):Array<String> {
    function r(v:Float, d:Int = 0):Float return Math.round(v * Math.pow(10, d)) / Math.pow(10, d);
    var gashes = machine.hob.parameterCount("gashes"), diameter = machine.hob.design.outsideDiameter, perMetre = 1000 / (Math.PI * diameter);
    var nLow = 100 * perMetre, nTop = 300 * perMetre;
    var lines = ['Z saddle plate modes (FE, head rigid on the eRob seat, upper bounds; supports at the block bolts / whole block footprints): ' + [for (i in 0...3) '${r(HobberFeResults.SADDLE_MODES_BOLTS_HZ[i])}..${r(HobberFeResults.SADDLE_MODES_BLOCKS_HZ[i])}'].join(", ")
      + ' Hz (head tilting about y on the plate, then two translation-yaw modes); seat tilt stiffness about y ${r(HobberFeResults.SADDLE_TILT_KNM_PER_RAD[0])}..${r(HobberFeResults.SADDLE_TILT_KNM_PER_RAD[1])} kN m/rad, '
      + 'hob tip ${HobberFeResults.SADDLE_TIP_UM_PER_100N} um per 100 N of cutting force'];
    var hit = [];
    for (i in 0...3) {
      var f = HobberFeResults.SADDLE_MODES_BOLTS_HZ[i], n = 60 * f / gashes, v = n / perMetre, lowV = 0.8 * v, highV = 1.2 * v;
      var inside = !(highV < 100 || lowV > 300);
      lines.push('  mode ${i + 1} (${r(f)} Hz): tooth-pass reaches it at ${r(n)} rpm = ${r(v)} m/min (hob Ø${r(diameter, 1)}, ${gashes} gashes); window ${r(lowV)}..${r(highV)} m/min against the used range 100..300 m/min (${r(nLow)}..${r(nTop)} rpm): '
        + (inside ? "INSIDE the range" : highV < 100 ? "below the range" : "above the range"));
      if (inside) hit.push(i + 1);
    }
    lines.push('Z saddle conclusion: ' + (hit.length == 0 ? "no saddle mode lies in the used speed range." : 'saddle mode ${hit.join(", ")} lie${hit.length == 1 ? "s" : ""} in the 100..300 m/min range as an upper bound (the head on the eRob bearing is softer, so the true modes are lower and may sit '
      + 'in other windows). Proposal, not implemented: avoid the speed windows above when finishing; or stiffen the plate where the seat tilts (box the plate with ribs along the block rows on the swivel side, a steel stiffener under the eRob pocket) '
      + 'and re-run fem/saddle_fem.py; a thicker plate needs the swivel seat or the block planes moved (decision).'));
    return lines;
  }

  /** The members that move with the Z joint (the Z saddle and everything carried on it), and their mass and centre of mass in the work frame. */
  static function zMoving(machine:FlexsplineHobber):{mass:Float, x:Float, y:Float, z:Float, ids:Array<String>} {
    var def = machine.definition(), poses = machine.solvedPoses(), back = F.inverse(FlexsplineHobber.MACHINE_FROM_WORK);
    var moving = new Map<String, Bool>();
    moving.set("blockZLeft", true);
    var changed = true;
    while (changed) {
      changed = false;
      for (j in def.joints) if (moving.exists(j.parent) && !moving.exists(j.child)) {
        moving.set(j.child, true);
        changed = true;
      }
    }
    var mass = 0.0, mx = 0.0, my = 0.0, mz = 0.0, ids = [];
    for (m in machine.components()) if (moving.exists(m.id)) {
      var props = m.component.massProperties(), at = F.transformPoint(F.compose(back, poses.get(m.id)), props.centreOfMass.x, props.centreOfMass.y, props.centreOfMass.z);
      mass += props.mass;
      mx += props.mass * at.x;
      my += props.mass * at.y;
      mz += props.mass * at.z;
      ids.push(m.id);
    }
    return {mass: mass, x: mx / mass, y: my / mass, z: mz / mass, ids: ids};
  }

  /**
   * The Z screw stands off the rail pair's middle (`HobberZOffset`), so its thrust and the moving mass's weight, which acts at the mass's
   * centre of mass, are `e` apart across the machine and form a moment about the rail normal (x), the blocks' yaw. The sign of the offset must be
   * the side of the centre of mass. The moment is shared by the four blocks and set against the block row's rated yaw moment (static: the
   * weight, held by the brake; dynamic: the sizing's peak thrust, the rated moment scaled by C/C0 because the catalogue lists static moments
   * only). Returns the lines; throws when the sign is wrong or a block is over its rating.
   */
  public static function zOffsetMoments(machine:FlexsplineHobber):Array<String> {
    var z = zMoving(machine), g = 9.80665, offset = HobberZLayout.axisY(), result = machinekit.hobbing.HobberZOffset.result();
    if (offset * (z.y - 0) <= 0) throw 'Z screw at y ${offset} is on the opposite side of the moving mass centre (y ${z.y})';
    var report:Dynamic = FlexsplineHobberLayoutReport.measure(machine), mass = z.mass, q = 0.005 / (2 * Math.PI);
    var nut:machinekit.motion.BallNut = cast machine.components().filter(m -> m.id == "Znut")[0].component;
    var motor:machinekit.motion.ServoMotor = cast machine.components().filter(m -> m.id == "Zmotor")[0].component;
    var demand = machinekit.sizing.LinearDriveSizing.demand({
      massKg: mass, motors: 1, metresPerRadian: q, efficiency: nut.efficiency(), rotaryInertiaKgM2: 0.4e-4, acceleration: 0.5, velocity: 0.2, strokeM: 0.2,
      cuttingForceN: 100, dragForceN: 100, gravityForceN: mass * g, rotaryDragNm: 0.1, reserve: 1.5
    }, motor.rating);
    var peakThrust = demand.peakNm * nut.efficiency() / q, weight = mass * g;
    var arm = Math.abs(offset - z.y) / 1000, mirrored = Math.abs(-offset - z.y) / 1000;
    var block:machinekit.motion.LinearRailBlock = cast machine.components().filter(m -> m.id == "blockZLeft")[0].component;
    var rating:Null<machinekit.motion.LinearRailProfile.LinearRailRating> = null;
    for (item in block.spec.ratings) if (item.yawMoment != null && rating == null) rating = item;
    if (rating == null) throw "Z block row " + block.spec.designation + " has no rated yaw moment";
    var blocks = 4, staticMoment = weight * arm / blocks, dynamicMoment = peakThrust * arm / blocks;
    var yawBase:Float = rating.yawMoment ?? 0.0, yawStatic:Float = yawBase, yawDynamic:Float = yawBase * rating.dynamicLoad / rating.staticLoad;
    function r(v:Float, d:Int = 2):Float return Math.round(v * Math.pow(10, d)) / Math.pow(10, d);
    var lines = [
      'Z screw offset: y ${offset} (minimum ${result.minimum} mm set by ${result.governing}; casting allows at most ${result.maximum} mm); Z moving mass ${r(mass)} kg, centre of mass y ${r(z.y, 1)}',
      'Z offset moments on ${blocks} ${block.spec.designation} blocks (${rating.source}): arm ${r(arm * 1000, 1)} mm (on the other side it would be ${r(mirrored * 1000, 1)} mm); ' +
      'static weight ${r(weight)} N: ${r(staticMoment, 3)} N m per block against yaw ${yawStatic} N m (safety factor ${r(yawStatic / Math.max(staticMoment, 1e-9), 1)}); ' +
      'peak thrust ${r(peakThrust)} N: ${r(dynamicMoment, 3)} N m per block against ${r(yawDynamic, 1)} N m (dynamic, scaled by C/C0; safety factor ${r(yawDynamic / Math.max(dynamicMoment, 1e-9), 1)}); roll about the rail axis: none (thrust and weight are along it)'
    ];
    if (staticMoment > yawStatic || dynamicMoment > yawDynamic) throw "Z blocks exceed their yaw rating: " + lines.join("; ");
    return lines;
  }

  /**
   * The spindle seat against the cartridge (FRAME_PLAN R2): in the housing's own z (tool face at 180, encoder stator at 0..-14), the seat's
   * bore must lie inside the assumed cartridge clamp land (`SpindleHousing.CLAMP_FROM..CLAMP_TO`) and carry its clamp screws inside that
   * length. Internal bearing seats are part of the purchase, not an invented housing design. Returns the line; throws otherwise.
   */
  public static function spindleSeat(machine:FlexsplineHobber):Array<String> {
    var poses = machine.solvedPoses(), housing = poses.get("spindle/housing"), back = F.inverse(FlexsplineHobber.MACHINE_FROM_WORK);
    var seat = HobberSaddle.spindleSeatRange(), toHousing = F.inverse(F.compose(back, housing));
    function zOf(y:Float):Float return F.transformPoint(toHousing, 0, y, HobberZLayout.SEAT_Z).z;
    var a = zOf(seat.y0), b = zOf(seat.y1), from = Math.min(a, b), to = Math.max(a, b);
    var screws = [for (y in seat.clampYs) zOf(y)];
    var lines = ['Spindle seat grips housing z ${Math.round(from * 10) / 10}..${Math.round(to * 10) / 10} (${Math.round((to - from) * 10) / 10} mm) of the clamping length ${SpindleHousing.CLAMP_FROM}..${SpindleHousing.CLAMP_TO} (assumed); ' +
      'BT30 cartridge; clamp screws at z ${screws.join(", ")}; rear encoder and front flange at 170..180 stay outside the clamp (axial envelope ASSUMED)'];
    if (from < SpindleHousing.CLAMP_FROM - 1e-6 || to > SpindleHousing.CLAMP_TO + 1e-6) throw "spindle seat leaves the housing's clamping length: " + lines.join("; ");
    if (to - from < 120) throw "spindle clamp engagement below the assumed 120 mm requirement";
    for (z in screws) if (z < from || z > to) throw "spindle seat clamp screw outside the seat: " + lines.join("; ");
    return lines;
  }

  /**
   * The relief in the bed casting under the spindle servo (`HobberSpindleRelief`). Checks (1) the relief's pose chain against the solved
   * machine at three joint states (servo frame within 1e-6 mm and radians), (2) the servo, with its connector envelopes, keeps the running
   * gap from the finished casting at the extreme poses including the overtravel on every axis, and (3) the casting keeps `WALL` of solid
   * material under the pocket's floor. Returns the report lines; throws with the numbers otherwise.
   */
  public static function spindleRelief(machine:FlexsplineHobber):Array<String> {
    var relief = HobberSpindleRelief.result(), lines = [];
    function r(v:Float, d:Int = 2):Float return Math.round(v * Math.pow(10, d)) / Math.pow(10, d);
    var model = new cadkit.modeling.AssemblyModel("mm");
    machine.addTo(model, "");
    var state = model.initialState();
    var toMachine = FlexsplineHobber.MACHINE_FROM_WORK, zero = HobberSpindleRelief.motorZeroPose(HobSpindle.MOTOR_CLOCKING);
    function analytic(x:Float, y:Float, z:Float, a:Float) return F.compose(toMachine, F.compose(HobberSpindleRelief.motion(x, y, z, a), zero));
    // (1) the chain.
    var xl = machine.travel.filter(t -> t.id == "X")[0].lower, xu = machine.travel.filter(t -> t.id == "X")[0].upper;
    for (s in [{x: xl, y: 0.0, z: 0.0, a: 0.0}, {x: xl, y: -10.0, z: -10.0, a: 15 * Math.PI / 180}, {x: (xl + xu) / 2, y: 7.0, z: 120.0, a: -9 * Math.PI / 180}]) {
      state.setJoint("X", s.x);
      state.setJoint(FlexsplineHobber.Y, s.y);
      state.setJoint("Z", s.z);
      state.setJoint("swivel/A", s.a);
      state.forwardKinematics();
      var actual = state.worldPose("spindle/motor"), expected = analytic(s.x, s.y, s.z, s.a), worst = 0.0;
      worst = Math.max(worst, Math.max(Math.abs(actual.x - expected.x), Math.max(Math.abs(actual.y - expected.y), Math.abs(actual.z - expected.z))));
      var ma = F.toRotationMatrix(actual), me = F.toRotationMatrix(expected);
      for (i in 0...9) worst = Math.max(worst, Math.abs(ma[i] - me[i]));
      if (worst > 1e-6) throw 'spindle relief pose chain differs from the machine by ${worst} at X ${s.x}, Y ${s.y}, Z ${s.z}, A ${r(s.a * 180 / Math.PI, 1)} deg';
    }
    for (t in machine.travel) if (t.id == "X" && t.lower < HobberScales.stroke("X").lower - 1e-9) throw 'X lower limit ${t.lower} is below the design stroke ${HobberScales.stroke("X").lower} the relief is derived for';
    state.setJoint("X", xl);
    state.forwardKinematics();
    var cache = new machinekit.assembly.PosedParts(), members = machine.components();
    var castingPose = state.worldPose("bed/casting");
    var expectedCasting = F.compose(toMachine, FoundationLayout.worldPose("bed"));
    var cp = F.toRotationMatrix(castingPose), ce = F.toRotationMatrix(expectedCasting);
    var offset = Math.max(Math.abs(castingPose.x - expectedCasting.x), Math.max(Math.abs(castingPose.y - expectedCasting.y), Math.abs(castingPose.z - expectedCasting.z)));
    for (i in 0...9) offset = Math.max(offset, Math.abs(cp[i] - ce[i]));
    if (offset > 1e-6) throw 'the bed casting is not at the bed frame the relief assumes (off by ${offset})';
    var casting = cache.posed(members.filter(m -> m.id == "bed/casting")[0].component, castingPose);
    var motor = members.filter(m -> m.id == "spindle/motor")[0].component;
    // (2) the gap at the extremes with the overtravel.
    var ot = HobberXLayout.OVERTRAVEL, worstGap = 1e9, worstAt = "";
    var x = HobberScales.stroke("X"), y = HobberScales.stroke("Y"), z = HobberScales.stroke("Z");
    for (xv in [x.lower - ot, x.upper + ot]) for (yv in [y.lower - ot, y.upper + ot]) for (zv in [z.lower - ot, z.lower, 0.0]) for (a in [-HobberSwivelLayout.RANGE, 0.0, HobberSwivelLayout.RANGE]) {
      var posed = cache.posed(motor, analytic(xv, yv, zv, a)), gap = posed.distance(casting);
      posed.close();
      if (gap < worstGap) {
        worstGap = gap;
        worstAt = 'X ${xv}, Y ${yv}, Z ${zv}, A ${r(a * 180 / Math.PI, 1)} deg';
      }
    }
    if (worstGap < HobberSpindleRelief.RUNNING_GAP - 1e-6)
      throw 'spindle servo comes ${r(worstGap, 3)} mm from the bed casting at ${worstAt} (running gap ${HobberSpindleRelief.RUNNING_GAP} mm) with the relief in place';
    // (3) the wall under the floor, in the casting's frame (the casting is posed at the bed frame). With the servo standing above the
    // cartridge it never reaches the casting and there is no relief.
    if (relief == null) {
      cache.close();
      lines.push('Spindle servo (${HobSpindle.MOTOR}, connectors to LH assumed positions) stands above the cartridge and never comes within the ${HobberSpindleRelief.RUNNING_GAP} mm running gap of the bed casting: '
        + 'no relief in the casting; least gap to the finished casting over the extremes with overtravel ${r(worstGap, 2)} mm at ${worstAt}');
      return lines;
    }
    var wall = HobberCastingPockets.WALL;
    var d = FoundationLayout.dimensions("bed");
    var slab = cadkit.modeling.Part.box(relief.x1 - relief.x0, relief.y1 - relief.y0, wall, cadkit.modeling.Align.Min, cadkit.modeling.Align.Min, cadkit.modeling.Align.Min)
      .translated(new cadkit.modeling.Vector(relief.x0, relief.y0, relief.z0 - wall));
    var slabPosed = machinekit.assembly.PosedParts.moved(slab, castingPose);
    slab.close();
    var solid = slabPosed.intersect(casting), missing = (relief.x1 - relief.x0) * (relief.y1 - relief.y0) * wall - solid.volume();
    solid.close();
    slabPosed.close();
    cache.close();
    if (missing > 1.0) throw 'the bed casting is not solid for ${wall} mm under the spindle relief floor (z ${relief.z0}): ${r(missing)} mm3 missing in the footprint x ${relief.x0}..${relief.x1}, y ${relief.y0}..${relief.y1}';
    lines.push('Spindle servo relief in the bed casting (${HobSpindle.MOTOR}, connectors to LH assumed positions): pocket x ${relief.x0}..${relief.x1}, y ${relief.y0}..${relief.y1} in the casting frame '
      + '(${r(relief.width, 1)} x ${r(relief.depth, 1)} mm), floor z ${relief.z0} (${r(d[2] - relief.z0, 1)} mm below the top), '
      + 'box volume ${r(relief.volume / 1000, 1)} cm3, opens through ${relief.faces.join(", ")}; set by ${relief.governing} (${relief.poses} poses); ' +
      'least gap to the finished casting over the extremes with overtravel ${r(worstGap, 2)} mm at ${worstAt}; ${wall} mm of material under the floor');
    return lines;
  }

  /** Procedures, drive sizing and strict integrity of the assembled machine; returns the number of checks. */
  public static function run(machine:FlexsplineHobber, sweep:Bool = false):Int {
    var presentation = materia.presentation.Presentations.assembled(machine.definition(), -1.1, 0.35, "hobber");
    presentation.procedures = HobberAssemblyProcedures.build(machine);
    materia.presentation.PresentationCodec.validate(machine.definition(), presentation);
    var headReport = swivelLoad(machine);
    // A common translation of the entire machine cannot change head load on
    // the output face. This catches mixing posed COM with a construction datum.
    var translated:Map<String, materia.assembly.AssemblyRecord.AssemblyFrame> = [];
    for (id => pose in machine.solvedPoses()) translated.set(id, F.compose(F.translation(17, 29, 31), pose));
    if (headReport.join("\n") != swivelLoad(machine, translated).join("\n")) throw "head demands depend on the machine's world origin";
    var lines = HobberServiceDesignChecks.run(machine).concat(bt30(machine)).concat(outboard(machine)).concat(sizing(machine)).concat(scales(machine)).concat(headReport).concat(zOffsetMoments(machine)).concat(spindleSeat(machine)).concat(spindleDrive(machine).lines).concat(zSaddleModes(machine)).concat(spindleRelief(machine)), errors = newErrors(machine, sweep);
    for (line in lines) Sys.println(line);
    if (errors.length > 0) throw 'hobber strict integrity: ${errors.length} findings\n  ' + errors.join("\n  ");
    return presentation.procedures.length + lines.length + 2;
  }

  /** The machine survives encode and decode with its member recipes and collision hulls. */
  public static function roundTrip(machine:FlexsplineHobber):Void {
    var rebuilt = MachineAssembly.decode(machine.encode(), MachineKitComponents.standard());
    if (rebuilt.components().length != machine.components().length) throw "the machine lost members in persistence";
  }

  /**
   * Drive sizing on the assembled machine at 400 W: the layout report's continuous thrust margin (rated torque, 20 % reserve,
   * 0.5 m/s2, 100 N process + 100 N friction, gravity on Z) and the full duty envelope of `LinearDriveSizing` (peak and RMS
   * torque with a 1.5 reserve, speed, inertia ratio). Returns one line per axis; throws when an axis fails.
   */
  public static function sizing(machine:FlexsplineHobber):Array<String> {
    var report:Dynamic = FlexsplineHobberLayoutReport.measure(machine), lines:Array<String> = [];
    for (entry in (report.drives : Array<Dynamic>)) {
      var id:String = entry.axis, members = machine.components();
      var motor:machinekit.motion.ServoMotor = cast members.filter(m -> m.id == HobberDrives.member(id,"motor"))[0].component;
      var nut:machinekit.motion.BallNut = cast members.filter(m -> m.id == HobberDrives.member(id,"nut"))[0].component;
      var lead = 0.005, q = lead / (2 * Math.PI), mass:Float = entry.movingMassKg;
      var demand = machinekit.sizing.LinearDriveSizing.demand({
        massKg: mass, motors: 1, metresPerRadian: q, efficiency: nut.efficiency(), rotaryInertiaKgM2: 0.4e-4,
        acceleration: 0.5, velocity: 0.2, strokeM: id == "Y" ? 0.03 : id == "X" ? 0.095 : 0.2,
        cuttingForceN: 100, dragForceN: 100, gravityForceN: id == "Z" ? mass * 9.80665 : id == "Y" ? mass * 9.80665 * Math.sin(Math.PI / 12) : 0,
        rotaryDragNm: 0.1, reserve: 1.5
      }, motor.rating);
      var line = '$id ${motor.rating.designation}: moving ${Math.round(mass * 100) / 100} kg, thrust margin ${Math.round(entry.thrustMarginN)} N, '
        + 'peak ${Math.round(demand.peakNm * 1000) / 1000} N m (peak rating ${motor.rating.peakTorque}), rms ${Math.round(demand.rmsNm * 1000) / 1000} N m '
        + '(rated ${motor.rating.ratedTorque}), inertia ratio ${Math.round(demand.inertiaRatio * 100) / 100}, failures: ${demand.failures.join("; ")}';
      lines.push(line);
      if (demand.failures.length > 0) throw "drive sizing fails: " + line;
      if (id == "Z") {
        // The braked servo holds the axis with the power off: the brake is bound to the motor's drive and holds the screw torque of the weight.
        var brake = machine.actuatorFor("Zdrive").brake, load = mass * 9.80665 * q;
        if (brake == null) throw "Z drive has no brake bound";
        if (brake.holdingEffort < 1.5 * load) throw 'Z brake holds ${brake.holdingEffort} N m against ${Math.round(load * 1000) / 1000} N m of weight';
        lines.push('Z brake: holds ${Math.round(brake.holdingEffort * 100) / 100} N m (assumed) against ${Math.round(load * 1000) / 1000} N m of weight, release ${brake.releaseChannel} read back on ${brake.releasedSensor}');
      }
    }
    return lines;
  }

  public static function main():Void {
    var args = Sys.args(), began = Sys.time();
    if (args.indexOf("spindlepose") >= 0) {
      var names = [], spindles:Array<HobSpindle> = [];
      for (k in 0...4) {
        names.push("k" + k);
        spindles.push(new HobSpindle(null, null, k));
      }
      names.push("default");
      spindles.push(new HobSpindle());
      for (i in 0...names.length) {
        var poses = spindles[i].solvedPoses();
        var m:materia.assembly.AssemblyRecord.AssemblyFrame = poses.get("motor");
        Sys.println(names[i] + " motor " + [m.x, m.y, m.z] + " rot " + F.toRotationMatrix(m).map(v -> Math.round(v * 1000) / 1000));
      }
      var spin = HobberSpindleRelief.motorZeroPose(HobSpindle.MOTOR_CLOCKING);
      Sys.println("zero pose " + [spin.x, spin.y, spin.z] + " rot " + F.toRotationMatrix(spin).map(v -> Math.round(v * 1000) / 1000));
      if (args.length == 1) return;
    }
    if (args.indexOf("clocking") >= 0) {
      for (k in 0...4) {
        var r = HobberSpindleRelief.reliefFor(k);
        Sys.println(r == null ? 'clocking $k: no relief' : 'clocking $k: x ${r.x0}..${r.x1} y ${r.y0}..${r.y1} floor ${r.z0} (box ${Math.round(r.volume / 1000)} cm3) faces ${r.faces.join(",")} governed by ${r.governing}');
      }
      if (args.length == 1) return;
    }
    var machine = new FlexsplineHobber();
    Sys.println('machine built: ${machine.components().length} members in ${Math.round(Sys.time() - began)} s (${Math.round(Sys.cpuTime())} s CPU)');
    if (args.indexOf("ymodule") >= 0) {
      for(entry in machine.components())if(StringTools.startsWith(entry.id,"Ymodule/")) Sys.println(entry.id+" "+entry.component.massProperties().mass+" kg");
    }
    if (args.indexOf("memos") >= 0) for (line in machinekit.cache.Memos.report()) Sys.println(line);
    if (args.indexOf("procedures") >= 0) {
      var presentation = materia.presentation.Presentations.assembled(machine.definition(), -1.1, 0.35, "hobber");
      presentation.procedures = HobberAssemblyProcedures.build(machine);
      materia.presentation.PresentationCodec.validate(machine.definition(), presentation);
      Sys.println('procedures valid: ${[for (p in presentation.procedures) p.id].join(", ")}');
    }
    if (args.indexOf("sizing") >= 0) for (line in sizing(machine)) Sys.println(line);
    if (args.indexOf("scales") >= 0) for (line in scales(machine)) Sys.println(line);
    if (args.indexOf("swivel") >= 0) for (line in swivelLoad(machine)) Sys.println(line);
    if (args.indexOf("zoffset") >= 0) for (line in zOffsetMoments(machine)) Sys.println(line);
    if (args.indexOf("servicedesign") >= 0) for (line in HobberServiceDesignChecks.run(machine)) Sys.println(line);
    if (args.indexOf("chipguards") >= 0) for (line in HobberChipGuardChecks.run(machine)) Sys.println(line);
    if (args.indexOf("motorstudy") >= 0) for (line in HobberMotorStudy.run(machine)) Sys.println(line);
    if (args.indexOf("outboard") >= 0) for (line in outboard(machine)) Sys.println(line);
    if (args.indexOf("bt30") >= 0) for (line in bt30(machine)) Sys.println(line);
    if (args.indexOf("spindleseat") >= 0) for (line in spindleSeat(machine)) Sys.println(line);
    if (args.indexOf("roundtrip") >= 0) {
      roundTrip(machine);
      Sys.println("round trip ok");
    }
    if (args.indexOf("spindledrive") >= 0) for (line in spindleDrive(machine).lines) Sys.println(line);
    if (args.indexOf("zsaddle") >= 0) for (line in zSaddleModes(machine)) Sys.println(line);
    if (args.indexOf("spindlerelief") >= 0) for (line in spindleRelief(machine)) Sys.println(line);
    if (args.indexOf("nutblocks") >= 0) for (d in machinekit.hobbing.HobberDrives.all().filter(axis->axis.id!="Y")) {
      var nut = new machinekit.motion.BallNut(), b = machinekit.hobbing.HobberDrives.nutBlock(d, nut), pose = machinekit.hobbing.HobberDrives.nutBlockPose(d, nut);
      function w(x:Float, y:Float, z:Float) {
        var p = F.transformPoint(pose, x, y, z);
        return [Math.round(p.x * 10) / 10, Math.round(p.y * 10) / 10, Math.round(p.z * 10) / 10];
      }
      Sys.println('${d.id}: block ${b.width} x ${b.thickness} x ${b.length}, centre height ${b.centreHeight}, base xs ${b.baseHoleXs()}, base depth ${b.baseDepth}');
      Sys.println('  flange holes ${[for (h in nut.mountingHoles()) [Math.round(h.x * 10) / 10, Math.round(h.y * 10) / 10]]}, tap drill ${nut.mountingTapDrill()}, flange dia ${nut.flangeDiameter}, bolt circle ${nut.boltCircleDiameter}');
      Sys.println('  world: origin ${w(0, 0, 0)} x ${w(1, 0, 0)} y ${w(0, 1, 0)} z ${w(0, 0, 1)} base centre ${w(0, -b.centreHeight, b.thickness / 2)}');
    }
    if (args.indexOf("crash") >= 0) for (line in HobberCrashProtection.report(machine)) Sys.println(line);
    if (args.indexOf("fasteners") >= 0) {
      Sys.println(machinekit.hobbing.HobberCarriageStockCheck.report());
      for (line in HobberFastenerChecks.report(machine)) Sys.println(line);
    }
    var checkedIntegrity:Null<Array<String>> = null;
    if (args.indexOf("fastenerfit") >= 0) {
      var report = AssemblyIntegrity.fasteners(machine, integrityOptions(machine, false));
      for (item in report.diagnostics.items) if (item.code == "fasteners.edge-support")
        Sys.println(item.code + " " + item.message);
      for (e in report.errors()) Sys.println(e.code + " " + e.message);
      Sys.println('fastener fit: ${report.errors().length} errors');
      report.require();
    }
    if (args.indexOf("integrity") >= 0) {
      var errors = integrity(machine, args.indexOf("sweep") >= 0);
      checkedIntegrity = errors;
      var counts = new Map<String, Int>();
      for (error in errors) {
        var code = error.split(" ")[0];
        counts.set(code, (counts.exists(code) ? counts.get(code) : 0) + 1);
      }
      for (code => count in counts) Sys.println('$code: $count');
      for (error in errors) Sys.println("  " + error);
      Sys.println('integrity: ${errors.length} errors in ${Math.round(Sys.time() - began)} s');
      if (errors.length > 0) Sys.exit(1);
    }
    // Service reports the declared paths without changing the machine.
    if (args.indexOf("coverfit") >= 0) for (line in coverFit(machine)) Sys.println(line);
    if (args.indexOf("service") >= 0) for (line in service(machine, checkedIntegrity)) Sys.println(line);
  }
}
