import machinekit.assembly.AssemblyIntegrity;
import machinekit.assembly.MachineAssembly;
import machinekit.component.MachineKitComponents;
import machinekit.hobbing.HobberScales;
import machinekit.hobbing.HobberSwivelLayout;
import machinekit.hobbing.HobberZLayout;
import materia.assembly.AssemblyFrames as F;

/** Frame, drive and integrity checks on the assembled hobber (FRAME_PLAN.md H1-H6). */
class HobberFrameChecks {
  /** `AssemblyIntegrity` in strict mode on the whole machine; `sweep` false checks the rest state only. */
  public static function integrity(machine:FlexsplineHobber, sweep:Bool, gap:Float = 2.0):Array<String> {
    var options:IntegrityOptions = {strict: true, processContacts: machine.processContacts(), runningGap: gap};
    if (!sweep) options.sweepStates = [];
    var report = AssemblyIntegrity.check(machine, options);
    return [for (item in report.errors()) item.code + " " + item.message];
  }

  /**
   * Members whose strict-integrity findings are known and belong to other work, not to the frames or drives: the spindle
   * cartridge (replaced by the bought one in R2), the electrical rack's mount on the lifting seat and the chip tray's resting
   * seat. A finding on any other member fails the check, so the frames, drives, guides, casting and anchors stay covered.
   */
  static final OPEN_MEMBERS = ["spindle/", "electricalPanel", "powerSupply", "-driver", "chipTray"];

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
      var housing = poses.get(a.id + "scale"), head = poses.get(a.id + "scaleHead");
      if (housing == null || head == null) throw 'axis ${a.id} has no scale members';
      var origin = F.transformPoint(back, housing.x, housing.y, housing.z);
      var worldAlong = F.transformVector(housing, 0, 0, 1), along = F.transformVector(back, worldAlong.x, worldAlong.y, worldAlong.z);
      var centre = F.transformPoint(back, head.x, head.y, head.z);
      var d = [toolWork.x - origin.x, toolWork.y - origin.y, toolWork.z - origin.z];
      var u = [along.x, along.y, along.z], along0 = d[0] * u[0] + d[1] * u[1] + d[2] * u[2];
      var across = [for (i in 0...3) d[i] - along0 * u[i]];
      var sense = jointDirection(a.id), sign = sense[0] * u[0] + sense[1] * u[1] + sense[2] * u[2];
      // Head position along the scale: at the initial joint value, then over the stroke with the guide overtravel (5 mm each end).
      var here = (centre.x - origin.x) * u[0] + (centre.y - origin.y) * u[1] + (centre.z - origin.z) * u[2];
      var from = here + sign * (travel.lower - 5 - travel.initial), to = here + sign * (travel.upper + 5 - travel.initial);
      var low = Math.min(from, to), high = Math.max(from, to), start = a.housing.startOffset(), end = start + a.housing.measuringLength;
      var names = ["x", "y", "z"], parts = [for (i in 0...3) if (Math.abs(u[i]) < 0.5) '${names[i]} ${Math.round(Math.abs(across[i]) * 10) / 10}'];
      var total = Math.sqrt(across[0] * across[0] + across[1] * across[1] + across[2] * across[2]);
      var line = '${a.id} scale ${a.housing.designation}: Abbe offset ${parts.join(", ")} mm (${Math.round(total * 10) / 10} mm), head ' +
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
   * is labelled. Gravity is -z; the output bearing's tilting moment is taken about the output face (the bearing lies behind it, which
   * adds `force x offset`, under 1 N m here). The cutting force is 100 N at the hob, in the direction that loads the bearing most.
   * Returns one line per quantity; throws when the demand exceeds a rating that is known.
   */
  public static function swivelLoad(machine:FlexsplineHobber):Array<String> {
    var def = machine.definition(), poses = machine.solvedPoses(), back = F.inverse(FlexsplineHobber.MACHINE_FROM_WORK);
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
    var com = {x: mx / mass, y: my / mass, z: mz / mass - HobberZLayout.SEAT_Z};
    var g = 9.80665, face = HobberSwivelLayout.OUTPUT_X, overhang = (face - com.x) / 1000;
    var tilt = mass * g * overhang;
    // Gravity torque about A: the centre of mass is (dy, dz) from the axis; turned by theta its lever on the vertical force is dy cos - dz sin.
    function gravityTorque(theta:Float):Float return mass * g * (com.y * Math.cos(theta) - com.z * Math.sin(theta)) / 1000;
    var worst = 0.0, at0 = gravityTorque(0), limit = HobberSwivelLayout.RANGE;
    for (i in -15...16) worst = Math.max(worst, Math.abs(gravityTorque(i * Math.PI / 180)));
    var tool = F.transformPoint(F.compose(back, F.compose(poses.get("hob"), machine.hob.connector("centre").frame)), 0, 0, 0);
    var hobOverhang = (face - tool.x) / 1000, hobLever = Math.sqrt(Math.pow(tool.y, 2) + Math.pow(tool.z - HobberZLayout.SEAT_Z, 2)) / 1000;
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
    for (id in ["swivel/housing", "swivel/output", "swivelAdapter"]) for (m in machine.components()) if (m.id == id) newMass += m.component.massProperties().mass;
    var lines = [
      'A head: ${r(mass)} kg on the output flange (${count} members), centre of mass x ${r(com.x, 1)} y ${r(com.y, 1)} z ${r(com.z, 1)} mm from the A axis; ' +
      'overhang ${r(overhang * 1000, 1)} mm in front of the output face',
      'A head change: swivel stack was ${r(oldMass)} kg (torque motor, crossed roller, clamp, housing, platen), now actuator ${r(machine.swivel.housing.massProperties().mass + machine.swivel.output.massProperties().mass)} kg ' +
      '+ adapter ring, ${r(newMass - machine.swivel.housing.massProperties().mass - machine.swivel.output.massProperties().mass)} kg',
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

  /** Procedures, drive sizing and strict integrity of the assembled machine; returns the number of checks. */
  public static function run(machine:FlexsplineHobber, sweep:Bool = false):Int {
    var presentation = materia.presentation.Presentations.assembled(machine.definition(), -1.1, 0.35, "hobber");
    presentation.procedures = HobberAssemblyProcedures.build(machine);
    materia.presentation.PresentationCodec.validate(machine.definition(), presentation);
    var lines = sizing(machine).concat(scales(machine)).concat(swivelLoad(machine)), errors = newErrors(machine, sweep);
    for (line in lines) Sys.println(line);
    if (errors.length > 0) throw 'hobber strict integrity: ${errors.length} findings\n  ' + errors.join("\n  ");
    return presentation.procedures.length + lines.length + 1;
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
      var motor:machinekit.motion.ServoMotor = cast members.filter(m -> m.id == id + "motor")[0].component;
      var nut:machinekit.motion.BallNut = cast members.filter(m -> m.id == id + "nut")[0].component;
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
    var machine = new FlexsplineHobber();
    Sys.println('machine built: ${machine.components().length} members in ${Math.round(Sys.time() - began)} s');
    if (args.indexOf("procedures") >= 0) {
      var presentation = materia.presentation.Presentations.assembled(machine.definition(), -1.1, 0.35, "hobber");
      presentation.procedures = HobberAssemblyProcedures.build(machine);
      materia.presentation.PresentationCodec.validate(machine.definition(), presentation);
      Sys.println('procedures valid: ${[for (p in presentation.procedures) p.id].join(", ")}');
    }
    if (args.indexOf("sizing") >= 0) for (line in sizing(machine)) Sys.println(line);
    if (args.indexOf("scales") >= 0) for (line in scales(machine)) Sys.println(line);
    if (args.indexOf("swivel") >= 0) for (line in swivelLoad(machine)) Sys.println(line);
    if (args.indexOf("roundtrip") >= 0) {
      roundTrip(machine);
      Sys.println("round trip ok");
    }
    if (args.indexOf("integrity") >= 0) {
      var errors = integrity(machine, args.indexOf("sweep") >= 0);
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
  }
}
