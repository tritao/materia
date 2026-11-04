import machinekit.assembly.AssemblyIntegrity;
import machinekit.assembly.MachineAssembly;
import machinekit.component.MachineKitComponents;
import machinekit.hobbing.HobberScales;
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

  /** Procedures, drive sizing and strict integrity of the assembled machine; returns the number of checks. */
  public static function run(machine:FlexsplineHobber, sweep:Bool = false):Int {
    var presentation = materia.presentation.Presentations.assembled(machine.definition(), -1.1, 0.35, "hobber");
    presentation.procedures = HobberAssemblyProcedures.build(machine);
    materia.presentation.PresentationCodec.validate(machine.definition(), presentation);
    var lines = sizing(machine).concat(scales(machine)), errors = newErrors(machine, sweep);
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
    }
    return lines;
  }

  public static function main():Void {
    var args = Sys.args(), began = Sys.time();
    if (args.indexOf("recipes") >= 0) {
      var block = machinekit.hobbing.HobberDrives.nutBlock(machinekit.hobbing.HobberDrives.x(), new machinekit.motion.BallNut());
      var type = block.componentType();
      if (type == null) throw "the nut block has no recipe";
      Sys.println('nut block recipe ${type.id}: ${block.values().token("drawing")} h=${block.values().number("centreHeight")}');
      return;
    }
    if (args.indexOf("relief") >= 0) {
      for (d in machinekit.hobbing.HobberDrives.all()) Sys.println(d.id + " relief " + machinekit.hobbing.HobberDrives.mountRelief(d));
      return;
    }
    if (args.indexOf("bounds") >= 0) {
      var d = machinekit.hobbing.HobberDrives.x();
      for (item in [{n: "mount", p: (d.drive.fixed : machinekit.component.MachineComponent)}, {n: "floating", p: (d.drive.floating : machinekit.component.MachineComponent)}]) {
        var g = item.p.geometry(), b = g.shape.bounds();
        Sys.println('${item.n}: x ${b.get_min().get_x()}..${b.get_max().get_x()} y ${b.get_min().get_y()}..${b.get_max().get_y()} z ${b.get_min().get_z()}..${b.get_max().get_z()} shaftHeight ${d.drive.fixed.spec.shaftHeight}');
        g.close();
      }
      return;
    }
    if (args.indexOf("equality") >= 0) {
      var a:Array<Array<Float>> = [[1.0, 2.5], [3.0]], b:Array<Array<Float>> = [[1.0, 2.5], [3.0]];
      Sys.println("nested literal equal: " + haxeon.Equality.equals(a, b));
      var hull = machinekit.component.CollisionHullFacet.of(new machinekit.hobbing.HobberSaddle("GE-Y-nut-seat"));
      if (hull == null) throw "no hull";
      var copy = new machinekit.component.CollisionHullFacet(hull.hulls);
      Sys.println("hull facet vs copy: " + haxeon.Equality.equals(hull.hulls, copy.hulls) + " hull " + hull.hulls[0]);
      var viaWire = [for (h in hull.hulls) [for (v in h) v + 0.0]];
      Sys.println("hull vs rebuilt arrays: " + haxeon.Equality.equals(hull.hulls, viaWire));
      return;
    }
    if (args.indexOf("tiny") >= 0) {
      for (design in ["GE-Z-saddle", "GE-X-carriage", "GE-X-drive-fixed", "GE-Z-drive-fixed", "GE-Y-drive-fixed", "GE-Y-nut-seat", "GE-Y-saddle"]) {
        var tiny = new MachineAssembly();
        tiny.addComponent("part", new machinekit.hobbing.HobberSaddle(design));
        try {
          var back = MachineAssembly.decode(tiny.encode(), MachineKitComponents.standard());
          Sys.println('$design: round trip ok (${back.components().length} members)');
        } catch (e:Dynamic) {
          var a = machinekit.component.CollisionHullFacet.of(new machinekit.hobbing.HobberSaddle(design));
          var first = a == null ? "none" : '${a.hulls.length} hulls, first sizes ${[for (h in a.hulls.slice(0, 6)) h.length]}';
          Sys.println('$design: FAILS ${Std.string(e).split("\n")[0]} ($first)');
        }
      }
      return;
    }
    if (args.indexOf("hulls") >= 0) {
      for (design in ["GE-X-carriage", "GE-Z-saddle"]) {
        var a = machinekit.component.CollisionHullFacet.of(new machinekit.hobbing.HobberSaddle(design));
        var b = machinekit.component.CollisionHullFacet.of(new machinekit.hobbing.HobberSaddle(design));
        if (a == null || b == null) throw "no hulls for " + design;
        var bad = 0;
        for (i in 0...a.hulls.length) for (j in 0...a.hulls[i].length) {
          var u = a.hulls[i][j], v = b.hulls[i][j];
          if (u != v || Math.isNaN(u)) {
            if (bad++ < 5) Sys.println('$design hull $i[$j]: $u vs $v');
          }
        }
        Sys.println('$design: ${a.hulls.length} hulls, $bad differing values');
        var text = haxe.Json.stringify(a.hulls), back:Array<Array<Float>> = haxe.Json.parse(text), lost = 0;
        for (i in 0...a.hulls.length) for (j in 0...a.hulls[i].length) if (a.hulls[i][j] != back[i][j]) {
          if (lost++ < 5) Sys.println('$design json $i[$j]: ${a.hulls[i][j]} -> ${back[i][j]}');
        }
        Sys.println('$design: json round trip changes $lost values');
      }
      return;
    }
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
