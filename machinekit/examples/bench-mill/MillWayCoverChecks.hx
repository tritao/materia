import cadkit.modeling.AssemblyModel;
import cadkit.modeling.AssemblyState;
import machinekit.assembly.AssemblyIntegrity;
import machinekit.cover.WayCover;
import machinekit.cover.WayCoverCatalog;
import machinekit.design.DesignRequirement;
import materia.assembly.AssemblyRecord.AssemblyFrame;

/**
 * The chip protection of the bench mill (`new BenchMill(false, false)`): a box cover on both ends of each axis, every one from the
 * catalogue. For each: its folded length fits the space at the near stop, it reaches the far stop, the fixed and moving flanges stand that far apart
 * when the axis is posed there, and the integrity run (every combination of the three axes at their ends and middles) finds nothing on it; the flange
 * screws are derived and fit; the Y covers' gutters drain to bores that exist in the base; the BOM has the covers and the wipers.
 */
class MillWayCoverChecks {
  static function check(condition:Bool, message:String):Void {
    if (!condition) throw 'Mill way covers: $message';
  }

  static function r(value:Float):Float return Math.round(value * 10) / 10;

  /** The axis each cover rides, and the unit vector along the cover from its fixed flange. */
  static var covers:Array<{id:String, axis:String}> = [
    {id: "coverYFront", axis: "y"}, {id: "coverYRear", axis: "y"}, {id: "coverXLeft", axis: "x"}, {id: "coverXRight", axis: "x"},
    {id: "coverZTop", axis: "z"}, {id: "coverZBottom", axis: "z"}
  ];

  static function mentions(text:String, members:Array<String>):Bool {
    for (id in members) if (text.indexOf(id) >= 0) return true;
    return false;
  }

  static function pose(mill:BenchMill, axis:String, value:Float, member:String):AssemblyFrame {
    var model = new AssemblyModel("mm");
    mill.addTo(model, "");
    var state = new AssemblyState(model.definition("mill-covers"));
    state.setJoint(axis, value);
    state.forwardKinematics();
    return state.worldPose(member);
  }

  /** Validate the declared host thread and foot bearing material in the actual drilled CAD, not just the face records. */
  static function mountMaterial(mill:BenchMill, mounts:Array<String>):Void {
    var derived = mill.derivedFasteners();
    if (derived == null) throw "Missing mounting joints";
    var poses = mill.solvedPoses();
    var parts = new Map<String, machinekit.component.MachineComponent>();for (m in mill.components()) parts.set(m.id, m.component);
    for (joint in derived.joints) if (mounts.indexOf(joint.clampedMember) >= 0) {
      var host = parts.get(joint.receivingMember), clamped = parts.get(joint.clampedMember);
      var face = host.fasteningFaces().filter(f -> f.id == joint.receivingFace)[0];
      var world = materia.assembly.AssemblyFrames.compose(poses.get(joint.receivingMember), face.frame);
      for (hole in joint.hostHoles) for (zone in ["thread wall", "grip", "head bearing"]) {
        var thread = zone == "thread wall", from = thread ? 0.01 : -joint.grip + 0.01;
        var to = thread ? joint.threadDepth - 0.01 : zone == "grip" ? -0.01 : -joint.grip + 0.2;
        var inner = thread ? joint.tapDrill / 2 + 0.1 : 2.8, outer = thread ? joint.tapDrill / 2 + 3 : 4.25;
        var cylinder = cadkit.modeling.Part.cylinderSpan(outer, from, to, hole.x, hole.y);
        var bore = cadkit.modeling.Part.cylinderSpan(inner, from - 1, to + 1, hole.x, hole.y), ring = cylinder.subtract(bore);
        cylinder.close();bore.close();var expected = ring.shape.volume();
        var tool = machinekit.assembly.PosedParts.moved(ring, world);ring.close();
        var id = thread ? joint.receivingMember : joint.clampedMember, raw = parts.get(id).geometry();
        var body = machinekit.assembly.PosedParts.moved(raw, poses.get(id));raw.close();
        var actual = machinekit.assembly.PosedParts.commonVolume(body, machinekit.assembly.PosedParts.boxOf(body), tool, machinekit.assembly.PosedParts.boxOf(tool));
        body.close();tool.close();
        check(Math.abs(expected - actual) < 0.01, '${joint.id}: $zone has $actual of $expected mm3 of required stock');
      }
    }
  }

  public static function main():Void {
    var mill = new BenchMill(false, false);
    var members:Array<String> = [];
    for (entry in covers) {
      var cover = mill.coverOf(entry.id), plan = cover.plan;
      Sys.println('${entry.id}: ${plan.row.id}, ${plan.folds} folds, folds to ${r(plan.compressed)} mm in ${r(plan.space)}, reaches ${r(plan.extended)} mm, '
        + '${plan.levels} skins, flanges ${r(2 * plan.plateHalf())} x ${r(plan.outerHeight())} mm');
      check(plan.compressed <= plan.space + 1e-9, '${entry.id} folds to ${plan.compressed} mm but only ${plan.space} mm is free at the near stop');
      check(Math.abs(plan.extended - plan.space - plan.stroke) < 1e-9, '${entry.id} must reach the far stop');
      // Posed at the near stop and the far stop, the flanges stand the space and the extended length apart.
      var spec = [for (s in mill.specs) if (s.id == entry.axis) s][0];
      var near = cover.foldedAt, far = near == spec.lower ? spec.upper : spec.lower;
      var apart = (value:Float) -> {
        var a = pose(mill, entry.axis, value, cover.fixedMember), b = pose(mill, entry.axis, value, cover.movingMember);
        return Math.sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y) + (a.z - b.z) * (a.z - b.z));
      };
      check(Math.abs(apart(near) - plan.space) < 0.01, '${entry.id}: at the near stop the flanges are ${r(apart(near))} mm apart, the plan says ${r(plan.space)}');
      check(Math.abs(apart(far) - plan.extended) < 0.01, '${entry.id}: at the far stop the flanges are ${r(apart(far))} mm apart, the plan says ${r(plan.extended)}');
      members = members.concat(cover.members());
      members.push("wall" + entry.id.substr(5));
      members.push("carrier" + entry.id.substr(5));
    }
    // Every combination of the three axes at their ends and middles: the covers pass each other and the head at every one of them.
    var states = new Array<Map<String, Float>>();
    for (x in [0.0, 125, 250]) for (y in [0.0, 75, 150]) for (z in [-250.0, -125, 0]) states.push(["x" => x, "y" => y, "z" => z]);
    var started = Sys.time();
    var report = AssemblyIntegrity.check(mill, {sweepStates: states});
    var related = [for (item in report.errors()) if (mentions(item.message, members)) item];
    Sys.println('integrity: ${report.errors().length} findings in ${report.statesChecked} states, ${related.length} on the covers, ${Math.round(Sys.time() - started)} s');
    for (item in related) Sys.println("  " + item.code + ": " + item.message);
    check(related.length == 0, "the covers are clear of the whole mill in every state: " + [for (item in related) item.message].join("; "));
    // Freeze the measured covered design's full finding set, including volumes and distances. The default has one design;
    // a second, uncovered machine must not survive just to supply a regression comparison.
    var baseline:Dynamic = haxe.Json.parse(sys.io.File.getContent("cover-integrity-baseline.json"));
    var expected:Array<String> = cast baseline.findings;
    var actual = [for (item in report.errors()) item.code + " " + item.message];
    actual.sort((a, b) -> Reflect.compare(a, b));
    var added = [for (finding in actual) if (expected.indexOf(finding) < 0) finding];
    var missing = [for (finding in expected) if (actual.indexOf(finding) < 0) finding];
    check(report.statesChecked == baseline.statesChecked, "the regression baseline checks every requested state");
    check(actual.length == expected.length && added.length == 0 && missing.length == 0,
      "integrity findings changed; added: " + added.join("; ") + "; missing: " + missing.join("; "));
    Sys.println('integrity baseline: ${expected.length} exact findings unchanged');
    // The flange screws are derived (the mill's constructor throws without them) and fit: the covers go on after everything they surround.
    var derived = mill.derivedFasteners();
    if (derived == null) throw "The mill derived no fasteners";
    var screws = AssemblyIntegrity.fasteners(mill, {fittedBefore: mill.coverInstallationOrder()});
    var screwFindings = [for (item in screws.errors()) if (mentions(item.message, members)) item];
    for (item in screwFindings) Sys.println("  " + item.code + ": " + item.message);
    check(screwFindings.length == 0, "the cover flange screws fit: " + [for (item in screwFindings) item.message].join("; "));
    var flangeJoints = [for (joint in derived.joints) if (StringTools.startsWith(joint.clampedMember, "cover")) joint].length;
    check(flangeJoints == 2 * covers.length, 'each cover has a fixed and a moving flange bolted on, found $flangeJoints flange joints');
    var mounts = [for (entry in mill.components()) if (Std.isOfType(entry.component, machinekit.cover.WayCoverMount)) entry.id];
    check(mounts.length == 8, "six fixed end walls and two moving head carriers");
    var mountJoints = 0;
    for (id in mounts) {
      var joints = [for (joint in derived.joints) if (joint.clampedMember == id) joint], count = 0, pins = 0;
      for (joint in joints) count += joint.count;
      for (joint in derived.dowels) if (joint.memberA == id || joint.memberB == id) pins += joint.count;
      check(count == 2 && pins == 2, '$id needs two derived clamping screws and two locating dowels, found $count and $pins');
      mountJoints += joints.length;
    }
    check(mountJoints == 9 && derived.dowelCount() == 16, "the side-mounted right end wall has two joints; all eight mounts are located");
    mountMaterial(mill, mounts);
    // The drains: declared on the base's gutters and found by the integrity run (a missing bore would be a finding).
    var declared = 0;
    for (requirement in mill.designRequirements()) switch requirement {
      case GuardedChannel(_, channel, _, drain):
        declared++;
        check(channel == "base" && drain.length == 2, "the Y covers' gutters have a drain each side");
      case _:
    }
    check(declared == 1, "one guarded channel is declared");
    // The BOM: a cover is one line (the whole cover is one purchase) and the wipers another.
    var lines = mill.billOfMaterials().lines();
    var coverLines = [for (line in lines) if (StringTools.startsWith(line.partNumber, "WAYCOVER-") && !StringTools.endsWith(line.partNumber, "-M")) line];
    var coverQuantity = 0;
    for (line in coverLines) coverQuantity += line.quantity;
    check(coverQuantity == covers.length, 'six covers are bought, the BOM has $coverQuantity');
    var wipers = [for (line in lines) if (StringTools.startsWith(line.partNumber, "WIPER-")) line];
    check(wipers.length == 1 && wipers[0].quantity == 24, "24 block end wipers on the twelve guide blocks");
    Sys.println("mill way covers: six covers fit, reach, sweep clear, bolt on and drain; BOM has the covers and 24 wipers");
  }
}
