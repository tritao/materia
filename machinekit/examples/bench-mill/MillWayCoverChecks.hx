import cadkit.modeling.AssemblyModel;
import cadkit.modeling.AssemblyState;
import machinekit.assembly.AssemblyIntegrity;
import machinekit.cover.WayCover;
import machinekit.cover.WayCoverCatalog;
import machinekit.design.DesignRequirement;
import materia.assembly.AssemblyRecord.AssemblyFrame;

/**
 * The chip protection of the bench mill (`new BenchMill(false, false, true)`): a box cover on both ends of each axis, every one from the
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

  /** Message with its numbers removed, to compare findings of two mills whose members stand a little differently. */
  static function shape(text:String):String return ~/[-0-9.]+/g.replace(text, "#");

  static function pose(mill:BenchMill, axis:String, value:Float, member:String):AssemblyFrame {
    var model = new AssemblyModel("mm");
    mill.addTo(model, "");
    var state = new AssemblyState(model.definition("mill-covers"));
    state.setJoint(axis, value);
    state.forwardKinematics();
    return state.worldPose(member);
  }

  public static function main():Void {
    var mill = new BenchMill(false, false, true);
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
    // The provisions do not add findings the mill did not have: the same sweep of the mill without covers finds the same kinds.
    var plain = AssemblyIntegrity.check(new BenchMill(false, false), {sweepStates: states});
    var before = [for (item in plain.errors()) shape(item.code + " " + item.message)];
    var added = [for (item in report.errors()) if (before.indexOf(shape(item.code + " " + item.message)) < 0) item];
    Sys.println('without covers: ${plain.errors().length} findings; ${added.length} of the covered mill\'s are new');
    for (item in added) Sys.println("  new " + item.code + ": " + item.message);
    check(added.length == 0, "the covered mill has no finding the plain one lacks: " + [for (item in added) item.message].join("; "));
    // The flange screws are derived (the mill's constructor throws without them) and fit: the covers go on after everything they surround.
    var ground = [for (entry in mill.components()) if (!mentions(entry.id, members)) entry.id];
    var derived = mill.derivedFasteners();
    if (derived == null) throw "The covered mill derived no fasteners";
    // A cover part is fitted after the screws of everything round it except the host it is bolted to (which it is a member of).
    var order = [for (id in members) {
      var hosts = [for (joint in derived.joints) if (joint.clampedMember == id) joint.receivingMember];
      {member: id, fastened: [for (other in ground) if (hosts.indexOf(other) < 0) other]};
    }];
    var screws = AssemblyIntegrity.fasteners(mill, {fittedBefore: order});
    var screwFindings = [for (item in screws.errors()) if (mentions(item.message, members)) item];
    for (item in screwFindings) Sys.println("  " + item.code + ": " + item.message);
    check(screwFindings.length == 0, "the cover flange screws fit: " + [for (item in screwFindings) item.message].join("; "));
    var flangeJoints = [for (joint in derived.joints) if (mentions(joint.clampedMember, members)) joint].length;
    check(flangeJoints == 2 * covers.length, 'each cover has a fixed and a moving flange bolted on, found $flangeJoints flange joints');
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
