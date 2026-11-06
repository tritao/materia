import machinekit.assembly.MachineAssembly;
import machinekit.assembly.AssemblyIntegrity;
import machinekit.assembly.PosedParts;
class MillFramedCoverChecks {
  static function check(condition:Bool, message:String):Void {
    if (!condition) throw "Framed cover: " + message;
  }
  /** Validate the declared host thread and foot bearing material in the actual drilled CAD, not just the face records. */
  static function mountMaterial(mill:MachineAssembly):Void {
    var derived = mill.derivedFasteners();
    if (derived == null) throw "Missing mounting joints";
    var poses = mill.solvedPoses();
    var parts = new Map<String, machinekit.component.MachineComponent>();
    for (m in mill.components()) parts.set(m.id, m.component);
    var cache = new machinekit.assembly.PosedParts();
    for (joint in derived.joints) {
      var screw = machinekit.standard.SocketHeadCapScrew.catalog().get(joint.size);
      var host = parts.get(joint.receivingMember), clamped = parts.get(joint.clampedMember);
      var face = host.fasteningFaces().filter(f -> f.id == joint.receivingFace)[0];
      var world = materia.assembly.AssemblyFrames.compose(poses.get(joint.receivingMember), face.frame);
      for (hole in joint.hostHoles) for (zone in ["thread wall", "grip", "head bearing"]) {
        var thread = zone == "thread wall", from = thread ? 0.01 : -joint.grip + 0.01;
        var to = thread ? joint.threadDepth - 0.01 : zone == "grip" ? -0.01 : -joint.grip + 0.2;
        var inner = thread ? joint.tapDrill / 2 + 0.1 : screw.clearanceMedium / 2 + 0.05,
          outer = thread ? joint.tapDrill / 2 + 3 : screw.headDiameter / 2;
        var cylinder = cadkit.modeling.Part.cylinderSpan(outer, from, to, hole.x, hole.y);
        var bore = cadkit.modeling.Part.cylinderSpan(inner,
          from - 1, to + 1, hole.x, hole.y), ring = cylinder.subtract(bore);
        cylinder.close();
        bore.close();
        var expected = ring.shape.volume();
        var tool = machinekit.assembly.PosedParts.moved(ring, world);
        ring.close();
        var id = thread ? joint.receivingMember : joint.clampedMember;
        var body = cache.posed(parts.get(id), poses.get(id));
        var actual = machinekit.assembly.PosedParts.commonVolume(
          body,
          machinekit.assembly.PosedParts.boxOf(body),
          tool,
          machinekit.assembly.PosedParts.boxOf(tool)
        );
        body.close();
        tool.close();
        check(Math.abs(expected - actual) < 0.01, '${joint.id}: $zone has $actual of $expected mm3 of required stock');
      }
    }
    cache.close();
  }

  /** Persistence must preserve the actual drilled stock, not merely the displayed mesh. */
  static function roundTrip(mill:MachineAssembly):Void {
    var restored = MachineAssembly.decode(mill.encode()), byId = new Map<String, machinekit.component.MachineComponent>();
    for (row in restored.components()) byId.set(row.id, row.component);
    var checked = 0;
    for (row in mill.components()) if (StringTools.startsWith(row.id, "xyStage/cover")
      && (Std.isOfType(row.component, machinekit.cover.WayCoverMount)
        || Std.isOfType(row.component, machinekit.milling.MillCoverOutrigger))) {
      var other = byId.get(row.id);
      if (other == null) throw "Saved assembly omitted " + row.id;
      check(row.component.bom.fingerprint == other.bom.fingerprint, "saved mount retains its BOM revision");
      check(haxe.Json.stringify(row.component.machinedCuts()) == haxe.Json.stringify(other.machinedCuts()),
        "saved mount retains its drilling table");
      var a = row.component.geometry(), b = other.geometry(), added = a.subtract(b), removed = b.subtract(a);
      var difference = added.volume() + removed.volume();
      a.close(); b.close(); added.close(); removed.close();
      check(difference < 0.01, row.id + " rebuilt stock differs by " + difference + " mm3");
      checked++;
    }
    check(checked == 6, "both walls and all four outriggers survive persistence");
    Sys.println("Framed cover persistence: six drilled mounts round-trip within 0.01 mm3");
  }

  public static function main():Void {
    var reference = new BenchMill(false, false), frame = MillProject.smallFrame();
    var stage = new machinekit.milling.MillXYStage(cast frame.xyAxis);
    var mill = MillFramePreview.assemble(reference, frame, stage);
    check(
      stage.saddle.machinedCuts().length == 0 && stage.table.machinedCuts().length == 0,
      "a concept does not drill its caller's reusable stage"
    );
    var repeated = MillFramePreview.assemble(reference, frame, stage);
    check(repeated.derivedFasteners() != null, "the same stage builds another independently fastened concept");
    for (requirement in stage.designRequirements()) switch requirement {
      case ServiceRemoval(_, _, _, off, _):
        for (member in off) check(
          !StringTools.startsWith(member, "cover"),
          "cover preparation stays in the owned stage snapshot"
        );
      case _:
    }
    var d:machinekit.assembly.JointFasteners.FastenerDerivation = cast mill.derivedFasteners();
    check(
      d.joints.length == 12 && d.count() == 24 && d.dowelCount() == 0,
      "two framed X covers derive twelve joints and 24 screws"
    );
    roundTrip(mill);
    // Check new procurement independently; the complete framed BOM has existing source-seat metadata conflicts.
    var purchased = new machinekit.component.Bom();
    for (entry in mill.components()) if (StringTools.startsWith(entry.id, "xyStage/cover") || StringTools.startsWith(
      entry.id,
      "frame/base/coverFront"
    )) purchased.addComponent(entry.component);
    for (line in d.lines()) purchased.add(line.item, line.quantity);
    var cuts = 0;
    for (member => holes in d.machining) cuts += holes.length;
    check(cuts == 48, "the joints cut every new clearance and tapped seat");
    var bom = purchased.lines(), coverCount = 0;
    for (line in bom) if (StringTools.startsWith(line.partNumber, "WAYCOVER-")) coverCount += line.quantity;
    var saved = mill.describe();
    var wipers = [for (extra in saved.machine.bomExtras) if (StringTools.startsWith(extra.item.partNumber,
      "WIPER-")) extra];
    check(coverCount == 2, "two complete covers are purchased");
    check(
      wipers.length == 1 && wipers[0].quantity * wipers[0].item.quantity == 24,
      "two end wipers on each of twelve guide blocks"
    );
    Sys.println('FRAMED HARDWARE ${d.joints.length} joints, ${d.count()} screws, ${d.dowelCount()} dowels');
    for (j in d.joints) Sys
      .println('JOINT ${j.id}: ${j.count} ${j.size}x${j.length}, grip ${j.grip}, engagement ${j.engagement}');
    var fit = AssemblyIntegrity.fasteners(mill);
    for (e in fit.errors()) Sys.println('FIT ${e.code}: ${e.message}');
    Sys.println('FRAMED FASTENER FINDINGS ${fit.errors().length}');
    mountMaterial(mill);
    var entries = mill.components(),
      added = entries.filter(m -> StringTools.startsWith(m.id, "xyStage/cover")), cache = new PosedParts();
    var model = new cadkit.modeling.AssemblyModel("mm");
    mill.addTo(model, "");
    var state = new cadkit.modeling.AssemblyState(model.definition("framed-cover-fit")), seen = new Map<String, Bool>();
    for (entry in entries) if (StringTools.startsWith(entry.id, "xyStage/cover") && StringTools.endsWith(
      entry.id,
      "-fixed"
    )) {
      var section:machinekit.cover.WayCoverSection = cast entry.component;
      var plan = section.plan, id = entry.id.substr(0, entry.id.length - 6), yAxis = id.indexOf("YFront") >= 0;
      var axis = yAxis ? "xyStage/y" : "xyStage/x", half = yAxis ? 80.0 : 125.0, near = StringTools.endsWith(
        id,
        "Right"
      ) ? -half : half;
      check(plan.compressed <= plan.space, "folded cover fits its allocated space");
      for (at in [{value: near, length: plan.space}, {value: -near, length: plan.extended}]) {
        state.setJoint(axis, at.value);
        state.forwardKinematics();
        var fixed = state.worldPose(id + "-fixed"), moving = state.worldPose(id + "-moving");
        var span = Math.sqrt(Math.pow(fixed.x - moving.x, 2) + Math.pow(fixed.y - moving.y, 2) + Math.pow(
          fixed.z - moving.z,
          2
        ));
        check(Math.abs(span - at.length) < 0.01, id + " reaches its end stop: " + span + " versus " + at.length);
      }
      state.setJoint(axis, 0);
      Sys.println(id + ": folded " + plan.compressed + " in " + plan.space + ", extends " + plan.extended + " mm");
    }
    var findings:Array<{a:String,
      b:String, volume:Float, x:Float, y:Float, z:Float}> = [], pairs = 0, start = Sys.time();
    for (x in [-125.0, 0, 125]) for (y in [-80.0, 0, 80]) for (z in [-250.0, -125, 0]) {
      state.setJoint("xyStage/x", x);
      state.setJoint("xyStage/y", y);
      state.setJoint("zAxis/z", z);
      state.forwardKinematics();
      for (a in added) for (b in entries) if (a.id != b.id && !(StringTools.startsWith(b.id,
        "xyStage/cover") && a.id > b.id)) {
        var volume = cache.overlap(a.component, state.worldPose(a.id), b.component, state.worldPose(b.id));
        pairs++;
        var key = a.id + "|" + b.id;
        if (volume > 0.01 && !seen.exists(key)) {
          seen.set(key, true);
          findings.push({a: a.id, b: b.id, volume: volume, x: x, y: y, z: z});
        }
      }
    }
    cache.close();
    Sys.println("FRAMEDCOVERFIT " + haxe.Json.stringify({
      seconds: Sys.time() - start,
      states: 27,
      pairs: pairs,
      findings: findings
    }));
    if (fit.errors().length > 0 || findings.length > 0) throw "Framed cover candidate does not fit";
  }
}
