import materia.assembly.AssemblyDefinition.AssemblyJointRole;
import materia.assembly.AssemblyDefinition.AssemblyJointType;
import machinekit.assembly.PosedParts;
import materia.assembly.AssemblyFrames as F;

/** Same-body mount conflicts must be checked as well as moving-body conflicts.
 * Thread fits require geometry, engagement and native clearance outside the
 * declared thread envelope. Travel/tolerance acceptance is a separate check. */
class RackMountClearanceChecks {
  public static function atZero(machine:machinekit.gantry.Gantry, bed:Float):Void {
    var members = machine.components(),
      rows:Array<Dynamic> = [], threads:Array<Dynamic> = [], tested = 0, gaps:Array<Dynamic> = [];
    var contacts:Map<String, Bool> = [];
    function allow(a:String, b:String):Void {
      contacts.set(a + "|" + b, true);
      contacts.set(b + "|" + a, true);
    }
    var definition = machine.definition();
    for (j in definition.joints) if (j.role == AssemblyJointRole.Tree && j.type == AssemblyJointType.Fixed) allow(
      j.parent,
      j.child
    );
    for (relation in machine.describe().machine.transmissions) switch relation.source {
      case RackAndPinion(pinion, rack):
        if (rack != null) allow(pinion, rack);
      case _:
        // Tooth engagement has its own periodic native check.
    }
    for (mount in machine.rackMounts) {
      var racks = [for (j in definition.joints) if (j.parent == mount.attachment && Std.isOfType(
        machine.component(j.child),
        machinekit.transmission.GearRackRun
      )) j.child];
      if (racks.length != 1) throw "Rack contact graph lost its installed element";
      for (edge in mount.contacts) allow(
        edge.first == "$rack" ? racks[0] : edge.first,
        edge.second == "$rack" ? racks[0] : edge.second
      );
    }
    var focused = [for (m in members) if (StringTools.startsWith(m.id, "rackMount") || StringTools.startsWith(
      m.id,
      "pinion"
    ) || StringTools.startsWith(
      m.id,
      "switchY"
    ) || StringTools.startsWith(
      m.id,
      "switchTriggerY"
    )) m.id];
    var chosen:Map<String, Bool> = [];
    for (id in focused) chosen.set(id, true);
    PosedParts.scope(cache -> {var poses = [for (m in members) machine.zeroPose(m.id)];
    var boxes = [for (i in 0...members.length) cache.box(members[i].component, poses[i])];
    function sectionOverlap(
      run:machinekit.transmission.GearRackRun,
      pose:materia.assembly.AssemblyRecord.AssemblyFrame,
      other:machinekit.component.MachineComponent,
      otherPose:materia.assembly.AssemblyRecord.AssemblyFrame
    ):Float {
      var single = new machinekit.transmission.GearRackRun(new machinekit.component.ComponentValues().setToken(
        "designation",
        run.section.spec.designation
      ).setNumber(
        "finishPinDiameter",
        run.finishPinDiameter
      ));
      var total = 0.0;
      for (section in 0...run.count) {
        var at = F.compose(pose, F.translation(0, 0, section * run.section.spec.lengthMm));
        if (!PosedParts.apart(cache.box(single, at), cache.box(other, otherPose))) total += cache.overlap(
          single,
          at,
          other,
          otherPose
        );
      }
      return total;
    }
    Sys.println("Rack mount bounds prepared: " + members.length);
    Sys.stdout().flush();
    for (i in 0...members.length) for (j in i + 1...members.length) {
      var a = members[i], b = members[j];
      if (!chosen.exists(a.id) && !chosen.exists(b.id)) continue;
      var pa = poses[i], pb = poses[j];
      if (PosedParts.apart(boxes[i], boxes[j], 1.5)) continue;
      var fit = RackMountInterfaces.threadFit(machine, a.id, b.id);
      if (fit == null) fit = RackMountInterfaces.threadFit(machine, b.id, a.id);
      if (fit != null) {
        RackMountInterfaces.validateThreadSolid(machine, cache, fit);
        threads.push(fit);
        continue;
      }
      // Whole-run booleans otherwise revisit thousands of remote tooth faces.
      // The rack is a non-overlapping compound of these exact source sections.
      var volume = Std.isOfType(a.component, machinekit.transmission.GearRackRun) ? sectionOverlap(
        cast a.component,
        pa,
        b.component,
        pb
      ) : Std.isOfType(
        b.component,
        machinekit.transmission.GearRackRun
      ) ? sectionOverlap(
        cast b.component,
        pb,
        a.component,
        pa
      ) : cache.overlap(
        a.component,
        pa,
        b.component,
        pb
      );
      tested++;
      if (volume > 0.001) rows.push({first: a.id, second: b.id, intersectionMm3: volume});
      if (volume <= 0.001 && !contacts.exists(a.id + "|" + b.id) && !RackMountInterfaces.sameFitAssembly(
        machine,
        a.id,
        b.id
      ) && !RackMountInterfaces.clearanceBore(
        machine,
        a.id,
        b.id
      ) && !RackMountInterfaces.clearanceBore(
        machine,
        b.id,
        a.id
      )) {
        var first = cache.posed(a.component,
          pa), second = cache.posed(b.component, pb), distance = first.distance(second);
        first.close();
        second.close();
        if (!Math.isFinite(distance) || distance < 0) throw "Native fixed clearance distance failed";
        if (distance < 1.5 - 1e-6) gaps.push({first: a.id, second: b.id, distanceMm: distance, requiredMm: 1.5});
      }
    }
    });
    sys.io.File.saveContent(
      'build/router-mount-implementation/clearance-zero-' + Std.int(bed) + '.json',
      haxe.Json.stringify(
        {
          bedMm: bed,
          tested: tested,
          qualifiedThreads: threads,
          toleranceFailures: gaps,
          status: rows.length == 0
      && gaps.length == 0 ? "zero-pose solids and independent-part gaps pass; travel separate" : "FAIL",
      intersections : rows
    },
      null,
      "  "
    )
    );
    Sys.println('Rack mount zero-pose diagnostic '
      + bed + ': ' + tested + ' native pairs, ' + rows.length + ' intersections');
    Sys.stdout().flush();
    if (rows.length > 0 || gaps.length > 0) throw "Fixed mount solid/tolerance clearance failed; see zero-pose report";
  }
}
