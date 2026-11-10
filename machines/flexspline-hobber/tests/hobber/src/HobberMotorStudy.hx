import hobber.*;
import hobber.HobberLayout;
import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.assembly.PosedParts;
import machinekit.assembly.PosedParts.PartBox;
import materia.assembly.AssemblyFrames as F;

/** P3 envelope study, not an alternate installed machine. Both side directions
 * retain the purchased drive and a rigidly rotated cradle. The negative-X side
 * needs a 10 mm interface plate. Unqualified joint compliance can only lower
 * its reported mode upper bounds. No claim of a released beside mounting. */
class HobberMotorStudy {
  static function r(x:Float, n:Int = 2):Float return Math.round(x * Math.pow(10, n)) / Math.pow(10, n);
  static function gap(a:PartBox, b:PartBox):Float {
    var x = Math.max(0, Math.max(a.minX - b.maxX, b.minX - a.maxX));
    var y = Math.max(0, Math.max(a.minY - b.maxY, b.minY - a.maxY));
    var z = Math.max(0, Math.max(a.minZ - b.maxZ, b.minZ - a.maxZ));
    return Math.sqrt(x * x + y * y + z * z);
  }
  public static function run(machine:FlexsplineHobber):Array<String> {
    var members = machine.components(), poses = machine.solvedPoses(), def = machine.definition();
    var turning = new Map<String, Bool>(); turning.set("swivel/output", true);
    var changed = true;
    while (changed) {
      changed = false;
      for (j in def.joints) if (turning.exists(j.parent) && !turning.exists(j.child)) {turning.set(j.child, true); changed = true;}
    }
    var group = ["motorCradle", "spindle/motor", "spindle/motorShaft", "spindle/motorEncoder", "spindle/motorPulley", "spindle/belt"];
    var expanded = true;
    while (expanded) {
      expanded = false;
      for (j in def.joints) if (group.indexOf(j.parent) >= 0 && group.indexOf(j.child) < 0) {group.push(j.child); expanded = true;}
    }
    var dynamics = HobberFrameChecks.spindleDrive(machine);
    var back = F.inverse(hobber.HobberFrame.workToMachine()), face = F.compose(back, poses.get("swivel/output"));
    var lines = ["P3: one installed top layout; beside layouts below are geometric studies. Adapter and joint stiffness assumptions are explicit."];
    var model = new cadkit.modeling.AssemblyModel("mm"); machine.addTo(model, "");
    // Independent study snapshot extends linear limits by the design overtravel.
    // The installed machine definition and its operational limits stay intact.
    var studyDefinition = model.definition("motor-layout-study");
    for (j in studyDefinition.joints) {
      if (j.id == "X") {j.limits.lower = hobber.HobberLayout.get().xMin - 5; j.limits.upper = 115;}
      if (j.id == "Y") {j.limits.lower = -15; j.limits.upper = 15;}
      if (j.id == "Z") {j.limits.lower = -15; j.limits.upper = 195;}
    }
    var state = new cadkit.modeling.AssemblyState(studyDefinition);
    var cache = new PosedParts();
    var adapter = Part.box(10, 89, 72).translated(new Vector(-53, 194.5, HobberLayout.get().nativeHobCentreZ - 36));
    var adapterMass = adapter.volume() * 1e-9 * 7850;
    var targets = ["bed/casting", "column/casting", "Xbridge", "Zplate", "Ymodule/carriage", "Ymodule/base", "Ymodule/railLeft", "Ymodule/railRight", "Ymodule/base", "Ymodule/base", "Ymodule/carriage", "ZdriveFixedBar", "ZdriveFloatingBar"];
    var lower = hobber.HobberLayout.get().xMin;
    for (side in [0, -1, 1]) {
      var c = side == 0 ? 1.0 : 0.0, s = side * 1.0;
      var rotate = F.fromRotationMatrix(0, 0, 0, [c, -s, 0, s, c, 0, 0, 0, 1]);
      var around = F.compose(poses.get("spindle/housing"), F.compose(rotate, F.inverse(poses.get("spindle/housing"))));
      var mass = 0.0, mx = 0.0, my = 0.0, mz = 0.0;
      for (m in members) if (turning.exists(m.id)) {
        var p = group.indexOf(m.id) >= 0 ? F.compose(around, poses.get(m.id)) : poses.get(m.id);
        var props = m.component.massProperties(), at = F.transformPoint(F.compose(back, p), props.centreOfMass.x, props.centreOfMass.y, props.centreOfMass.z);
        mass += props.mass; mx += props.mass * at.x; my += props.mass * at.y; mz += props.mass * at.z;
      }
      // Assumed solid interface blank. Finish cuts, remounting hardware and
      // access remain unqualified; this is a nominal study mass estimate.
      if (side != 0) {
        var centre = F.transformPoint(F.compose(back, poses.get("spindle/housing")), 0, 0, 0);
        mass += adapterMass; mx += adapterMass * (centre.x + (side < 0 ? 53 : -53));
        my += adapterMass * 194.5; mz += adapterMass * HobberLayout.get().nativeHobCentreZ;
      }
      var x = mx / mass - face.x, y = my / mass - face.y, z = mz / mass - face.z, torque = 0.0;
      for (angle in -15...16) torque = Math.max(torque, mass * 9.80665 * Math.abs(y * Math.cos(angle * Math.PI / 180) - z * Math.sin(angle * Math.PI / 180)) / 1000);
      var tool = F.transformPoint(F.compose(back, poses.get("hob")), 0, 0, 0);
      var tilt = -mass * 9.80665 * x / 1000 + 100 * (face.x - tool.x) / 1000;
      var minimum = 1e9, nearest = "", overlap = 0.0, witness = "", sampled = 0;
      for (xx in [lower - 5, 115.0]) for (yy in [-15.0, 15.0]) for (zz in [-15.0, 195.0]) for (aa in [-15.0, 0.0, 15.0]) {
        state.setJoint("X", xx); state.setJoint(FlexsplineHobber.Y, yy); state.setJoint("Z", zz); state.setJoint(FlexsplineHobber.A, aa * Math.PI / 180); state.forwardKinematics(); sampled++;
        var spindlePose = state.worldPose("spindle/housing");
        var rotation = F.compose(spindlePose, F.compose(rotate, F.inverse(spindlePose)));
        for (id in group) {
          var part = members.filter(m -> m.id == id)[0].component, p = F.compose(rotation, state.worldPose(id)), box = cache.box(part, p);
          for (other in targets) {
            var target = members.filter(m -> m.id == other)[0].component, q = state.worldPose(other), ob = cache.box(target, q);
            if (gap(box, ob) > minimum + 1e-8) continue;
            var a = cache.posed(part, p), b = cache.posed(target, q), d = a.distance(b);
            if (d < minimum) {minimum = d; nearest = id + "/" + other; witness = 'X$xx Y$yy Z$zz A$aa';}
            if (d < 1e-7) {var shared = a.intersect(b); overlap = Math.max(overlap, shared.volume()); shared.close();}
            a.close(); b.close();
          }
        }
      }
      var modes = [for (m in dynamics.modes) m.name + " " + r(m.f, 0) + " Hz"];
      var loadedModes = [for (m in dynamics.modes) m.name + " " + r(m.f * Math.sqrt(dynamics.modalMass / (dynamics.modalMass + adapterMass / 3)), 0) + " Hz"];
      lines.push((side == 0 ? "Top installed" : side < 0 ? "Beside inward (+work X)" : "Beside outward (-work X)")
        + ': head ${r(mass)} kg, COM relative output face (${r(x, 1)}, ${r(y, 1)}, ${r(z, 1)}) mm; A gravity torque ${r(torque)} N m, eRob tilt incl 100 N cut ${r(tilt)} N m; '
        + 'modes ${modes.join(", ")}${side == 0 ? " (existing analytic model)" : " (rigid-interface upper bounds under the same-cradle model; extra joint unqualified); if one third of adapter mass participates: " + loadedModes.join(", ")}; '
        + 'minimum ${r(minimum, 3)} mm, ${r(overlap, 3)} mm3 maximum sampled overlap, $nearest at $witness, $sampled states; '
        + (side == 0 ? "one existing cradle interface" : 'one assumed 10x72x89 steel interface blank ${r(adapterMass, 3)} kg, two interfaces instead of one; additional hardware awaits joint design.'));
    }
    adapter.close(); cache.close();
    lines.push("Decision: retain the installed top layout. Beside adds mass and an unqualified interface; the outward position increases eRob tilt, while the inward position must also clear the existing head structure. It is not better on every required measure. The geometric study excludes remounting hardware and adapter collisions; no candidate release is claimed.");
    return lines;
  }
}
