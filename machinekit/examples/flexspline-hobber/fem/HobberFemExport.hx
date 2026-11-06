import machinekit.hobbing.HobberSaddle;
import machinekit.hobbing.HobberZLayout;
import machinekit.hobbing.HobberSwivelLayout;
import machinekit.component.MachineComponent;
import materia.assembly.AssemblyFrames as F;

/** Read-only export for the FEM checks (fem/hobber_fem.py): STEP of the four parts in their authored frame, their holes, and the masses and
    centres the loads need (servo group on the cradle, head on the A flange), all in the authored frame (work frame, z measured from HobberZLayout.SEAT_Z).
    Usage: hl main.hl <output dir>. Nothing here changes the model. */
class HobberFemExport {
  static function r(v:Float, digits:Int = 4):Float {
    var f = Math.pow(10, digits);
    return Math.round(v * f) / f;
  }

  static function main() {
    var args = Sys.args();
    if (args.length != 1) throw "usage: hl main.hl <output dir>";
    var dir = args[0];
    sys.FileSystem.createDirectory(dir);
    var machine = new FlexsplineHobber();
    var poses = machine.solvedPoses(), back = F.inverse(FlexsplineHobber.MACHINE_FROM_WORK), seatZ = HobberZLayout.SEAT_Z;
    var partList:Array<Dynamic> = [];
    var out:Dynamic = {parts: partList};
    function at(id:String, x:Float, y:Float, z:Float) return F.transformPoint(F.compose(back, poses.get(id)), x, y, z);
    function member(designation:String):MachineComponent {
      for (entry in machine.components()) if (entry.component.designation == designation) return entry.component;
      throw "no part " + designation;
    }
    for (design in ["GE-X-carriage", "GE-Z-saddle", "GE-spindle-seat", "GE-motor-cradle"]) {
      var saddle = cast(member("HOBBER-" + design), HobberSaddle);
      var geo = saddle.geometry(), entryId = "";
      for (entry in machine.components()) if (entry.component == cast saddle) entryId = entry.id;
      var o = at(entryId, 0, 0, 0), ax = at(entryId, 1, 0, 0), ay = at(entryId, 0, 1, 0), az = at(entryId, 0, 0, 1);
      var pose = {id: entryId, origin: [r(o.x), r(o.y), r(o.z - seatZ)], axes: [[r(ax.x - o.x, 6), r(ax.y - o.y, 6), r(ax.z - o.z, 6)], [r(ay.x - o.x, 6), r(ay.y - o.y, 6), r(ay.z - o.z, 6)],
        [r(az.x - o.x, 6), r(az.y - o.y, 6), r(az.z - o.z, 6)]]};
      geo.shape.exportStep('$dir/$design.step');
      var props = saddle.massProperties();
      var holes = [for (h in saddle.holes()) {id: h.id, kind: h.kind, thread: h.thread, diameter: r(h.diameter), depth: r(h.depth),
        position: [for (v in h.position) r(v)], direction: [for (v in h.direction) r(v, 6)], slotLength: h.slotLength == null ? 0 : r(h.slotLength)}];
      partList.push({name: design, pose: pose, volume: geo.volume(), mass: props.mass, comLocal: [props.centreOfMass.x, props.centreOfMass.y, props.centreOfMass.z], holes: holes});
      geo.close();
      Sys.println("exported " + design);
    }
    // Component poses: where each part's authored frame lies in the work frame (to see the offset to the authored frame).
    // Servo group on the cradle: exactly the members and weights HobberFrameChecks.spindleDrive uses.
    var ids = ["spindle/motor", "spindle/motorShaft", "spindle/motorPulley", "spindle/motorEncoder", "motorCradle"], servo = {mass: 0.0, x: 0.0, y: 0.0, z: 0.0}, parts:Array<Dynamic> = [];
    for (m in machine.components()) {
      if (ids.indexOf(m.id) < 0) continue;
      var weight = m.id == "motorCradle" ? 1.0 / 3 : 1.0, props = m.component.massProperties(), p = at(m.id, props.centreOfMass.x, props.centreOfMass.y, props.centreOfMass.z);
      servo.mass += weight * props.mass;
      servo.x += weight * props.mass * p.x;
      servo.y += weight * props.mass * p.y;
      servo.z += weight * props.mass * (p.z - seatZ);
      parts.push({id: m.id, mass: props.mass, weight: weight, com: [r(p.x), r(p.y), r(p.z - seatZ)]});
    }
    var cradle = member("HOBBER-GE-motor-cradle").massProperties();
    var cc = at("motorCradle", cradle.centreOfMass.x, cradle.centreOfMass.y, cradle.centreOfMass.z);
    out.servo = {mass: servo.mass, com: [r(servo.x / servo.mass), r(servo.y / servo.mass), r(servo.z / servo.mass)], members: parts,
      cradleComWork: [r(cc.x), r(cc.y), r(cc.z - seatZ)], cradleComAuthored: [r(cradle.centreOfMass.x), r(cradle.centreOfMass.y), r(cradle.centreOfMass.z)]};
    // Head on the A flange.
    var def = machine.definition(), turning = new Map<String, Bool>();
    turning.set("swivel/output", true);
    var changed = true;
    while (changed) {
      changed = false;
      for (j in def.joints) if (turning.exists(j.parent) && !turning.exists(j.child)) {
        turning.set(j.child, true);
        changed = true;
      }
    }
    var mass = 0.0, mx = 0.0, my = 0.0, mz = 0.0;
    for (m in machine.components()) if (turning.exists(m.id)) {
      var props = m.component.massProperties(), p = at(m.id, props.centreOfMass.x, props.centreOfMass.y, props.centreOfMass.z);
      mass += props.mass;
      mx += props.mass * p.x;
      my += props.mass * p.y;
      mz += props.mass * p.z;
    }
    var tool = F.transformPoint(F.compose(back, F.compose(poses.get("hob"), machine.hob.connector("centre").frame)), 0, 0, 0);
    out.head = {mass: mass, com: [r(mx / mass), r(my / mass), r(mz / mass - seatZ)], outputFaceX: HobberSwivelLayout.OUTPUT_X, hob: [r(tool.x), r(tool.y), r(tool.z - seatZ)]};
    out.seatZ = seatZ;
    // Z moving mass (everything on the Z joint) for the saddle's inertial loads.
    var moving = new Map<String, Bool>();
    moving.set("blockZLeft", true);
    changed = true;
    while (changed) {
      changed = false;
      for (j in def.joints) if (moving.exists(j.parent) && !moving.exists(j.child)) {
        moving.set(j.child, true);
        changed = true;
      }
    }
    var zm = 0.0, zx = 0.0, zy = 0.0, zz = 0.0;
    for (m in machine.components()) if (moving.exists(m.id)) {
      var props = m.component.massProperties(), p = at(m.id, props.centreOfMass.x, props.centreOfMass.y, props.centreOfMass.z);
      zm += props.mass;
      zx += props.mass * p.x;
      zy += props.mass * p.y;
      zz += props.mass * p.z;
    }
    out.zMoving = {mass: zm, com: [r(zx / zm), r(zy / zm), r(zz / zm - seatZ)]};
    sys.io.File.saveContent('$dir/hobber-fem-export.json', haxe.Json.stringify(out, null, "  "));
    Sys.println("wrote " + dir + "/hobber-fem-export.json");
  }
}
