import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import cadkit.modeling.Location;
import machinekit.welding.WeldProbeGeometry;
import machinekit.welding.WeldProbeGeometry.WeldProbeFace;
import materia.assembly.AssemblyFrames;
import machinekit.welding.WeldProbeParkingBounds;

class WeldProbeTests {
  static var checks = 0;
  static function check(ok:Bool, reason:String):Void { checks++; if (!ok) throw reason; }
  static function main():Void {
    var plate = Part.box(100, 80, 10);
    var cutter = Part.box(20, 20, 20);
    var pierced = plate.subtract(cutter);
    var parts:Map<String, Part> = new Map(); parts.set("plate", pierced);
    var geometry = WeldProbeGeometry.of(parts, ["plate"]);
    var tops = [for (face in geometry.faces) if (face.normal.z > 0.9) face];
    check(tops.length == 1, "A planar pierced CAD plate supplies one top patch");
    var top = tops[0];
    check(!top.contains(new Vector(0, 0, 10)), "CAD holes are excluded from touch regions");
    check(top.contains(new Vector(30, 0, 10), 5), "Material away from the hole is a usable patch");
    check(!top.contains(new Vector(11, 0, 10), 2), "Inset excludes hole boundaries");
    check(!top.contains(new Vector(49, 0, 10), 2), "Inset excludes outer boundaries");
    check(top.containsRegion(new Vector(30, 0, 10), 5, 5, 2), "A complete uncertainty region can stay inside material");
    check(!top.containsRegion(new Vector(15, 0, 10), 25, 30, 0), "A region enclosing a hole is rejected even with material at all corners");
    check(!top.containsRegion(new Vector(30, 0, 10), 5, 25, 0), "A region crossing the outside boundary is rejected");
    var curved = Part.cylinder(10, 20);
    var curvedParts:Map<String, Part> = new Map(); curvedParts.set("curved", curved);
    var rejected = false;
    try WeldProbeGeometry.of(curvedParts, ["curved"]) catch (_:Dynamic) rejected = true;
    check(rejected, "Unsupported curved geometry is rejected rather than silently omitted from visibility");
    curved.close();
    var root = AssemblyFrames.translation(0, -1700, 10);
    var uncertainty = new WeldProbeParkingBounds(root, new Vector(20, 20, 0), 2 * Math.PI / 180);
    var region = uncertainty.region(top, new Vector(30, 0, 10));
    check(region.normalTravel < 1e-9, "Planar parking error does not invent vertical uncertainty");
    check(Math.max(region.halfU, region.halfV) > 70, "Parking yaw includes the CAD-derived chassis lever arm");
    check(!uncertainty.fits(top, new Vector(30, 0, 10), 1), "A narrow patch cannot accept the full parking envelope");
    var noError = new WeldProbeParkingBounds(root, new Vector(), 0);
    check(noError.fits(top, new Vector(30, 0, 10), 1), "A zero uncertainty envelope reduces to the nominal material patch");
    var samples = top.samples(9, 2);
    check(samples.length > 10, "Probe samples derive from actual face bounds");
    for (point in samples) check(top.contains(point, 2), "Every generated sample remains within its CAD patch");
    var clampLocal = Part.box(10, 10, 20);
    var clamp = clampLocal.placed(Location.translation(new Vector(30, 0, 10)));
    parts.set("clamp", clamp);
    geometry = WeldProbeGeometry.of(parts, ["plate"]);
    top = [for (face in geometry.faces) if (face.member == "plate" && face.normal.z > 0.9) face][0];
    check(!geometry.exposed(top, new Vector(30, 0, 10), 30), "A mating clamp blocks a nominal probe ray");
    check(geometry.exposed(top, new Vector(-30, 0, 10), 30), "An unobstructed probe ray stays usable");
    for (face in geometry.faces) if (face.member == "clamp") check(!face.target, "Fixture geometry is not a registration target");
    clamp.close(); clampLocal.close(); pierced.close(); cutter.close(); plate.close();

    var cell = new MobileWelderCell();
    var weldment = cell.weldment();
    var poses = cell.solvedPoses();
    var work = WeldProbeGeometry.findIn(cell, weldment, poses, ["table", "clamp"]);
    var useful = [for (face in work.faces) if (face.target) face];
    check(useful.length >= 12, "Actual mobile weldment supplies polygonal contact patches");
    var chassis = AssemblyFrames.translation(-500, -1700, 0);
    var envelope = new WeldProbeParkingBounds(chassis, new Vector(20, 20, 0), 2 * Math.PI / 180);
    var nominalInverse = AssemblyFrames.inverse(chassis);
    for (face in useful) {
      var p = face.centre;
      var bound = envelope.region(face, p);
      for (x in [-20.0, 20.0]) for (y in [-20.0, 20.0]) for (i in 0...17) {
        var angle = (i / 16 * 4 - 2) * Math.PI / 180;
        var error:materia.assembly.AssemblyRecord.AssemblyFrame = {x: x, y: y, z: 0, qx: 0, qy: 0,
          qz: Math.sin(angle / 2), qw: Math.cos(angle / 2)};
        var changed = AssemblyFrames.compose(chassis, AssemblyFrames.compose(error, nominalInverse));
        var actual = AssemblyFrames.transformPoint(changed, p.x, p.y, p.z);
        var out = AssemblyFrames.transformVector(changed, face.normal.x, face.normal.y, face.normal.z);
        var outward = new Vector(out.x, out.y, out.z);
        var at = new Vector(actual.x, actual.y, actual.z);
        var travel = (face.offset() - face.normal.dot(at)) / face.normal.dot(outward);
        var delta = at.add(outward.scale(travel)).subtract(p);
        check(Math.abs(delta.dot(face.u)) <= bound.halfU + 1e-7 && Math.abs(delta.dot(face.v)) <= bound.halfV + 1e-7 &&
          Math.abs(travel) <= bound.normalTravel + 1e-7, "Continuous parking bounds enclose the displaced probe-plane intersection");
      }
    }
    var exposed = 0;
    for (face in useful) for (point in face.samples(5, 1)) if (work.exposed(face, point, 100)) exposed++;
    check(exposed > 20, "The actual workpiece has exposed CAD-derived contact samples");
    var displaced:Map<String, materia.assembly.AssemblyRecord.AssemblyFrame> = new Map();
    var shift:materia.assembly.AssemblyRecord.AssemblyFrame = {x: 12, y: -8, z: 3, qx: 0, qy: 0,
      qz: Math.sin(0.3 / 2), qw: Math.cos(0.3 / 2)};
    for (id in poses.keys()) displaced.set(id, AssemblyFrames.compose(shift, poses.get(id)));
    var moved = WeldProbeGeometry.findIn(cell, weldment, displaced, ["table", "clamp"]);
    check(moved.faces.length == work.faces.length, "Global displacement preserves contact-patch count");
    for (i in 0...work.faces.length) {
      check(moved.faces[i].name() == work.faces[i].name(), "CAD face identity is deterministic");
      check(moved.faces[i].centre.subtract(work.faces[i].centre).length() < 1e-7,
        "Registration geometry remains in the nominal weldment reference frame");
    }
    Sys.println('Weld probe CAD geometry: $checks assertions passed, $exposed exposed samples');
  }
}
