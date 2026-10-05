import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import cadkit.modeling.Location;
import machinekit.welding.WeldProbeGeometry;
import machinekit.welding.WeldProbeGeometry.WeldProbeFace;
import materia.assembly.AssemblyFrames;
import machinekit.welding.WeldProbeParkingBounds;
import machinekit.welding.WeldProbePatterns;
import machinekit.welding.WeldProbeUncertainty.WeldProbeObservedBounds;
import machinekit.welding.WeldProbeParkingBounds.WeldProbeRegionBounds;
import processkit.perception.ContactPoseEnvelope;
import processkit.perception.ContactRegistration;
import processkit.perception.ContactRegistration.PlaneContact;
import robotkit.spatial.Transform3;
import robotkit.spatial.Vec3;
import robotkit.spatial.Quat;

class WeldProbeTests {
  static var checks = 0;
  static function check(ok:Bool, reason:String):Void { checks++; if (!ok) throw reason; }
  static function metres(point:Vector):Vec3 return new Vec3(point.x * 0.001, point.y * 0.001, point.z * 0.001);
  static function direction(point:Vector):Vec3 return new Vec3(point.x, point.y, point.z);
  static function observedBounds(envelope:ContactPoseEnvelope):WeldProbeObservedBounds {
    var estimate = envelope.estimate();
    return new WeldProbeObservedBounds((face, point) -> {
      var bound = envelope.region(metres(point), direction(face.normal), direction(face.u), direction(face.v), estimate);
      return new WeldProbeRegionBounds(bound.halfU * 1000, bound.halfV * 1000, bound.normalTravel * 1000, bound.tiltU, bound.tiltV);
    });
  }
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
    check(!geometry.exposedRegion(top, new Vector(30, 15, 10), 12, 5, 30),
      "An off-centre clamp masks the uncertain region even when its centre ray is clear");
    check(geometry.exposedRegion(top, new Vector(-30, 0, 10), 5, 5, 30), "A clear full approach region remains usable");
    for (face in geometry.faces) if (face.member == "clamp") check(!face.target, "Fixture geometry is not a registration target");
    clamp.close(); clampLocal.close(); pierced.close(); cutter.close(); plate.close();

    var cell = new MobileWelderCell();
    var weldment = cell.weldment();
    var poses = cell.solvedPoses();
    var work = WeldProbeGeometry.findIn(cell, weldment, poses, ["table", "clamp"]);
    var useful = [for (face in work.faces) if (face.target) face];
    var savedFaces = work.contactFaces(0.001);
    materia.project.SceneContactRegistration.validate({frame: weldment.reference, nominal: AssemblyFrames.identity(),
      translation: [0.02, 0.02, 0.0], rotation: [0.0, 0.0, 2 * Math.PI / 180],
      measurementError: 0.00001, contactOffset: 0.0005, faces: savedFaces},
      [for (face in work.faces) face.member].concat([weldment.reference]));
    var reconstructed = WeldProbeGeometry.fromContactFaces(savedFaces, 0.001);
    check(reconstructed.faces.length == work.faces.length, "Saved nominal CAD faces preserve fixtures and registration targets");
    for (i in 0...work.faces.length) {
      var original = work.faces[i], copy = reconstructed.faces[i];
      check(original.name() == copy.name() && original.target == copy.target && original.centre.subtract(copy.centre).length() < 1e-7 &&
        original.chords.length == copy.chords.length, "Saved contact geometry preserves CAD identity, units and contours");
      for (point in original.samples(5, 1)) check(copy.contains(point, 1), "Reconstructed contact geometry accepts the same CAD material samples");
    }
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
    var firstStages = WeldProbePatterns.stages(work, 3, [], envelope);
    check(WeldProbePatterns.stages(work, 3, [], envelope, 3, 9, (_, _, _) -> false).length == 0,
      "A contact pattern cannot reuse points excluded by arm configuration screening");
    check(firstStages.length > 0, "Actual CAD supplies first-plane patterns under the complete parking envelope");
    for (stage in firstStages) {
      check(stage.points.length == 3, "First-plane patterns contain three contacts");
      check(stage.points[1].subtract(stage.points[0]).cross(stage.points[2].subtract(stage.points[0])).length() > 1,
        "First-plane contacts are noncollinear");
      for (point in stage.points) check(envelope.fits(stage.face, point, 3), "Each first contact contains the full parking uncertainty");
    }
    // Later stages use a measured feasible set, never a fabricated zero-error parking pose.
    var nominalWork = new Transform3(new Vec3(0.5, 1.7, 0), Quat.identity());
    var measuredEnvelope = new ContactPoseEnvelope(nominalWork, new Vec3(0.02, 0.02, 0),
      new Vec3(0, 0, 2 * Math.PI / 180), 0.00001);
    var parkingError = new Transform3(new Vec3(0.011, -0.014, 0), Quat.fromRollPitchYaw(0, 0, 0.025));
    var measuredWork = parkingError.inverse().compose(nominalWork);
    var measuredContacts:Array<PlaneContact> = [];
    var measuredNormals:Array<Vector> = [];
    for (count in [3, 2, 1]) {
      var provider = observedBounds(measuredEnvelope);
      var options = WeldProbePatterns.stages(work, count, measuredNormals, provider, 3, 9);
      check(options.length > 0, 'Measured uncertainty permits a CAD-derived $count-contact stage');
      var stage = options[0];
      for (point in stage.points) {
        var bound = provider.region(stage.face, point);
        var normal = direction(stage.face.normal);
        var command = measuredEnvelope.estimate().transformPoint(metres(point));
        var ray = measuredWork.inverse().rotation.rotate(measuredEnvelope.estimate().rotation.rotate(normal));
        var located = measuredWork.inverse().transformPoint(command);
        var distance = (stage.face.offset() * 0.001 - normal.dot(located)) / normal.dot(ray);
        var delta = located.add(ray.scale(distance)).sub(metres(point));
        check(Math.abs(delta.dot(direction(stage.face.u))) * 1000 <= bound.halfU + 1e-6 &&
          Math.abs(delta.dot(direction(stage.face.v))) * 1000 <= bound.halfV + 1e-6 &&
          Math.abs(distance) * 1000 <= bound.normalTravel + 1e-6,
          "Measured CAD region encloses the physical probe intersection after recentering");
        measuredContacts.push(new PlaneContact(normal, stage.face.offset() * 0.001,
          measuredWork.transformPoint(metres(point))));
      }
      measuredNormals.push(stage.face.normal);
      measuredEnvelope.refine(measuredContacts, 4096, 32);
    }
    var measuredFit = ContactRegistration.fit(measuredContacts, nominalWork);
    check(measuredFit.accepted && measuredFit.rank == 6, "Measured-bound CAD stages yield a fully observable registration");
    var reduced = new WeldProbeParkingBounds(chassis, new Vector(), 0);
    var firstNormal = firstStages[0].face.normal;
    var secondStages = WeldProbePatterns.stages(work, 2, [firstNormal], reduced);
    check(secondStages.length > 0, "Independent second-plane patterns are available after uncertainty is reduced");
    for (stage in secondStages) check(Math.abs(stage.points[1].subtract(stage.points[0]).dot(firstNormal.cross(stage.face.normal))) >= 3,
      "Second-plane contacts observe the remaining rotational component");
    var thirdStages = WeldProbePatterns.stages(work, 1, [firstNormal, secondStages[0].face.normal], reduced);
    check(thirdStages.length > 0, "Third-plane candidates complete the independent three-plane basis");
    for (stage in thirdStages) check(Math.abs(firstNormal.cross(secondStages[0].face.normal).dot(stage.face.normal)) > 1e-3,
      "Third-plane normals observe the remaining translation");
    var actual = new Transform3(new Vec3(0.02, -0.02, 0), Quat.fromRollPitchYaw(0, 0, 2 * Math.PI / 180));
    var observations:Array<PlaneContact> = [];
    for (stage in [firstStages[0], secondStages[0], thirdStages[0]]) for (point in stage.points) {
      var normal = stage.face.normal;
      observations.push(new PlaneContact(new Vec3(normal.x, normal.y, normal.z), stage.face.offset() * 0.001,
        actual.transformPoint(new Vec3(point.x * 0.001, point.y * 0.001, point.z * 0.001))));
    }
    var fitted = ContactRegistration.fit(observations, Transform3.identity());
    check(fitted.accepted && fitted.rank == 6, "The CAD-selected 3-2-1 contacts observe all six rigid pose components");
    var testPoint = new Vec3(0.4, -0.1, 0.05);
    check(fitted.require().transformPoint(testPoint).sub(actual.transformPoint(testPoint)).norm() < 1e-7,
      "CAD-selected contact constraints recover a displaced rigid work frame");
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
