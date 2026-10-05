import processkit.perception.ContactPoseEnvelope;
import processkit.perception.ContactRegistration.PlaneContact;
import robotkit.spatial.Transform3;
import robotkit.spatial.Quat;
import robotkit.spatial.Vec3;

class ContactPoseEnvelopeTests {
  static var checks = 0;
  static function check(ok:Bool, message:String):Void { checks++; if (!ok) throw message; }
  static function samples(actual:Transform3, noise:Float):Array<PlaneContact> {
    var points = [new Vec3(-0.1, 0.004, 0.02), new Vec3(0.1, 0.004, 0.02), new Vec3(0.1, 0.004, 0.08),
      new Vec3(-0.04, -0.03, 0.01), new Vec3(0.04, -0.03, 0.01), new Vec3(0.09, -0.03, 0.05)];
    var normals = [new Vec3(0, 1, 0), new Vec3(0, 1, 0), new Vec3(0, 1, 0), new Vec3(0, 0, 1), new Vec3(0, 0, 1), new Vec3(1, 0, 0)];
    return [for (i in 0...points.length) new PlaneContact(normals[i], normals[i].dot(points[i]),
      actual.transformPoint(points[i]).add(actual.rotation.rotate(normals[i]).scale(i % 2 == 0 ? noise : -noise)))];
  }
  static function encloses(envelope:ContactPoseEnvelope, actual:Transform3, estimate:Transform3):Void {
    var point = new Vec3(0.29, 0.03, 0.04), normal = new Vec3(0, 1, 0), u = new Vec3(1, 0, 0), v = new Vec3(0, 0, 1);
    var bound = envelope.region(point, normal, u, v, estimate);
    var located = actual.inverse().transformPoint(estimate.transformPoint(point));
    var direction = actual.rotation.conjugate().rotate(estimate.rotation.rotate(normal));
    var travel = normal.dot(point.sub(located)) / normal.dot(direction);
    var delta = located.add(direction.scale(travel)).sub(point);
    check(Math.abs(delta.dot(u)) <= bound.halfU + 1e-10 && Math.abs(delta.dot(v)) <= bound.halfV + 1e-10 &&
      Math.abs(travel) <= bound.normalTravel + 1e-10, "The measured pose envelope encloses the real plane intersection");
  }
  public static function run():Void {
    var nominal = new Transform3(new Vec3(1.7, -0.2, 0.6), Quat.fromRollPitchYaw(0, 0, 0.7));
    for (x in [-0.02, 0.02]) for (y in [-0.02, 0.02]) for (yaw in [-2.0, 2.0]) {
      var error = new Transform3(new Vec3(x, y, 0), Quat.fromRollPitchYaw(0, 0, yaw * Math.PI / 180));
      var actual = error.inverse().compose(nominal);
      var contacts = samples(actual, 0.000005);
      var envelope = new ContactPoseEnvelope(nominal, new Vec3(0.02, 0.02, 0), new Vec3(0, 0, 2 * Math.PI / 180), 0.00001);
      encloses(envelope, actual, nominal);
      envelope.refine(contacts.slice(0, 3));
      var estimate = envelope.estimate();
      encloses(envelope, actual, estimate);
      var remaining = envelope.region(new Vec3(0.29, 0.03, 0.04), new Vec3(0, 1, 0), new Vec3(1, 0, 0), new Vec3(0, 0, 1), estimate);
      check(remaining.normalTravel < 0.003, 'Measured first-plane normal uncertainty is reduced (${remaining.normalTravel})');
      envelope.refine(contacts);
      estimate = envelope.estimate();
      encloses(envelope, actual, estimate);
      check(estimate.transformPoint(new Vec3(0.2, -0.03, 0.05)).sub(actual.transformPoint(new Vec3(0.2, -0.03, 0.05))).norm() < 0.001,
        "Additional independent contacts reduce the provisional pose error");
    }
    var interiorActual = new Transform3(new Vec3(), Quat.fromRollPitchYaw(0, 0, 0.03)).inverse().compose(nominal);
    var interior = new ContactPoseEnvelope(nominal, new Vec3(0.02, 0.02, 0), new Vec3(0, 0, 0.04), 0.00001);
    interior.refine(samples(interiorActual, 0.000005).slice(0, 3));
    var unresolved = interior.region(new Vec3(0.29, 0.03, 0.04), new Vec3(0, 1, 0), new Vec3(1, 0, 0), new Vec3(0, 0, 1), interior.estimate());
    check(unresolved.halfU > 0.01, "Unobserved in-plane displacement remains bounded rather than assumed absent");
    encloses(interior, interiorActual, interior.estimate());
    var spatialError = new Transform3(new Vec3(0.01, -0.015, 0.008), Quat.fromRollPitchYaw(0.01, -0.015, 0.025));
    var spatialActual = spatialError.inverse().compose(nominal);
    var spatial = new ContactPoseEnvelope(nominal, new Vec3(0.02, 0.02, 0.02), new Vec3(0.03, 0.03, 0.04), 0.00001);
    encloses(spatial, spatialActual, nominal);
    spatial.refine(samples(spatialActual, 0.000005).slice(0, 3), 128, 32);
    encloses(spatial, spatialActual, spatial.estimate());
    spatial.refine(samples(spatialActual, 0.000005), 4096, 256);
    encloses(spatial, spatialActual, spatial.estimate());
    spatial.refine(samples(spatialActual, 0.000005), 0, 1);
    check(spatial.cells == 1, "Reducing the cell budget merges feasible regions conservatively");
    encloses(spatial, spatialActual, spatial.estimate());
    var boundary = new ContactPoseEnvelope(nominal, new Vec3(), new Vec3(), 0.00001);
    boundary.refine(samples(nominal, 0.00001));
    encloses(boundary, nominal, boundary.estimate());
    var coarse = new ContactPoseEnvelope(nominal, new Vec3(0.02, 0.02, 0), new Vec3(0, 0, 0.04), 0.00001);
    var actual = new Transform3(new Vec3(0.01, -0.02, 0), Quat.fromRollPitchYaw(0, 0, 0.03)).inverse().compose(nominal);
    coarse.refine(samples(actual, 0), 0, 1);
    encloses(coarse, actual, coarse.estimate());
    check(coarse.cells == 1, "An exhausted subdivision budget retains a conservative unsplit cell");
    var bad = new ContactPoseEnvelope(nominal, new Vec3(0.02, 0.02, 0), new Vec3(0, 0, 0.04), 0.00001);
    var outside = new Transform3(new Vec3(0, 0, 0.01), Quat.identity()).compose(nominal);
    var rejected = false;
    try bad.refine(samples(outside, 0)) catch (_:Dynamic) rejected = true;
    check(rejected, "Observations inconsistent with the declared chassis error are rejected");
    Sys.println('Contact pose envelope: $checks assertions passed');
  }
}
