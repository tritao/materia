import processkit.perception.ContactRegistration;
import processkit.perception.ContactRegistration.PlaneContact;
import robotkit.spatial.Vec3;
import robotkit.spatial.Quat;
import robotkit.spatial.Transform3;

class ContactRegistrationTests {
  static var checks:Int = 0;
  static function check(value:Bool, message:String):Void {
    checks++;
    if (!value) throw message;
  }
  static function contacts(actual:Transform3):Array<PlaneContact> {
    var points = [new Vec3(-0.1, -0.1, 0), new Vec3(0.1, -0.1, 0), new Vec3(0.1, 0.1, 0),
      new Vec3(-0.1, 0.02, 0.05), new Vec3(0.1, 0.02, 0.05), new Vec3(0.04, -0.04, 0.05)];
    var normals = [new Vec3(0, 0, 1), new Vec3(0, 0, 1), new Vec3(0, 0, 1),
      new Vec3(0, 1, 0), new Vec3(0, 1, 0), new Vec3(1, 0, 0)];
    return [for (i in 0...points.length) new PlaneContact(normals[i], normals[i].dot(points[i]), actual.transformPoint(points[i]))];
  }
  static function close(found:Transform3, expected:Transform3):Void {
    for (point in [new Vec3(), new Vec3(0.2, -0.1, 0.3), new Vec3(-0.1, 0.2, 0.1)])
      check(found.transformPoint(point).sub(expected.transformPoint(point)).norm() < 1e-7, "Registration recovers the full rigid frame");
  }
  public static function main():Void {
    var nominal = new Transform3(new Vec3(1.7, -0.2, 0.6), Quat.fromRollPitchYaw(0.1, -0.1, 0.7));
    for (x in [-0.02, 0.0, 0.02]) for (y in [-0.02, 0.02]) for (yaw in [-2.0, 2.0]) {
      var correction = new Transform3(new Vec3(x, y, 0.01), Quat.fromRollPitchYaw(0.01, -0.015, yaw * Math.PI / 180));
      var actual = nominal.compose(correction);
      var result = ContactRegistration.fit(contacts(actual), nominal);
      check(result.accepted && result.rank == 6, 'Three-face fit is accepted: ${result.reason}');
      close(result.require(), actual);
      check(result.maxResidual < 1e-8, "Exact contacts have negligible residual");
    }
    // Parking rotates about the chassis, so the distant work origin moves by more than 20 mm.
    for (x in [-0.02, 0.02]) for (y in [-0.02, 0.02]) for (yaw in [-2.0, 2.0]) {
      var parking = new Transform3(new Vec3(x, y, 0), Quat.fromRollPitchYaw(0, 0, yaw * Math.PI / 180));
      var observed = parking.inverse().compose(nominal);
      var result = ContactRegistration.fit(contacts(observed), nominal);
      check(result.accepted, "Chassis-origin parking uncertainty is recoverable");
      close(result.require(), observed);
    }
    var actual = nominal.compose(new Transform3(new Vec3(0.01, -0.02, 0.01), Quat.fromRollPitchYaw(0, 0, 0.03)));
    var samples = contacts(actual);
    var plane = ContactRegistration.fit(samples.slice(0, 3), nominal);
    check(!plane.accepted && plane.rank == 3, "One face cannot authorize a complete work frame");
    var refused = false;
    try plane.require() catch (_:Dynamic) refused = true;
    check(refused, "Provisional frames cannot be used as accepted registration");
    var two = ContactRegistration.fit(samples.slice(0, 5), nominal);
    check(!two.accepted && two.rank == 5, "Two faces leave translation along their intersection unobserved");
    var extra = samples.copy();
    extra.push(new PlaneContact(samples[0].normal, samples[0].offset, samples[0].measured.add(actual.rotation.rotate(new Vec3(0, 0, 0.003)))));
    check(!ContactRegistration.fit(extra, nominal).accepted, "Contradictory contacts are rejected");
    check(!ContactRegistration.fit(samples, nominal, 0.0005, 0.005).accepted, "Corrections outside the configured translation bound are rejected");
    check(!ContactRegistration.fit(samples, nominal, 0.0005, 0.15, 0.01).accepted, "Corrections outside the configured angular bound are rejected");
    var observer = new Transform3(new Vec3(-4, 2, 1), Quat.fromRollPitchYaw(0.2, -0.3, 1.1));
    close(ContactRegistration.fit(contacts(observer.compose(actual)), observer.compose(nominal)).require(), observer.compose(actual));
    var scaled = [for (sample in samples) new PlaneContact(sample.normal.scale(3), sample.offset * 3, sample.measured)];
    close(ContactRegistration.fit(scaled, nominal).require(), actual);
    var duplicate = [for (_ in 0...6) samples[0]];
    check(!ContactRegistration.fit(duplicate, nominal).accepted, "Repeated contacts do not create observability");
    check(!ContactRegistration.fit(samples, nominal, 0.0005, 0.15, 0.1, 1).accepted, "An exhausted iteration budget is not accepted");
    var bad = false;
    try ContactRegistration.fit([], nominal) catch (_:Dynamic) bad = true;
    check(bad, "Empty observations are rejected");
    var missing = false;
    try ContactRegistration.fit([null], nominal) catch (_:Dynamic) missing = true;
    check(missing, "Missing observations are rejected");
    var invalidPlane = false;
    try new PlaneContact(new Vec3(), 0, new Vec3()) catch (_:Dynamic) invalidPlane = true;
    check(invalidPlane, "A zero normal cannot describe a contact plane");
    Sys.println('Contact registration: $checks assertions passed');
  }
}
