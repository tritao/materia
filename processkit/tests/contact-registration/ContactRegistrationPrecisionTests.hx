import processkit.perception.ContactRegistrationPrecision;
import processkit.perception.ContactRegistration;
import processkit.perception.ContactRegistration.PlaneContact;
import processkit.ContactProbeRunner.ContactProbeRequest;
import robotkit.spatial.Vec3;
import robotkit.spatial.Transform3;

class ContactRegistrationPrecisionTests {
  static var checks = 0;
  static function check(ok:Bool, reason:String):Void { checks++; if (!ok) throw reason; }
  public static function run():Void {
    var targets = [for (x in [0.0, 0.35]) for (y in [-0.07, 0.07]) for (z in [0.0, 0.4]) new Vec3(x, y, z)];
    var precision = new ContactRegistrationPrecision(targets);
    var normals = [new Vec3(0, 0, 1), new Vec3(0, 1, 0), new Vec3(1, 0, 0)];
    var stages = [[new Vec3(), new Vec3(0.011, 0, 0), new Vec3(0, 0.0045, 0)],
      [new Vec3(0.2, 0.03, 0.04), new Vec3(0.25, 0.03, 0.04)], [new Vec3(0.19, 0.02, 0.09)]];
    var errors = [for (i in 0...3) precision.contactError(stages[i], normals[i], normals.slice(0, i), 0.00001)];
    var broad = precision.contactError([new Vec3(), new Vec3(0.35, 0, 0), new Vec3(0, 0.14, 0)],
      normals[0], [], 0.00001);
    check(errors[0] < broad && errors[0] < 0.000001, "A narrow triangle needs finer sensing over the complete work extent");
    // All sign combinations exercise the rigid fit, independently of the budget's calculation.
    for (mask in 0...64) {
      var contacts:Array<PlaneContact> = [];
      var index = 0;
      for (stage in 0...3) for (point in stages[stage]) {
        var noise = errors[stage] * ((mask & (1 << index)) == 0 ? -1 : 1);
        contacts.push(new PlaneContact(normals[stage], normals[stage].dot(point), point.add(normals[stage].scale(noise))));
        index++;
      }
      var fitted = ContactRegistration.fit(contacts, Transform3.identity()).require();
      for (target in targets)
        check(fitted.transformPoint(target).sub(target).norm() <= ContactRegistrationPrecision.DEFAULT_WORK_ERROR,
          "Bounded contact errors must preserve the work-point accuracy through the rank-six rigid fit");
    }
    var request = new ContactProbeRequest(Transform3.identity(), new Vec3(0, 0, -1), 0.1, 0.01,
      0.0005, 0.002, 0.0005);
    var calibratedError = errors[0] * 0.5;
    var refined = request.withPrecision(errors[0], calibratedError, 0.02);
    check(refined.fineSpeed * 0.02 <= errors[0] - calibratedError + 1e-15,
      "Fine speed keeps worst-case sampling and feedback skew within the remaining contact budget");
    check(refined.calibratedError == calibratedError,
      "Contact measurement records its calibrated sensor bound separately from timing uncertainty");
    check(refined.backoff > request.contactOffset + request.coarseSpeed * 0.04 && refined.backoff < request.backoff,
      "Fine entry clears the bounded coarse observation without a long precision search");
    var rejected = false;
    try precision.contactError([new Vec3(), new Vec3(), new Vec3()], normals[0], [], 0.00001)
      catch (_:Dynamic) rejected = true;
    check(rejected, "A degenerate triangle cannot receive a precision budget");
    rejected = false;
    try request.withPrecision(0, 0, 0.02) catch (_:Dynamic) rejected = true;
    check(rejected, "Zero contact precision is rejected");
    rejected = false;
    try request.withPrecision(0.00001, 0.00001, 0.02) catch (_:Dynamic) rejected = true;
    check(rejected, "Calibrated error cannot consume the whole budget and leave no room for acquisition age");
    Sys.println('Contact registration precision: $checks assertions passed');
  }
}
