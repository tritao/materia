import processkit.perception.ContactRegistrationSequence;
import processkit.perception.ContactRegistrationSequence.ContactRegistrationStage;
import processkit.perception.ContactPoseEnvelope;
import processkit.ContactProbeRunner.ContactProbeRequest;
import robotkit.spatial.Vec3;
import robotkit.spatial.Transform3;
import robotkit.spatial.Quat;

class ContactRegistrationSequenceTests {
  static var checks = 0;
  static function check(ok:Bool, message:String):Void { checks++; if (!ok) throw message; }
  static function request(point:Vec3):ContactProbeRequest
    return new ContactProbeRequest(new Transform3(point, Quat.identity()), new Vec3(0, 0, -1), 0.1);
  public static function run():Void {
    var points = [new Vec3(-0.1, -0.1, 0), new Vec3(0.1, -0.1, 0), new Vec3(0.1, 0.1, 0),
      new Vec3(-0.1, 0.02, 0.05), new Vec3(0.1, 0.02, 0.05), new Vec3(0.04, -0.04, 0.05)];
    var planeNormals = [new Vec3(0, 0, 1), new Vec3(0, 1, 0), new Vec3(1, 0, 0)];
    var nominal = new Transform3(new Vec3(1.7, -0.2, 0.6), Quat.fromRollPitchYaw(0, 0, 0.7));
    var error = new Transform3(new Vec3(0.012, -0.009, 0), Quat.fromRollPitchYaw(0, 0, 0.025));
    var actual = error.inverse().compose(nominal);
    var envelope = new ContactPoseEnvelope(nominal, new Vec3(0.02, 0.02, 0), new Vec3(0, 0, 0.04), 0.00001);
    var calls:Array<Int> = [];
    var sequence = new ContactRegistrationSequence(envelope, (count, normals, bounds) -> {
      calls.push(count);
      check(normals.length == 3 - count && bounds == envelope, "Selection receives prior independent normals and measured bounds");
      if (count < 3) check(bounds.cells > 1, "Later stages receive refined uncertainty rather than a replaced zero prior");
      var start = count == 3 ? 0 : count == 2 ? 3 : 5;
      var normal = planeNormals[3 - count];
      return new ContactRegistrationStage(normal, normal.dot(points[start]),
        [for (i in start...start + count) request(points[i])]);
    });
    var pending:Null<ContactProbeRequest> = sequence.start();
    for (i in 0...points.length) {
      check(pending != null && sequence.result == null, "Every contact remains pending without publishing a work frame");
      pending = sequence.observe(actual.transformPoint(points[i]));
    }
    check(pending == null && sequence.result != null, "The sixth completed contact terminates the sequence");
    var fitted:processkit.perception.ContactRegistration.ContactRegistrationResult = cast sequence.result;
    check(fitted.accepted && fitted.rank == 6 && fitted.require().transformPoint(new Vec3()).sub(actual.translation).norm() < 1e-7,
      "Only the accepted full rigid fit publishes the recovered frame");
    check(calls.length == 3 && calls[0] == 3 && calls[1] == 2 && calls[2] == 1, "Stage selection follows the measured 3-2-1 sequence");
    var rejected = false;
    try sequence.start() catch (_:Dynamic) rejected = true;
    check(rejected, "Registration cannot reuse contacts at another parking pose");
    var invalid = new ContactRegistrationSequence(new ContactPoseEnvelope(Transform3.identity(),
      new Vec3(0.02, 0.02, 0.02), new Vec3(), 0.00001), (count, _, _) ->
        new ContactRegistrationStage(new Vec3(0, 0, 1), 0, [for (_ in 0...count) request(new Vec3())]));
    invalid.start(); invalid.observe(new Vec3()); invalid.observe(new Vec3());
    rejected = false;
    try invalid.observe(new Vec3()) catch (_:Dynamic) rejected = true;
    check(rejected && invalid.result == null, "Parallel later-stage planes fail without authorizing welding");
    var deficient = new ContactRegistrationSequence(new ContactPoseEnvelope(Transform3.identity(),
      new Vec3(0.02, 0.02, 0.02), new Vec3(0.01, 0.01, 0.01), 0.00001), (count, _, _) ->
        new ContactRegistrationStage(planeNormals[3 - count], 0, [for (_ in 0...count) request(new Vec3())]));
    deficient.start();
    rejected = false;
    try for (_ in 0...6) deficient.observe(new Vec3()) catch (_:Dynamic) rejected = true;
    check(rejected && deficient.result == null, "Independent planes alone cannot authorize a rank-deficient contact pattern");
    var inconsistent = new ContactRegistrationSequence(new ContactPoseEnvelope(Transform3.identity(),
      new Vec3(0.001, 0.001, 0.001), new Vec3(), 0.00001), (count, _, _) ->
        new ContactRegistrationStage(new Vec3(0, 0, 1), 0, [for (_ in 0...count) request(new Vec3())]));
    inconsistent.start();
    rejected = false;
    try for (_ in 0...3) inconsistent.observe(new Vec3(0, 0, 0.01)) catch (_:Dynamic) rejected = true;
    check(rejected && inconsistent.result == null, "Inconsistent measured contacts fail before selecting the next stage");
    Sys.println('Contact registration sequence: $checks assertions passed');
  }
}
