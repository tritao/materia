import haxe.Int64;
import processkit.perception.ContactRegistrationSequence;
import processkit.perception.ContactRegistrationSequence.ContactRegistrationStage;
import processkit.perception.ContactRegistration;
import processkit.perception.ContactRegistration.PlaneContact;
import processkit.perception.ContactPoseEnvelope;
import processkit.perception.ContactMeasurement;
import processkit.ContactProbeRunner.ContactProbeRequest;
import robotkit.spatial.Vec3;
import robotkit.spatial.Transform3;
import robotkit.spatial.Quat;

class ContactRegistrationSequenceTests {
  static var checks = 0;
  static function check(ok:Bool, message:String):Void { checks++; if (!ok) throw message; }
  static function request(point:Vec3):ContactProbeRequest
    return new ContactProbeRequest(new Transform3(point, Quat.identity()), new Vec3(0, 0, -1), 0.1);
  static function observation(point:Vec3, sequence:Int, sensor:String = "probe/touch", frame:String = "tool/tcp",
      clock:String = "robot-clock", uncertainty:Float = 0.000001):ContactMeasurement {
    var timestamp = Int64.ofInt(sequence);
    return new ContactMeasurement(sensor, frame, "tool_contact", "arm/base", timestamp,
      timestamp, clock, timestamp, Int64.ofInt(0), point, uncertainty);
  }
  public static function run():Void {
    var points = [new Vec3(-0.1, -0.1, 0), new Vec3(0.1, -0.1, 0), new Vec3(0.1, 0.1, 0),
      new Vec3(-0.1, 0.02, 0.05), new Vec3(0.1, 0.02, 0.05), new Vec3(0.04, -0.04, 0.05)];
    var planeNormals = [new Vec3(0, 0, 1), new Vec3(0, 1, 0), new Vec3(1, 0, 0)];
    var checkpoint = [0.03, -0.2];
    var checkpointStage = new ContactRegistrationStage(planeNormals[0], 0, [request(new Vec3())], checkpoint);
    checkpoint[0] = 0.04;
    var exposedCheckpoint:Array<Float> = cast checkpointStage.entryJoints;
    exposedCheckpoint[0] = 0.05;
    check(checkpointStage.entryJoints != null && checkpointStage.entryJoints[0] == 0.03,
      "A selected stage protects its checked transition target from caller mutation");
    var nominal = new Transform3(new Vec3(1.7, -0.2, 0.6), Quat.fromRollPitchYaw(0, 0, 0.7));
    var error = new Transform3(new Vec3(0.012, -0.009, 0), Quat.fromRollPitchYaw(0, 0, 0.025));
    var actual = error.inverse().compose(nominal);
    var envelope = new ContactPoseEnvelope(nominal, new Vec3(0.02, 0.02, 0), new Vec3(0, 0, 0.04), 0.00001);
    check(envelope.containsWorkFrame(actual) && !envelope.containsWorkFrame(
      new Transform3(new Vec3(0.03, 0, 0), Quat.identity()).inverse().compose(nominal)),
      "The retained pose envelope accepts an in-bound work placement and rejects a fitted pose outside it");
    var smallTilt = new Transform3(new Vec3(), Quat.fromRollPitchYaw(0.000005, 0, 0));
    var noisyContacts:Array<PlaneContact> = [];
    var contactNormals = [planeNormals[0], planeNormals[0], planeNormals[0],
      planeNormals[1], planeNormals[1], planeNormals[2]];
    for (i in 0...points.length) noisyContacts.push(new PlaneContact(contactNormals[i],
      contactNormals[i].dot(points[i]), smallTilt.transformPoint(points[i])));
    var boundedEnvelope = new ContactPoseEnvelope(Transform3.identity(), new Vec3(0.01, 0.01, 0.01), new Vec3(), 0.00001);
    boundedEnvelope.refine(noisyContacts);
    var noisyFit = ContactRegistration.fit(noisyContacts, Transform3.identity(), 0.00001, 0.02, 0.01);
    var boundedFit = boundedEnvelope.constrainWorkFrame(noisyFit.require());
    check(!boundedEnvelope.containsWorkFrame(noisyFit.estimate) && boundedFit != null &&
      boundedEnvelope.containsWorkFrame(cast boundedFit) && boundedEnvelope.maximumPlaneResidual(cast boundedFit) <= 0.00001,
      "A fit just outside a zero-width authored tilt bound is clamped only when every noisy plane observation remains satisfied");
    var calls:Array<Int> = [];
    var sequence = new ContactRegistrationSequence(envelope, (count, normals, bounds) -> {
      calls.push(count);
      check(normals.length == 3 - count && bounds == envelope, "Selection receives prior independent normals and measured bounds");
      if (count < 3) {
        var unresolved = bounds.region(points[5], new Vec3(1, 0, 0), new Vec3(0, 1, 0), new Vec3(0, 0, 1), bounds.estimate());
        check(unresolved.normalTravel > 0.005, "Later stages retain unobserved displacement rather than a replaced zero prior");
      }
      var start = count == 3 ? 0 : count == 2 ? 3 : 5;
      var normal = planeNormals[3 - count];
      return new ContactRegistrationStage(normal, normal.dot(points[start]),
        [for (i in start...start + count) request(points[i])]);
    });
    var pending:Null<ContactProbeRequest> = sequence.start();
    var active = sequence.activeStage();
    check(active != null && active.probes.length == 3,
      "The sequence exposes the selected checked stage for CAD preflight and execution reporting");
    for (i in 0...points.length) {
      check(pending != null && sequence.result == null, "Every contact remains pending without publishing a work frame");
      pending = sequence.observeMeasurement(observation(actual.transformPoint(points[i]), i + 1));
      if (i == 2) {
        check(pending == null && sequence.stageTransitionPending && sequence.activeStage() == null,
          "A completed plane pauses for its checked return before selecting the next plane");
        pending = sequence.selectNextStage();
        active = sequence.activeStage();
        check(active != null && active.probes.length == 2,
          "The next stage is selected after the first-plane transition");
      }
      if (i == 4) {
        check(pending == null && sequence.stageTransitionPending && sequence.activeStage() == null,
          "The second plane also pauses for its checked return");
        pending = sequence.selectNextStage();
        active = sequence.activeStage();
        check(active != null && active.probes.length == 1,
          "The final stage is selected after the second-plane transition");
      }
    }
    check(pending == null && sequence.result != null, "The sixth completed contact terminates the sequence");
    var measurementCopy = sequence.measurements;
    measurementCopy.pop();
    check(sequence.measurements.length == 6 && sequence.measurements[0].unit == "m" &&
      sequence.measurements[5].sequence == Int64.ofInt(6), "Accepted registration retains all typed observations");
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
    invalid.start();
    for (i in 1...4) invalid.observeMeasurement(observation(new Vec3(), i));
    rejected = false;
    try invalid.selectNextStage() catch (_:Dynamic) rejected = true;
    check(rejected && invalid.result == null, "Parallel later-stage planes fail without authorizing welding");
    var deficient = new ContactRegistrationSequence(new ContactPoseEnvelope(Transform3.identity(),
      new Vec3(0.02, 0.02, 0.02), new Vec3(0.01, 0.01, 0.01), 0.00001), (count, _, _) ->
        new ContactRegistrationStage(planeNormals[3 - count], 0, [for (_ in 0...count) request(new Vec3())]));
    deficient.start();
    rejected = false;
    try {
      for (i in 1...4) deficient.observeMeasurement(observation(new Vec3(), i));
      deficient.selectNextStage();
      for (i in 4...6) deficient.observeMeasurement(observation(new Vec3(), i));
      deficient.selectNextStage();
      deficient.observeMeasurement(observation(new Vec3(), 6));
    } catch (_:Dynamic) rejected = true;
    check(rejected && deficient.result == null, "Independent planes alone cannot authorize a rank-deficient contact pattern");
    var inconsistent = new ContactRegistrationSequence(new ContactPoseEnvelope(Transform3.identity(),
      new Vec3(0.001, 0.001, 0.001), new Vec3(), 0.00001), (count, _, _) ->
        new ContactRegistrationStage(new Vec3(0, 0, 1), 0, [for (_ in 0...count) request(new Vec3())]));
    inconsistent.start();
    rejected = false;
    try for (i in 1...4) inconsistent.observeMeasurement(observation(new Vec3(0, 0, 0.01), i)) catch (_:Dynamic) rejected = true;
    check(rejected && inconsistent.result == null, "Inconsistent measured contacts fail before selecting the next stage");
    var mixedSource = new ContactRegistrationSequence(new ContactPoseEnvelope(Transform3.identity(),
      new Vec3(0.02, 0.02, 0.02), new Vec3(), 0.00001), (count, _, _) ->
        new ContactRegistrationStage(new Vec3(0, 0, 1), 0, [for (_ in 0...count) request(new Vec3())]));
    mixedSource.start();
    mixedSource.observeMeasurement(observation(new Vec3(), 1));
    rejected = false;
    try mixedSource.observeMeasurement(observation(new Vec3(), 2, "another-probe")) catch (_:Dynamic) rejected = true;
    check(rejected && mixedSource.measurements.length == 1 && mixedSource.result == null,
      "A second sensor cannot be mixed into one accepted registration");
    var overBudget = new ContactRegistrationSequence(new ContactPoseEnvelope(Transform3.identity(),
      new Vec3(0.02, 0.02, 0.02), new Vec3(), 0.00001), (count, _, _) ->
        new ContactRegistrationStage(new Vec3(0, 0, 1), 0, [for (_ in 0...count) request(new Vec3())]));
    overBudget.start();
    rejected = false;
    try overBudget.observeMeasurement(observation(new Vec3(), 1, "probe/touch", "tool/tcp", "robot-clock", 0.00002))
      catch (_:Dynamic) rejected = true;
    check(rejected && overBudget.measurements.length == 0 && overBudget.result == null,
      "An observation outside the saved measurement-error budget cannot enter registration");
    Sys.println('Contact registration sequence: $checks assertions passed');
  }
}
