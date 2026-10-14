package processkit.perception;

import processkit.ContactProbeRunner.ContactProbeRequest;
import processkit.perception.ContactRegistration.PlaneContact;
import processkit.perception.ContactRegistration.ContactRegistrationResult;
import robotkit.spatial.Vec3;

/** Prepared probes on one nominal work-frame plane; all lengths are in metres. */
class ContactRegistrationStage {
  public final normal:Vec3;
  public final offset:Float;
  public final probes:Array<ContactProbeRequest>;
  /** Checked entry posture for this stage; null uses the saved registration start. */
  public var entryJoints(get, never):Null<Array<Float>>;
  final checkpoint:Null<Array<Float>>;
  public function new(normal:Vec3, offset:Float, probes:Array<ContactProbeRequest>, entryJoints:Null<Array<Float>> = null) {
    if (normal == null || !Math.isFinite(normal.norm()) || normal.norm() < 1e-12 || !Math.isFinite(offset) || probes == null || probes.length == 0)
      throw "Contact registration stage needs a plane and prepared probes";
    for (probe in probes) if (probe == null) throw "Contact registration stage has a missing probe";
    if (entryJoints != null) for (joint in entryJoints)
      if (!Math.isFinite(joint)) throw "Contact registration entry has a non-finite joint";
    this.normal = normal.normalized(); this.offset = offset / normal.norm(); this.probes = probes.copy();
    checkpoint = entryJoints == null ? null : entryJoints.copy();
  }
  function get_entryJoints():Null<Array<Float>> return checkpoint == null ? null : checkpoint.copy();
}

/** Measured 3-2-1 registration policy; selection receives only CAD/observed uncertainty, never physical work poses. */
class ContactRegistrationSequence {
  public final envelope:ContactPoseEnvelope;
  /** Typed source observations retained with the accepted pose for downstream uncertainty checks. */
  public var measurements(get, never):Array<ContactMeasurement>;
  public var result(default, null):Null<ContactRegistrationResult> = null;
  final select:Int->Array<Vec3>->ContactPoseEnvelope->ContactRegistrationStage;
  final contacts:Array<PlaneContact> = [];
  final observedMeasurements:Array<ContactMeasurement> = [];
  final normals:Array<Vec3> = [];
  var stage:Null<ContactRegistrationStage> = null;
  var index = 0;
  var started = false;
  public var stageTransitionPending(default, null) = false;

  /** The selected CAD plane and its checked probes, while the sequence works on that plane. */
  public function activeStage():Null<ContactRegistrationStage> return stage;
  function get_measurements():Array<ContactMeasurement> return observedMeasurements.copy();

  public function new(envelope:ContactPoseEnvelope,
      select:Int->Array<Vec3>->ContactPoseEnvelope->ContactRegistrationStage) {
    if (envelope == null || select == null) throw "Contact registration needs bounded uncertainty and a CAD probe selector";
    this.envelope = envelope; this.select = select;
  }

  public function start():ContactProbeRequest {
    if (started) throw "Contact registration sequence cannot be reused";
    started = true;
    return selectStage();
  }

  function selectStage():ContactProbeRequest {
    var count = 3 - normals.length;
    var selected = select(count, normals.copy(), envelope);
    if (selected == null || selected.probes.length != count)
      throw 'Contact registration needs exactly $count probes in this stage';
    if (normals.length == 1 && normals[0].cross(selected.normal).norm() < 1e-3 ||
        normals.length == 2 && Math.abs(normals[0].cross(normals[1]).dot(selected.normal)) < 1e-3)
      throw "Contact registration stages need independent plane normals";
    stage = selected; index = 0; stageTransitionPending = false;
    return selected.probes[0];
  }

  /** Select the next plane from completed measurements; the runner then follows its checked entry. */
  public function selectNextStage():ContactProbeRequest {
    if (!started || !stageTransitionPending || result != null)
      throw "Contact registration has no pending plane transition";
    return selectStage();
  }

  /** Production observations retain sensor identity, units, frame, acquisition time and error bound. */
  public function observeMeasurement(measurement:ContactMeasurement):Null<ContactProbeRequest> {
    if (!started || stage == null || result != null || measurement == null)
      throw "Contact registration has no pending typed observation";
    if (measurement.unit != "m" || measurement.coordinateFrame.length == 0 ||
        (measurement.sensorKind != processkit.tool.WeldSensor.KIND && measurement.sensorKind != "tool_contact") ||
        measurement.uncertaintyMeters > envelope.measurementError)
      throw 'Contact registration measurement "${measurement.sensorId}" has unsupported units, frame or uncertainty';
    if (observedMeasurements.length > 0) {
      var previous = observedMeasurements[observedMeasurements.length - 1];
      if (measurement.sensorId != previous.sensorId || measurement.sensorKind != previous.sensorKind ||
          measurement.sensorFrame != previous.sensorFrame || measurement.coordinateFrame != previous.coordinateFrame ||
          measurement.sourceClockId != previous.sourceClockId ||
          measurement.unit != previous.unit || measurement.sequence <= previous.sequence ||
          measurement.sourceTimestampNs <= previous.sourceTimestampNs || measurement.jointTimestampNs <= previous.jointTimestampNs)
        throw 'Contact registration measurement "${measurement.sensorId}" is ambiguous or inconsistent with the accepted source stream';
    }
    observedMeasurements.push(measurement);
    return observePoint(measurement.point);
  }

  function observePoint(measured:Vec3):Null<ContactProbeRequest> {
    var active:ContactRegistrationStage = cast stage;
    contacts.push(new PlaneContact(active.normal, active.offset, measured));
    index++;
    if (index < active.probes.length) return active.probes[index];
    envelope.refine(contacts);
    normals.push(active.normal);
    stage = null;
    index = 0;
    if (normals.length < 3) {
      stageTransitionPending = true;
      return null;
    }
    var rotation = envelope.rotationBound;
    var rotationLimit = Math.max(1e-12, Math.min(Math.PI,
      Math.abs(rotation.x) + Math.abs(rotation.y) + Math.abs(rotation.z)));
    // Fit translation is expressed at the work origin, while saved bounds are
    // chassis-origin motion. Include the rotation lever arm about that origin.
    var translationLimit = Math.max(1e-12, envelope.translationBound.norm() +
      2 * envelope.nominalWork.translation.norm() * Math.sin(rotationLimit * 0.5));
    var fitted = ContactRegistration.fit(contacts, envelope.nominalWork, envelope.measurementError,
      translationLimit, rotationLimit);
    fitted.require();
    var bounded = envelope.constrainWorkFrame(fitted.estimate);
    if (bounded == null) {
      throw 'Contact registration fit lies outside the retained chassis pose-error bounds '
        + '(${envelope.describeWorkFrame(fitted.estimate)}; fit residual ${fitted.maxResidual} m)';
    }
    var correction = envelope.nominalWork.inverse().compose(bounded);
    result = new ContactRegistrationResult(bounded, correction, fitted.rank,
      envelope.maximumPlaneResidual(bounded), true, null);
    return null;
  }
}
