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
  public function new(normal:Vec3, offset:Float, probes:Array<ContactProbeRequest>) {
    if (normal == null || !Math.isFinite(normal.norm()) || normal.norm() < 1e-12 || !Math.isFinite(offset) || probes == null || probes.length == 0)
      throw "Contact registration stage needs a plane and prepared probes";
    for (probe in probes) if (probe == null) throw "Contact registration stage has a missing probe";
    this.normal = normal.normalized(); this.offset = offset / normal.norm(); this.probes = probes.copy();
  }
}

/** Measured 3-2-1 registration policy; selection receives only CAD/observed uncertainty, never physical work poses. */
class ContactRegistrationSequence {
  public final envelope:ContactPoseEnvelope;
  public var result(default, null):Null<ContactRegistrationResult> = null;
  final select:Int->Array<Vec3>->ContactPoseEnvelope->ContactRegistrationStage;
  final contacts:Array<PlaneContact> = [];
  final normals:Array<Vec3> = [];
  var stage:Null<ContactRegistrationStage> = null;
  var index = 0;
  var started = false;

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
    stage = selected; index = 0;
    return selected.probes[0];
  }

  /** Called only after an executed probe has completed its checked retreat. */
  public function observe(measured:Vec3):Null<ContactProbeRequest> {
    if (!started || stage == null || result != null || measured == null)
      throw "Contact registration has no pending observation";
    var active:ContactRegistrationStage = cast stage;
    contacts.push(new PlaneContact(active.normal, active.offset, measured));
    index++;
    if (index < active.probes.length) return active.probes[index];
    envelope.refine(contacts);
    normals.push(active.normal);
    if (normals.length < 3) return selectStage();
    var fitted = ContactRegistration.fit(contacts, envelope.nominalWork, envelope.measurementError);
    fitted.require();
    result = fitted;
    return null;
  }
}
