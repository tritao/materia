package robotkit.manipulation;

import robotkit.model.Joint;
import robotkit.model.LinkId;
import robotkit.spatial.Vec3;

/**
 * Where a redundant arm's swivel (elbow) angle is measured: a shoulder, an
 * elbow and a wrist point, each fixed on a link. The swivel is how far the
 * elbow is turned about the shoulder-wrist line, from the plane holding that
 * line and `reference` (a direction in the arm's base frame, base Z by
 * default), positive by the right-hand rule about shoulder -> wrist. On a
 * 7-axis arm it is the one motion left once the tool pose is fixed, so it
 * names the arm's configuration (see kinematicskit `SwivelTask`).
 */
class ArmSwivel {
  public final shoulderLink:LinkId;
  public final shoulder:Vec3;
  public final elbowLink:LinkId;
  public final elbow:Vec3;
  public final wristLink:LinkId;
  public final wrist:Vec3;
  public final reference:Vec3;

  public function new(shoulderLink:LinkId, shoulder:Vec3, elbowLink:LinkId, elbow:Vec3, wristLink:LinkId,
      wrist:Vec3, ?reference:Vec3) {
    if (shoulderLink == null || elbowLink == null || wristLink == null || shoulder == null || elbow == null ||
        wrist == null)
      throw "An arm swivel needs a shoulder, an elbow and a wrist point";
    this.shoulderLink = shoulderLink;
    this.shoulder = shoulder;
    this.elbowLink = elbowLink;
    this.elbow = elbow;
    this.wristLink = wristLink;
    this.wrist = wrist;
    this.reference = reference == null ? new Vec3(0.0, 0.0, 1.0) : reference;
  }

  /**
   * The swivel through three joints' pivots (where each joint sits on its
   * child link): for a 7-axis arm, its 2nd, 4th and 6th joints.
   */
  public static function throughJoints(shoulder:Joint, elbow:Joint, wrist:Joint, ?reference:Vec3):ArmSwivel {
    function pivot(joint:Joint):Vec3
      return new Vec3(joint.childFramePosition[0], joint.childFramePosition[1], joint.childFramePosition[2]);
    return new ArmSwivel(shoulder.child.id, pivot(shoulder), elbow.child.id, pivot(elbow), wrist.child.id,
      pivot(wrist), reference);
  }
}
