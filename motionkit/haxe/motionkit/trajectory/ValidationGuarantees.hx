package motionkit.trajectory;

import trajectorykit.validation.ValidationGuarantee;

/** Per-check validation claims; sampled checks are not continuous proofs. */
class ValidationGuarantees {
  public final jointPosition:ValidationGuarantee;
  public final jointVelocity:ValidationGuarantee;
  public final jointAcceleration:ValidationGuarantee;
  public final jerk:ValidationGuarantee;
  public final continuity:ValidationGuarantee;
  public final taskSpace:ValidationGuarantee;
  /** Clearance against the cell (COLLISION.md CL4b); unchecked without a clearance world. */
  public final collision:ValidationGuarantee;

  public function new(jointPosition:ValidationGuarantee,
      jointVelocity:ValidationGuarantee, jointAcceleration:ValidationGuarantee,
      jerk:ValidationGuarantee, continuity:ValidationGuarantee,
      taskSpace:ValidationGuarantee, collision:ValidationGuarantee) {
    this.jointPosition = jointPosition;
    this.jointVelocity = jointVelocity;
    this.jointAcceleration = jointAcceleration;
    this.jerk = jerk;
    this.continuity = continuity;
    this.taskSpace = taskSpace;
    this.collision = collision;
  }
}
