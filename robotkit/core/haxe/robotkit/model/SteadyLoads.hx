package robotkit.model;

/**
 * The loads on a machine that do not depend on its motion, which a drive must carry on top of
 * accelerating: gravity and friction. They are assumptions, not datasheet values.
 */
class SteadyLoads {
  /**
   * Running friction of a sliding axis's rails, in N, for an axis whose joint carries no friction of
   * its own (`Joint.frictionLoss`). Assumption: about 5 N for a hobby-class axis on preloaded
   * recirculating-ball rail blocks. Rail makers quote friction coefficients of roughly 0.002 to
   * 0.005 of the preload plus seal drag; no value for any particular block is used.
   */
  public var railDrag:Float = 5.0;
  /** Acceleration of gravity along the model's -Z, m/s². */
  public var gravity:Float = DriveLoads.GRAVITY;

  public function new(?railDrag:Float, ?gravity:Float) {
    if (railDrag != null) this.railDrag = railDrag;
    if (gravity != null) this.gravity = gravity;
    if (!(this.railDrag >= 0.0) || !(this.gravity >= 0.0)) throw "Steady loads must be non-negative";
  }

  public function copy():SteadyLoads return new SteadyLoads(railDrag, gravity);

  /** The friction a sliding joint's rails add, in N: its own dry friction, or the assumed rail drag. */
  public function friction(joint:Joint):Float
    return joint.frictionLoss > 0.0 ? joint.frictionLoss : joint.type == JointType.Prismatic ? railDrag : 0.0;
}
