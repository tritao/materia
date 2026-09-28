package motionkit.robot;

import motionkit.trajectory.ValidationGuarantees;

/** Position in the authored program and current path. */
class ManipulatorProgress {
  public final block:Int;
  public final op:Int;
  public final pathDistance:Float;
  public final guarantees:Null<ValidationGuarantees>;
  public function new(block:Int, op:Int, pathDistance:Float,
      ?guarantees:ValidationGuarantees) {
    this.block = block; this.op = op; this.pathDistance = pathDistance;
    this.guarantees = guarantees;
  }
}
