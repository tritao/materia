package robotkit.core;



/**
  A robot joint that moves with another: joint `follower`'s position is
  `ratio * leader + offset`, by index into the robot description's joints and
  in SI units. A lead screw turning with its carriage is one.
**/
class CoupledJoint {
  public final follower:Int;
  public final leader:Int;
  public final ratio:Float;
  public final offset:Float;

  public function new(follower:Int, leader:Int, ratio:Float, offset:Float) {
    if (follower < 0 || leader < 0 || follower == leader || !Math.isFinite(ratio) ||
        ratio == 0.0 || !Math.isFinite(offset))
      throw "A coupled joint needs distinct joints, a nonzero finite ratio and a finite offset";
    this.follower = follower;
    this.leader = leader;
    this.ratio = ratio;
    this.offset = offset;
  }
}
