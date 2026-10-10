package collisionkit;

/** A body of a `CollisionDescription`: its name, pair-class group (CL-D4), whether it is fixed in the cell, and its pose. */
class DescribedBody {
  public final name:String;
  public final group:Int;
  public final fixed:Bool;
  public var pose:CollisionPose;

  public function new(name:String, group:Int, fixed:Bool, pose:CollisionPose) {
    this.name = name;
    this.group = group;
    this.fixed = fixed;
    this.pose = pose;
  }
}
