package robotkit.model;

/** Follower joint coordinate = leader * ratio + offset, in SI units. */
class JointCoupling {
  public final id:String;
  public final leader:JointId;
  public final follower:JointId;
  public final ratio:Float;
  public final offset:Float;

  public function new(id:String, leader:JointId, follower:JointId,
      ratio:Float, offset:Float) {
    if (id == null || StringTools.trim(id).length == 0 ||
        leader == null || StringTools.trim(leader).length == 0 ||
        follower == null || StringTools.trim(follower).length == 0 ||
        leader == follower || !Math.isFinite(ratio) || ratio == 0.0 ||
        !Math.isFinite(offset))
      throw "Joint coupling needs distinct joints, nonzero finite ratio and finite offset";
    this.id = id;
    this.leader = leader;
    this.follower = follower;
    this.ratio = ratio;
    this.offset = offset;
  }
}
