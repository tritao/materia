package robotkit.model;

/** Follower joint coordinate = leader * ratio + offset, in SI units. */
class JointCoupling {
  public final id:String;
  public final leader:JointId;
  public final follower:JointId;
  public final ratio:Float;
  public final offset:Float;
  /** Share of power passed from leader to follower or back, such as a lead screw's 0.4; 1 is lossless. */
  public var efficiency:Float = 1.0;
  /**
   * How stiff the coupling is, as force at the leader per unit of the leader's travel (N/m for a
   * sliding leader, N m/rad for a turning one), such as a timing belt's stretch. Zero means rigid.
   */
  public var stiffness:Float = 0.0;
  /** Lost motion when the coupling reverses, in the leader's units, such as a lead screw nut's backlash. */
  public var backlash:Float = 0.0;
  /**
   * Constant resisting effort the coupling adds at the follower while it moves, in the follower's
   * units (N m for a turning follower), such as a screw nut's drag.
   */
  public var drag:Float = 0.0;

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
