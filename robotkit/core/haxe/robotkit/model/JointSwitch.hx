package robotkit.model;

/** A physical home or limit switch, expressed in the monitored joint's SI coordinate. */
class JointSwitch {
  public final id:String;
  public final joint:JointId;
  public final frameId:FrameId;
  public final role:String;
  /** -1 trips toward decreasing coordinates, +1 toward increasing coordinates. */
  public final side:Int;
  public final trip:Float;
  public final hysteresis:Float;
  /** Bounded closing-position variation, rather than noise on every tick. */
  public final repeatability:Float;
  public final seed:Int;
  /** Optional motor follower sensed by this physical side, rather than the shared leader. */
  public final driveJoint:Null<JointId>;

  public function new(id:String, joint:JointId, frameId:FrameId, role:String, side:Int,
      trip:Float, hysteresis:Float, repeatability:Float, seed:Int = 1, ?driveJoint:JointId) {
    if (id == null || StringTools.trim(id).length == 0 ||
        joint == null || StringTools.trim(joint).length == 0 ||
        frameId == null || StringTools.trim(frameId).length == 0)
      throw "A joint switch needs an ID, monitored joint and physical frame";
    if (role != "home" && role != "limit") throw "A joint switch is home or limit";
    if (side != -1 && side != 1) throw "A joint switch side must be -1 or 1";
    if (!Math.isFinite(trip) || !Math.isFinite(hysteresis) || hysteresis < 0 ||
        !Math.isFinite(repeatability) || repeatability < 0)
      throw "Switch trip and non-negative hysteresis/repeatability must be finite";
    if (driveJoint != null && StringTools.trim(driveJoint).length == 0) throw "Switch drive joint is empty";
    this.driveJoint = driveJoint;
    this.id = id; this.joint = joint; this.frameId = frameId; this.role = role;
    this.side = side; this.trip = trip; this.hysteresis = hysteresis;
    this.repeatability = repeatability; this.seed = seed;
  }
}
