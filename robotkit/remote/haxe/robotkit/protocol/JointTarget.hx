package robotkit.protocol;

@:wire
class JointTarget {
  @:id(1) public var robotId:haxe.Int64;
  @:id(2) public var joint:Int;
  @:id(3) public var mode:Int;
  @:id(4) public var target:Float;
  @:id(5) public var sequence:haxe.Int64;
  /** Reserved: must be zero until a shared deadline clock is negotiated. */
  @:id(6) public var expiryNs:haxe.Int64;

  public function new(?robotId:haxe.Int64 = null, ?joint:Int = 0, ?mode:Int = 1,
      ?target:Float = 0.0, ?sequence:haxe.Int64 = null,
      ?expiryNs:haxe.Int64 = null) {
    this.robotId = robotId == null ? haxe.Int64.ofInt(0) : robotId;
    this.joint = joint;
    this.mode = mode;
    this.target = target;
    this.sequence = sequence == null ? haxe.Int64.ofInt(0) : sequence;
    this.expiryNs = expiryNs == null ? haxe.Int64.ofInt(0) : expiryNs;
  }
}
