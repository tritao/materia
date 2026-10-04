package robotkit.protocol;

import haxe.Int64;

/** One atomic, sequenced command containing one or more joint targets. */
@:wire
class JointTargets {
  @:id(1) public var robotId:Int64;
  @:id(2) public var targets:Array<JointTargetValue>;
  @:id(3) public var sequence:Int64;
  /** Reserved until a shared deadline clock is negotiated; must be zero. */
  @:id(4) public var expiryNs:Int64;

  public function new(?robotId:Int64 = null, ?targets:Array<JointTargetValue> = null,
      ?sequence:Int64 = null, ?expiryNs:Int64 = null) {
    this.robotId = robotId == null ? Int64.ofInt(0) : robotId;
    this.targets = targets == null ? [] : targets;
    this.sequence = sequence == null ? Int64.ofInt(0) : sequence;
    this.expiryNs = expiryNs == null ? Int64.ofInt(0) : expiryNs;
  }
}
