package robotkit.protocol;

@:wire
class RobotStateMsg {
  @:id(1) public var robotId:haxe.Int64;
  @:id(2) public var sequence:haxe.Int64;
  @:id(3) public var sourceTimestampNs:haxe.Int64;
  @:id(4) public var q:Array<Float>;
  @:id(5) public var dq:Array<Float>;
  @:id(6) public var effort:Array<Float>;
  @:id(7) public var mode:Int;
  @:id(8) public var fault:Int;
  @:id(9) public var receivedTimestampNs:haxe.Int64;
  @:id(10) public var safety:Int;

  public function new(?robotId:haxe.Int64 = null, ?sequence:haxe.Int64 = null,
      ?sourceTimestampNs:haxe.Int64 = null, ?q:Array<Float> = null,
      ?dq:Array<Float> = null, ?effort:Array<Float> = null, ?mode:Int = 0,
      ?fault:Int = 0, ?receivedTimestampNs:haxe.Int64 = null, ?safety:Int = 0) {
    this.robotId = robotId == null ? haxe.Int64.ofInt(0) : robotId;
    this.sequence = sequence == null ? haxe.Int64.ofInt(0) : sequence;
    this.sourceTimestampNs = sourceTimestampNs == null
      ? haxe.Int64.ofInt(0)
      : sourceTimestampNs;
    this.q = q == null ? [] : q;
    this.dq = dq == null ? [] : dq;
    this.effort = effort == null ? [] : effort;
    this.mode = mode;
    this.fault = fault;
    this.receivedTimestampNs = receivedTimestampNs == null
      ? this.sourceTimestampNs
      : receivedTimestampNs;
    this.safety = safety;
  }

}
