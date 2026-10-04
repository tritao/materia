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
  @:id(11) public var trajectoryQueueDepth:Int;
  @:id(12) public var trajectoryActive:Bool;
  @:id(13) public var trajectoryTimeNs:haxe.Int64;
  @:id(14) public var trajectoryDurationNs:haxe.Int64;
  @:id(15) public var trajectoryTag:haxe.Int64;
  @:id(16) public var trajectoryTagTimeNs:haxe.Int64;
  @:id(17) public var sessionState:Int;
  @:id(18) public var activePlanId:haxe.Int64;
  @:id(19) public var committedUntilNs:haxe.Int64;
  @:id(20) public var queueEndTimeNs:haxe.Int64;

  public function new(?robotId:haxe.Int64 = null, ?sequence:haxe.Int64 = null,
      ?sourceTimestampNs:haxe.Int64 = null, ?q:Array<Float> = null,
      ?dq:Array<Float> = null, ?effort:Array<Float> = null, ?mode:Int = 0,
      ?fault:Int = 0, ?receivedTimestampNs:haxe.Int64 = null, ?safety:Int = 0,
      ?trajectoryQueueDepth:Int = 0, ?trajectoryActive:Bool = false,
      ?trajectoryTimeNs:haxe.Int64, ?trajectoryDurationNs:haxe.Int64,
      ?trajectoryTag:haxe.Int64, ?trajectoryTagTimeNs:haxe.Int64,
      ?sessionState:Int = 0, ?activePlanId:haxe.Int64,
      ?committedUntilNs:haxe.Int64, ?queueEndTimeNs:haxe.Int64) {
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
    this.trajectoryQueueDepth = trajectoryQueueDepth;
    this.trajectoryActive = trajectoryActive;
    this.trajectoryTimeNs = trajectoryTimeNs == null ? haxe.Int64.ofInt(0) : trajectoryTimeNs;
    this.trajectoryDurationNs = trajectoryDurationNs == null ? haxe.Int64.ofInt(0) : trajectoryDurationNs;
    this.trajectoryTag = trajectoryTag == null ? haxe.Int64.ofInt(0) : trajectoryTag;
    this.trajectoryTagTimeNs = trajectoryTagTimeNs == null ? haxe.Int64.ofInt(0) : trajectoryTagTimeNs;
    this.sessionState = sessionState;
    this.activePlanId = activePlanId == null ? haxe.Int64.ofInt(0) : activePlanId;
    this.committedUntilNs = committedUntilNs == null ? haxe.Int64.ofInt(0) : committedUntilNs;
    this.queueEndTimeNs = queueEndTimeNs == null ? haxe.Int64.ofInt(0) : queueEndTimeNs;
  }

}
