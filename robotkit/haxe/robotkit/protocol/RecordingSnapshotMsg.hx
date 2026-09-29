package robotkit.protocol;

/** Typed MessagePack payload for the recording channel. */
@:wire
class RecordingSnapshotMsg {
  @:id(1) public var id:String;
  @:id(2) public var sourceSequence:haxe.Int64;
  @:id(3) public var sourceTimestampNs:haxe.Int64;
  @:id(4) public var receivedTimestampNs:haxe.Int64;
  @:id(5) public var sourceClockId:String;
  @:id(6) public var receivedClockId:String;
  @:id(7) public var positions:Array<Float>;
  @:id(8) public var velocities:Array<Float>;
  @:id(9) public var efforts:Array<Float>;
  @:id(10) public var sensors:Array<RecordingSensorMsg>;
  @:id(11) public var mode:Int;
  @:id(12) public var faultCode:Int;
  @:id(13) public var safety:Int;
  @:id(14) public var trajectoryQueueDepth:Int;
  @:id(15) public var trajectoryActive:Bool;
  @:id(16) public var trajectoryTimeNs:haxe.Int64;
  @:id(17) public var trajectoryDurationNs:haxe.Int64;
  @:id(18) public var trajectoryTag:haxe.Int64;
  @:id(19) public var trajectoryTagTimeNs:haxe.Int64;
  @:id(20) public var sessionState:Int;
  @:id(21) public var activePlanId:haxe.Int64;
  @:id(22) public var committedUntilNs:haxe.Int64;
  @:id(23) public var queueEndTimeNs:haxe.Int64;

  public function new() {}
}
