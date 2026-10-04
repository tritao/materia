package robotkit.protocol;

/** Typed MessagePack payload for the recording channel. */
@:wire
class RecordingProcessEventMsg {
  @:id(1) public var planId:haxe.Int64;
  @:id(2) public var channel:String;
  @:id(3) public var value:RecordingProcessValueMsg;
  @:id(4) public var scheduledTimeNs:haxe.Int64;
  @:id(5) public var appliedOwnerTimeNs:haxe.Int64;
  @:id(6) public var cause:Int;
  @:id(7) public var robotId:String;

  public function new() {}
}
