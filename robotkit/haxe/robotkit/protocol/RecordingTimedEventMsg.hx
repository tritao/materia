package robotkit.protocol;

/** Typed MessagePack payload for the recording channel. */
@:wire
class RecordingTimedEventMsg {
  @:id(1) public var timeNs:haxe.Int64;
  @:id(2) public var channel:String;
  @:id(3) public var value:RecordingProcessValueMsg;
  @:id(4) public var holdPolicy:Int;

  public function new() {}
}
