package robotkit.protocol;

/** Typed MessagePack payload for the recording channel. */
@:wire
class RecordingSensorMsg {
  @:id(1) public var sensorId:String;
  @:id(2) public var kind:String;
  @:id(3) public var frameId:String;
  @:id(4) public var sequence:haxe.Int64;
  @:id(5) public var sourceTimestampNs:haxe.Int64;
  @:id(6) public var receivedTimestampNs:haxe.Int64;
  @:id(7) public var sourceClockId:String;
  @:id(8) public var receivedClockId:String;
  @:id(9) public var values:Array<Float>;
  @:id(10) public var linkId:String;
  @:id(11) public var mountPosition:Array<Float>;
  @:id(12) public var mountRotation:Array<Float>;
  @:id(13) public var image:Null<RecordingImageMsg>;
  @:id(14) public var robotId:String;

  public function new() {}
}
