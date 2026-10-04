package robotkit.protocol;

/** Typed MessagePack payload for the recording channel. */
@:wire
class RecordingWorldEventMsg {
  @:id(1) public var kind:Int;
  @:id(2) public var robotId:String;

  public function new() {}
}
