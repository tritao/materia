package robotkit.protocol;

/** Typed MessagePack payload for the recording channel. */
@:wire
class RecordingProcessValueMsg {
  @:id(1) public var kind:Int;
  @:id(2) public var digital:Bool;
  @:id(3) public var analog:Float;
  @:id(4) public var command:String;
  @:id(5) public var argument:Float;

  public function new() {}
}
