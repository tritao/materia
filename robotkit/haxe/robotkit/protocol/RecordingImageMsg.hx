package robotkit.protocol;

/** Typed MessagePack payload for the recording channel. */
@:wire
class RecordingImageMsg {
  @:id(1) public var width:Int;
  @:id(2) public var height:Int;
  @:id(3) public var encoding:String;
  @:id(4) public var pixels:haxe.io.Bytes;

  public function new() {}
}
