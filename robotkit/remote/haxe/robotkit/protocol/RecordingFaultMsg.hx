package robotkit.protocol;

/** Typed MessagePack payload for the recording channel. */
@:wire
class RecordingFaultMsg {
  @:id(1) public var id:String;
  @:id(2) public var code:Int;
  @:id(3) public var message:String;
  @:id(4) public var fatal:Bool;

  public function new() {}
}
