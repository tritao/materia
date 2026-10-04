package robotkit.protocol;

/** Typed MessagePack payload for the recording channel. */
@:wire
class RecordingJointTargetMsg {
  @:id(1) public var joint:Int;
  @:id(2) public var mode:Int;
  @:id(3) public var target:Float;
  @:id(4) public var velocity:Float;
  @:id(5) public var stiffness:Float;
  @:id(6) public var damping:Float;
  @:id(7) public var feedforward:Float;

  public function new() {}
}
