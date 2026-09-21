package robotkit.protocol;

@:wire
class FrameTarget {
  @:id(1) public var robotId:haxe.Int64;
  @:id(2) public var frame:String;
  @:id(3) public var position:Array<Float>;
  @:id(4) public var rotation:Array<Float>;
  @:id(5) public var expiryNs:haxe.Int64;

  public function new(?robotId:haxe.Int64 = null, ?frame:String = "",
      ?position:Array<Float> = null, ?rotation:Array<Float> = null,
      ?expiryNs:haxe.Int64 = null) {
    this.robotId = robotId == null ? haxe.Int64.ofInt(0) : robotId;
    this.frame = frame;
    this.position = position == null ? [] : position;
    this.rotation = rotation == null ? [] : rotation;
    this.expiryNs = expiryNs == null ? haxe.Int64.ofInt(0) : expiryNs;
  }
}
