package robotkit.protocol;

@:wire
class CameraFrame {
  @:id(1) public var robotId:haxe.Int64;
  @:id(2) public var sensorId:String;
  @:id(3) public var kind:String;
  @:id(4) public var frameId:String;
  @:id(5) public var sequence:haxe.Int64;
  @:id(6) public var sourceTimestampNs:haxe.Int64;
  @:id(7) public var receivedTimestampNs:haxe.Int64;
  @:id(8) public var width:Int;
  @:id(9) public var height:Int;
  @:id(10) public var format:PixelFormat;
  @:id(11) public var pixels:BufferRef;
  @:id(12) public var linkId:String;
  @:id(13) public var mountPosition:Array<Float>;
  @:id(14) public var mountRotation:Array<Float>;
  @:id(15) public var sourceClockId:String;
  @:id(16) public var receivedClockId:String;

  public function new(?robotId:haxe.Int64 = null, ?sensorId:String = "",
      ?kind:String = "camera", ?frameId:String = "", ?sequence:haxe.Int64 = null,
      ?sourceTimestampNs:haxe.Int64 = null, ?receivedTimestampNs:haxe.Int64 = null,
      ?width:Int = 0, ?height:Int = 0, ?format:PixelFormat = PixelFormat.RGB8,
      ?pixels:BufferRef = null, ?linkId:String = "",
      ?mountPosition:Array<Float>, ?mountRotation:Array<Float>,
      ?sourceClockId:String = "unspecified",
      ?receivedClockId:String = "robotkit.monotonic") {
    this.robotId = robotId == null ? haxe.Int64.ofInt(0) : robotId;
    this.sensorId = sensorId;
    this.kind = kind;
    this.frameId = frameId;
    this.sequence = sequence == null ? haxe.Int64.ofInt(0) : sequence;
    this.sourceTimestampNs = sourceTimestampNs == null
      ? haxe.Int64.ofInt(0) : sourceTimestampNs;
    this.receivedTimestampNs = receivedTimestampNs == null
      ? haxe.Int64.ofInt(0) : receivedTimestampNs;
    this.width = width;
    this.height = height;
    this.format = format;
    this.pixels = pixels == null ? new BufferRef() : pixels;
    this.linkId = linkId;
    this.mountPosition = mountPosition == null ? [0.0, 0.0, 0.0] : mountPosition.copy();
    this.mountRotation = mountRotation == null ? [0.0, 0.0, 0.0, 1.0] : mountRotation.copy();
    this.sourceClockId = sourceClockId;
    this.receivedClockId = receivedClockId;
  }
}
