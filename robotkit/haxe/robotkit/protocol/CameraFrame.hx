package robotkit.protocol;

@:wire
class CameraFrame {
  @:id(1) public var sensor:haxe.Int64;
  @:id(2) public var timestampNs:haxe.Int64;
  @:id(3) public var width:Int;
  @:id(4) public var height:Int;
  @:id(5) public var format:PixelFormat;
  @:id(6) public var pixels:BufferRef;

  public function new(?sensor:haxe.Int64 = null,
      ?timestampNs:haxe.Int64 = null, ?width:Int = 0, ?height:Int = 0,
      ?format:PixelFormat = PixelFormat.RGB8, ?pixels:BufferRef = null) {
    this.sensor = sensor == null ? haxe.Int64.ofInt(0) : sensor;
    this.timestampNs = timestampNs == null ? haxe.Int64.ofInt(0) : timestampNs;
    this.width = width;
    this.height = height;
    this.format = format;
    this.pixels = pixels == null ? new BufferRef() : pixels;
  }
}
