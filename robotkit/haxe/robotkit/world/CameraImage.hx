package robotkit.world;

import haxe.io.Bytes;

/**
 * Immutable camera pixels carried with a sensor observation.
 *
 * The constructor copies the caller's bytes and `bytes()` returns a copy, so
 * no caller can mutate a published image; frames can therefore share one
 * instance instead of copying the pixels again.
 */
class CameraImage {
  static inline var MAX_BYTES:Int = 67108864;

  public final width:Int;
  public final height:Int;
  /** One of `rgb8`, `depth32f` (little-endian metres), or `jpeg`. */
  public final encoding:String;
  final pixelBytes:Bytes;

  public function new(width:Int, height:Int, encoding:String, bytes:Bytes) {
    if (width <= 0 || height <= 0 || encoding == null || bytes == null)
      throw "Camera image requires positive dimensions, an encoding, and pixel bytes";
    var bytesPerPixel = bytesPerPixelFor(encoding);
    if (bytes.length == 0 || bytes.length > MAX_BYTES)
      throw "Camera image byte length is outside the supported range";
    if (bytesPerPixel != 0 && (width > Std.int(MAX_BYTES / bytesPerPixel / height) ||
        bytes.length != width * height * bytesPerPixel))
      throw 'Camera image "$encoding" byte length does not match its dimensions';
    this.width = width;
    this.height = height;
    this.encoding = encoding;
    pixelBytes = copyOf(bytes);
  }

  /** Bytes per pixel for a raw encoding, or 0 for a compressed one. */
  public static function bytesPerPixelFor(encoding:String):Int
    return switch encoding {
      case "rgb8": 3;
      case "depth32f": 4;
      case "jpeg": 0;
      case _: throw 'Unsupported camera image encoding "$encoding"';
    };

  public var byteLength(get, never):Int;

  /** Returns an independent copy of the pixel bytes. */
  public function bytes():Bytes return copyOf(pixelBytes);

  /** Reads one little-endian float from a `depth32f` image, in metres. */
  public function depthAt(x:Int, y:Int):Float {
    if (encoding != "depth32f") throw "Camera image is not a depth32f image";
    if (x < 0 || y < 0 || x >= width || y >= height)
      throw "Depth pixel lies outside the image";
    return pixelBytes.getFloat((y * width + x) * 4);
  }

  function get_byteLength():Int return pixelBytes.length;

  static function copyOf(source:Bytes):Bytes {
    var result = Bytes.alloc(source.length);
    result.blit(0, source, 0, source.length);
    return result;
  }
}
