package robotkit.protocol;

import haxe.io.Bytes;
import robotkit.world.CameraImage;

/** Decoded CameraFrame metadata paired with owned bytes resolved from its attachment. */
class CameraFrameData {
  public final frame:CameraFrame;
  final pixelBytes:Bytes;

  public function new(frame:CameraFrame, pixels:Bytes, ?offset:Int = 0,
      ?length:Int = -1) {
    if (frame == null || pixels == null)
      throw "Camera frame data requires metadata and pixel bytes";
    var resolvedLength = length < 0 ? pixels.length - offset : length;
    if (offset < 0 || offset > pixels.length || resolvedLength <= 0 ||
        resolvedLength > pixels.length - offset)
      throw "Camera frame data references bytes outside the pixel buffer";
    this.frame = new CameraFrame(frame.robotId, frame.sensorId, frame.kind,
      frame.frameId, frame.sequence, frame.sourceTimestampNs,
      frame.receivedTimestampNs, frame.width, frame.height, frame.format,
      frame.pixels, frame.linkId, frame.mountPosition, frame.mountRotation,
      frame.sourceClockId, frame.receivedClockId);
    pixelBytes = copySlice(pixels, offset, resolvedLength);
  }

  /** Returns a separate copy of the resolved attachment. */
  public function pixels():Bytes return copyBytes(pixelBytes);

  /** Returns an owned world image while copying directly from retained bytes. */
  public function image():CameraImage {
    var encoding = switch frame.format {
      case PixelFormat.RGB8: "rgb8";
      case PixelFormat.Depth32F: "depth32f";
      case PixelFormat.JPEG: "jpeg";
      case _: throw "Camera frame has an unsupported pixel format";
    };
    return new CameraImage(frame.width, frame.height, encoding, pixelBytes);
  }

  static function copySlice(source:Bytes, offset:Int, length:Int):Bytes {
    var result = Bytes.alloc(length);
    result.blit(0, source, offset, length);
    return result;
  }

  static function copyBytes(source:Bytes):Bytes {
    var result = Bytes.alloc(source.length);
    result.blit(0, source, 0, source.length);
    return result;
  }
}
