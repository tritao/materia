package visionkit;

import VisionKitNative;
import haxe.io.Bytes;

/** Caller-owned image bytes with explicit row stride. */
class ImageView {
  public final width:Int;
  public final height:Int;
  public final strideBytes:Int;
  public final pixelFormat:Int;
  public final bytes:Bytes;

  public function new(width:Int, height:Int, strideBytes:Int, pixelFormat:Int, bytes:Bytes) {
    var pixelBytes = switch pixelFormat { case 1: 3; case 2: 1; case 3: 4; case _: 0; };
    if (width <= 0 || height <= 0 || pixelBytes == 0 || bytes == null ||
        strideBytes < width * pixelBytes ||
        bytes.length < (height - 1) * strideBytes + width * pixelBytes)
      throw "Invalid image view";
    this.width = width; this.height = height; this.strideBytes = strideBytes;
    this.pixelFormat = pixelFormat; this.bytes = bytes;
  }

  public function native():vk_image_view {
    var n = new vk_image_view();
    n.set_struct_size(vk_image_view.size());
    n.set_width(width); n.set_height(height);
    n.set_stride_bytes(strideBytes); n.set_pixel_format(pixelFormat);
    n.set_data_bytes(bytes);
    return n;
  }
}
