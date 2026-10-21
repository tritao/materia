package app.editor;

import cadkit.threads.ThreadTexture;
import haxe.io.Bytes;
import materia.project.ThreadSpecData;
import nativekit.scene.ImageData;

/** CadKit's ThreadTexture as a scene image with its mip levels. */
class CosmeticThreadTexture {
  public static inline var SIZE:Int = ThreadTexture.SIZE;
  public static inline var MIPS:Int = ThreadTexture.MIPS;

  public static function image(spec:ThreadSpecData, normal:Bool):ImageData
    return new ImageData(SIZE, SIZE).setMipCount(MIPS).setPixels(pixels(spec, normal));

  public static function pixels(spec:ThreadSpecData, normal:Bool):Bytes return ThreadTexture.pixels(spec, normal);
}
