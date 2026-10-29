package cadkit.threads;

import cadkit.modeling.StrokeText;
import haxe.io.Bytes;

/** A generated image with every mip level, largest first, RGBA8. */
class DetailImage {
  public final width:Int;
  public final height:Int;
  public final mips:Int;
  public final pixels:Bytes;

  public function new(width:Int, height:Int, mips:Int, pixels:Bytes) {
    this.width = width;
    this.height = height;
    this.mips = mips;
    this.pixels = pixels;
  }
}

/**
 * Cosmetic textures for surface details on a face of revolution (see `SurfaceDetail`): a tangent-space normal map or a colour
 * map, with explicit mip levels whose normals keep their mean length for specular antialiasing, like `ThreadTexture`.
 * u runs round the face from its reference direction (counter-clockwise about the axis), v along it from its start.
 */
class DetailTexture {
  public static inline var KNURL_SIZE:Int = 64;
  public static inline var SCALE_WIDTH:Int = 2048;
  public static inline var SCALE_HEIGHT:Int = 128;
  /** Fractions of the surface line, from its start: where the ticks begin, how far each length reaches, where numbers sit. */
  public static inline var TICK_START:Float = 0.06;
  public static inline var TICK_LONG:Float = 0.36;
  public static inline var TICK_MEDIUM:Float = 0.28;
  public static inline var TICK_SHORT:Float = 0.2;
  public static inline var NUMBER_BASELINE:Float = 0.46;
  public static inline var NUMBER_HEIGHT:Float = 2.4;
  /** Half the width of a tick and of a numeral stroke, millimetres. */
  public static inline var TICK_HALF_WIDTH:Float = 0.11;
  public static inline var STROKE_HALF_WIDTH:Float = 0.12;

  /**
   * Straight knurling: one tooth across the image, repeated `teeth` times round the face by the mapping. `radius` and `depth`
   * are millimetres and set how steep the flanks are (a triangular tooth).
   */
  public static function knurl(teeth:Int, depth:Float, radius:Float, normal:Bool):DetailImage {
    var size = KNURL_SIZE;
    var pitch = 2 * Math.PI * radius / teeth;
    var slope = pitch > 0 ? 2 * depth / pitch : 0.0;
    var base = Bytes.alloc(size * size * 4);
    for (y in 0...size) for (x in 0...size) {
      var t = (x + 0.5) / size;
      var height = Math.abs(2 * t - 1);
      var sign = t < 0.5 ? -1.0 : 1.0;
      var offset = (y * size + x) * 4;
      if (normal) {
        var nx = -sign * slope, nz = 1.0, length = Math.sqrt(nx * nx + nz * nz);
        base.set(offset, channel(nx / length));
        base.set(offset + 1, channel(0.0));
        base.set(offset + 2, channel(nz / length));
      } else {
        var color = Std.int(Math.round(255 * (0.88 + 0.12 * height)));
        base.set(offset, color);
        base.set(offset + 1, color);
        base.set(offset + 2, color);
      }
      base.set(offset + 3, 255);
    }
    return mipChain(base, size, size, normal);
  }

  /**
   * A graduated scale: `divisions` marks round the face, long every tenth, medium every fifth and short between, in the
   * outer part of the surface line, with every `every`-th mark numbered above them. `radius` and `radiusEnd` are the face's
   * radius at its start and far end and `slant` its length along the surface, all millimetres, so marks keep their size.
   */
  public static function graduation(divisions:Int,
    every:Int, radius:Float, radiusEnd:Float, slant:Float, normal:Bool):DetailImage {
    var width = SCALE_WIDTH, height = SCALE_HEIGHT;
    var ink = [for (_ in 0...width * height) 0.0];
    var perMmV = height / slant;
    function perMmU(v:Float):Float return width / (2 * Math.PI * (radius + (radiusEnd - radius) * v));
    for (k in 0...divisions) {
      var reach = k % 10 == 0 ? TICK_LONG : k % 5 == 0 ? TICK_MEDIUM : TICK_SHORT;
      var x = k / divisions * width;
      var halfWidth = Math.max(0.7, TICK_HALF_WIDTH * perMmU(0.2));
      stroke(ink, width, height, x, TICK_START * height, x, reach * height, halfWidth);
    }
    if (every > 0) {
      var k = 0;
      while (k < divisions) {
        var text = Std.string(k);
        var scale = NUMBER_HEIGHT / StrokeText.CELL_HEIGHT;
        var total = StrokeText.width(text, NUMBER_HEIGHT);
        var u = perMmU(NUMBER_BASELINE);
        var centre = k / divisions * width, baseV = NUMBER_BASELINE * height;
        var halfWidth = Math.max(0.7, STROKE_HALF_WIDTH * u);
        for (index in 0...text.length) {
          var strokes = StrokeText.strokes(text.charAt(index));
          if (strokes == null) continue;
          var left = index * (StrokeText.CELL_WIDTH + StrokeText.GAP) * scale - total / 2;
          for (points in strokes) {
            for (i in 0...Std.int(points.length / 2) - 1) stroke(
              ink,
              width,
              height,
              centre + (left + points[2 * i] * scale) * u,
              baseV + points[2 * i + 1] * scale * perMmV,
              centre + (left + points[2 * i + 2] * scale) * u,
              baseV + points[2 * i + 3] * scale * perMmV,
              halfWidth
            );
          }
        }
        k += every;
      }
    }
    var base = Bytes.alloc(width * height * 4);
    for (y in 0...height) for (x in 0...width) {
      var offset = (y * width + x) * 4;
      if (normal) {
        // Engraved: the surface falls where there is ink, so its slope follows the ink's gradient.
        var left = ink[y * width + (x + width - 1) % width], right = ink[y * width + (x + 1) % width];
        var up = ink[Std.int(Math.max(0,
          y - 1)) * width + x], down = ink[Std.int(Math.min(height - 1, y + 1)) * width + x];
        var nx = (right - left) * 0.8, ny = (down - up) * 0.8, length = Math.sqrt(nx * nx + ny * ny + 1.0);
        base.set(offset, channel(nx / length));
        base.set(offset + 1, channel(ny / length));
        base.set(offset + 2, channel(1.0 / length));
      } else {
        var color = Std.int(Math.round(255 * (1.0 - 0.82 * ink[y * width + x])));
        base.set(offset, color);
        base.set(offset + 1, color);
        base.set(offset + 2, color);
      }
      base.set(offset + 3, 255);
    }
    return mipChain(base, width, height, normal);
  }

  /** Draws a capsule from (x0, y0) to (x1, y1) of half width `half` texels into `ink`, wrapping round in x. */
  static function stroke(ink:Array<Float>,
    width:Int, height:Int, x0:Float, y0:Float, x1:Float, y1:Float, half:Float):Void {
    var minX = Std.int(Math.floor(Math.min(x0, x1) - half - 1)), maxX = Std.int(Math.ceil(Math.max(x0, x1) + half + 1));
    var minY = Std.int(Math.max(0, Math.floor(Math.min(y0, y1) - half - 1)));
    var maxY = Std.int(Math.min(height - 1, Math.ceil(Math.max(y0, y1) + half + 1)));
    var dx = x1 - x0, dy = y1 - y0, length2 = dx * dx + dy * dy;
    for (y in minY...maxY + 1) for (x in minX...maxX + 1) {
      var px = x + 0.5 - x0, py = y + 0.5 - y0;
      var t = length2 < 1e-12 ? 0.0 : Math.max(0.0, Math.min(1.0, (px * dx + py * dy) / length2));
      var ex = px - t * dx, ey = py - t * dy;
      var coverage = Math.max(0.0, Math.min(1.0, 0.5 + half - Math.sqrt(ex * ex + ey * ey)));
      if (coverage <= 0) continue;
      var index = y * width + (((x % width) + width) % width);
      if (coverage > ink[index]) ink[index] = coverage;
    }
  }

  static function channel(value:Float):Int return Std.int(Math.round((value * 0.5 + 0.5) * 255));

  /** Every mip level below `base`, each half the size (at least one texel) of the last, appended after it. */
  public static function mipChain(base:Bytes, width:Int, height:Int, normal:Bool):DetailImage {
    var levels:Array<{w:Int, h:Int, offset:Int}> = [{w: width, h: height, offset: 0}];
    var total = width * height * 4, w = width, h = height;
    while (w > 1 || h > 1) {
      w = Std.int(Math.max(1, w / 2));
      h = Std.int(Math.max(1, h / 2));
      levels.push({w: w, h: h, offset: total});
      total += w * h * 4;
    }
    var result = Bytes.alloc(total);
    result.blit(0, base, 0, width * height * 4);
    for (level in 1...levels.length) {
      var source = levels[level - 1], target = levels[level];
      for (y in 0...target.h) for (x in 0...target.w) {
        var value = [0.0, 0.0, 0.0];
        var samples = 0;
        for (dy in 0...2) for (dx in 0...2) {
          var sx = Std.int(Math.min(source.w - 1, x * 2 + dx)), sy = Std.int(Math.min(source.h - 1, y * 2 + dy));
          var at = source.offset + (sy * source.w + sx) * 4;
          samples++;
          for (axis in 0...3) value[axis] += normal ? result.get(at + axis) / 127.5 - 1 : result.get(at + axis) * 1.0;
        }
        var out = target.offset + (y * target.w + x) * 4;
        // The mean vector is kept, with the variance of the finer levels, so the shader can filter roughness from its length.
        for (axis in 0...3) result.set(
          out + axis,
          normal ? channel(value[axis] / samples) : Std.int(Math.round(value[axis] / samples))
        );
        result.set(out + 3, 255);
      }
    }
    return new DetailImage(width, height, levels.length, result);
  }
}
