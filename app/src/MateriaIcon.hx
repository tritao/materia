import nativekit.ui.host.WindowIcon;
import nativekit.ui.host.ApplicationIconSet;
import haxe.io.Bytes;

/** Small vector drawing rasterized into the executable at startup. */
class MateriaIcon {
  public static function create():ApplicationIconSet {
    return new ApplicationIconSet([for (size in [16, 32, 64, 128, 256]) render(size)]);
  }

  static function render(size:Int):WindowIcon {
    var pixels = Bytes.alloc(size * size * 4);
    for (y in 0...size) {
      for (x in 0...size) {
        var px = (x + 0.5) * 64 / size - 0.5;
        var py = (y + 0.5) * 64 / size - 0.5;
        var dx = Math.max(0, Math.abs(px - 31.5) - 23.5);
        var dy = Math.max(0, Math.abs(py - 31.5) - 23.5);
        var inside = dx * dx + dy * dy <= 8 * 8;
        var color:Int = inside ? 0x172438ff : 0x00000000;
        if (inside) {
          var first = segmentDistance(px, py, 16, 45, 16, 19);
          var second = segmentDistance(px, py, 16, 19, 32, 37);
          var third = segmentDistance(px, py, 32, 37, 48, 19);
          var fourth = segmentDistance(px, py, 48, 19, 48, 45);
          if (Math.min(Math.min(first, second), Math.min(third, fourth)) <= 3)
            color = 0xf3f7ffff;
          if ((px - 49) * (px - 49) + (py - 13) * (py - 13) <= 16)
            color = 0xefa936ff;
        }
        var offset = (y * size + x) * 4;
        pixels.set(offset, color >>> 24);
        pixels.set(offset + 1, (color >>> 16) & 0xff);
        pixels.set(offset + 2, (color >>> 8) & 0xff);
        pixels.set(offset + 3, color & 0xff);
      }
    }
    return new WindowIcon(size, size, size * 4, pixels);
  }

  static function segmentDistance(x:Float, y:Float, ax:Float, ay:Float,
      bx:Float, by:Float):Float {
    var vx = bx - ax;
    var vy = by - ay;
    var t = Math.max(0, Math.min(1, ((x - ax) * vx + (y - ay) * vy) /
      (vx * vx + vy * vy)));
    var dx = x - ax - t * vx;
    var dy = y - ay - t * vy;
    return Math.sqrt(dx * dx + dy * dy);
  }
}
