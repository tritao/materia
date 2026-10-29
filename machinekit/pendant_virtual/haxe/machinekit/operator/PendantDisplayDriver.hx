package machinekit.operator;

import haxe.io.Bytes;
import materia.project.LiveDisplay;

/**
 * Fills a pendant's OLED with what the pendant firmware's own core draws. The pendant itself is a `PendantDevice`, shared with the
 * other outputs of the same pendant; this is the view of its screen.
 */
class PendantDisplayDriver implements LiveDisplaySource {
  /** The module's colour: lit pixels in a cool white, scaled down when the core dims the display. */
  static inline var RED:Int = 200;
  static inline var GREEN:Int = 230;
  static inline var BLUE:Int = 255;

  final device:PendantDevice;
  var released = false;
  var lastChecksum:Int = 0;
  var lastPower:Int = -1;
  var started = false;

  /** Makes this driver available to the editor under `PendantDisplaySignals.SOURCE`. */
  public static function register():Void {
    LiveDisplays.register(PendantDisplaySignals.SOURCE, function(config) return new PendantDisplayDriver(config));
  }

  public function new(config:LiveDisplayConfig) {
    device = PendantDevice.acquire(config);
  }

  public function width():Int return VirtualPendant.WIDTH;

  public function height():Int return VirtualPendant.HEIGHT;

  public function release():Void {
    if (released) return;
    released = true;
    device.release();
  }

  public function advance(nowMs:Int, inputs:LiveInputs):Null<Bytes> {
    device.update(nowMs, inputs);
    var power = device.displayPower(nowMs);
    var checksum = device.checksum(nowMs);
    if (started && power == lastPower && checksum == lastChecksum) return null;
    started = true;
    lastPower = power;
    lastChecksum = checksum;
    var pixels = device.pixels(nowMs);
    var scale = power == VirtualPendant.POWER_DIM ? 0.3 : 1.0;
    var out = Bytes.alloc(pixels.length * 4);
    for (i in 0...pixels.length) {
      var lit = pixels.get(i) != 0;
      out.set(i * 4, lit ? Std.int(RED * scale) : 0);
      out.set(i * 4 + 1, lit ? Std.int(GREEN * scale) : 0);
      out.set(i * 4 + 2, lit ? Std.int(BLUE * scale) : 0);
      out.set(i * 4 + 3, 255);
    }
    return out;
  }
}
