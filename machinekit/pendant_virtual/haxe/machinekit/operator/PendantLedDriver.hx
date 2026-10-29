package machinekit.operator;

import haxe.io.Bytes;
import materia.project.LiveDisplay;

/**
 * Lights a key's cap: the picture is one colour, the key's own, bright while the pendant's core says the key's LED is on and
 * dim (the cap's colour showing through) while it is off. A view of the shared `PendantDevice`, like the OLED.
 */
class PendantLedDriver implements LiveDisplaySource {
  /** The picture is a single colour; the cap's face is a disc that shows it. */
  static inline var SIZE:Int = 4;
  /** The share of the colour an unlit cap keeps. */
  static inline var OFF:Float = 0.25;

  final device:PendantDevice;
  final bit:Int;
  final colour:Array<Int>;
  var released = false;
  var shown = -1;

  /** Makes this driver available to the editor under `PendantDisplaySignals.LED_SOURCE`. */
  public static function register():Void {
    LiveDisplays.register(PendantDisplaySignals.LED_SOURCE, function(config) return new PendantLedDriver(config));
  }

  public function new(config:LiveDisplayConfig) {
    bit = PendantDisplaySignals.keys().indexOf(config.channel);
    if (bit < 0) throw 'The pendant has no key LED called "${config.channel}"';
    colour = PendantDisplaySignals.ledColour(config.channel);
    device = PendantDevice.acquire(config);
  }

  public function width():Int return SIZE;

  public function height():Int return SIZE;

  public function takeJogs():Array<LiveJog> return [];

  public function release():Void {
    if (released) return;
    released = true;
    device.release();
  }

  public function advance(nowMs:Int, inputs:LiveInputs):Null<Bytes> {
    device.update(nowMs, inputs);
    var on = (device.leds(nowMs) >> bit) & 1;
    if (on == shown) return null;
    shown = on;
    var scale = on == 1 ? 1.0 : OFF;
    var out = Bytes.alloc(SIZE * SIZE * 4);
    for (i in 0...SIZE * SIZE) {
      out.set(i * 4, Std.int(colour[0] * scale));
      out.set(i * 4 + 1, Std.int(colour[1] * scale));
      out.set(i * 4 + 2, Std.int(colour[2] * scale));
      out.set(i * 4 + 3, 255);
    }
    return out;
  }
}
