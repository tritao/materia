package app.editor;

import haxe.io.Bytes;
import machinekit.hmi.OledModule;
import machinekit.operator.VirtualPendant;

/** The live sources the editor can attach to a display, by the name a model gives it (`SurfaceDetail.Display`). */
class LiveDisplays {
  public static function create(source:String):Null<LiveDisplaySource> {
    return source == OledModule.DISPLAY_SOURCE ? new PendantOledSource() : null;
  }
}

/**
 * The pendant's OLED as the pendant firmware's own core draws it. The axis and step selectors and the wheel are the model's
 * joints, read as the pendant would read them. No controller is attached yet, so the core shows what the pendant shows with
 * no link.
 */
class PendantOledSource implements LiveDisplaySource {
  /** The module's colour: lit pixels in a cool white, scaled down when the core dims the display. */
  static inline var RED:Int = 200;
  static inline var GREEN:Int = 230;
  static inline var BLUE:Int = 255;

  static inline var AXIS:String = "selectors/axis/position";
  static inline var STEP:String = "selectors/step/position";
  static inline var WHEEL:String = "jog/wheel/encoder";
  static inline var WHEEL_ENABLE:String = "jog/wheel-enable/press";
  /** The keys in the order of PENDANT_STATE's button bits: Start, Hold, Stop, F1, F2, F3. */
  static final KEYS:Array<String> = [
    "cycle-keys/start/press",
    "cycle-keys/hold/press",
    "cycle-keys/stop/press",
    "function-keys/f1/press",
    "function-keys/f2/press",
    "function-keys/f3/press"
  ];
  /** The 100 PPR wheel, read in quadrature. */
  static inline var COUNTS_PER_TURN:Int = 400;

  final pendant:VirtualPendant;
  var lastChecksum:Int = 0;
  var lastPower:Int = -1;
  var started = false;

  public function new() {
    pendant = new VirtualPendant(0);
  }

  public function width():Int return VirtualPendant.WIDTH;

  public function height():Int return VirtualPendant.HEIGHT;

  public function advance(nowMs:Int, controls:LiveControls):Null<Bytes> {
    var counts = Math.round(controls.position(WHEEL) / 360 * COUNTS_PER_TURN);
    var keys = 0;
    for (bit in 0...KEYS.length) if (controls.pressed(KEYS[bit])) keys |= 1 << bit;
    pendant.tick(
      nowMs,
      counts,
      controls.detentIndex(AXIS),
      controls.detentIndex(STEP),
      keys,
      controls.pressed(WHEEL_ENABLE),
      true
    );
    var power = pendant.displayPower(nowMs);
    var checksum = pendant.checksum(nowMs);
    if (started && power == lastPower && checksum == lastChecksum) return null;
    started = true;
    lastPower = power;
    lastChecksum = checksum;
    var pixels = pendant.pixels(nowMs);
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
