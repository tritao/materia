package machinekit.operator;

import haxe.io.Bytes;
import materia.project.LiveDisplay;

/**
 * Fills a pendant's OLED with what the pendant firmware's own core draws. The controls are the model's joints, read through the
 * display's bindings (`PendantDisplaySignals`), and a controller (the real operator service, serving the display's profile)
 * answers, so the screen shows its labels, LEDs and refusals.
 */
class PendantDisplayDriver implements LiveDisplaySource {
  /** The module's colour: lit pixels in a cool white, scaled down when the core dims the display. */
  static inline var RED:Int = 200;
  static inline var GREEN:Int = 230;
  static inline var BLUE:Int = 255;
  /** The 100 PPR wheel, read in quadrature. */
  static inline var COUNTS_PER_TURN:Int = 400;
  /** Long enough for the controller to bring the link up and send every label. */
  static inline var SETTLE_MS:Int = 3000;

  final pendant:VirtualPendant;
  final config:LiveDisplayConfig;
  var lastChecksum:Int = 0;
  var lastPower:Int = -1;
  var started = false;
  var lastSignature:String = "";
  var busyUntil:Int = 0;
  var lastNowMs:Int = 0;

  /** Makes this driver available to the editor under `PendantDisplaySignals.SOURCE`. */
  public static function register():Void {
    LiveDisplays.register(PendantDisplaySignals.SOURCE, function(config) return new PendantDisplayDriver(config));
  }

  public function new(config:LiveDisplayConfig) {
    this.config = config;
    pendant = new VirtualPendant(0);
    var profile = ReferenceOperatorProfiles.named(config.profile);
    if (profile == null) throw 'The pendant display has no operator profile called "${config.profile}"';
    var compiled = PendantCompiler.compile(profile, [1, 10, 100], 50, 250);
    pendant.attachController(0, compiled.profile, compiled.reasons, compiled.labels);
  }

  public function width():Int return VirtualPendant.WIDTH;

  public function height():Int return VirtualPendant.HEIGHT;

  public function busy():Bool return lastNowMs < busyUntil;

  public function advance(nowMs:Int, inputs:LiveInputs):Null<Bytes> {
    var counts = Math.round(inputs.position(config.joint(PendantDisplaySignals.WHEEL)) / 360 * COUNTS_PER_TURN);
    var keys = 0;
    var names = PendantDisplaySignals.keys();
    for (bit in 0...names.length) if (inputs.pressed(config.joint(names[bit]))) keys |= 1 << bit;
    var axis = inputs.detentIndex(config.joint(PendantDisplaySignals.AXIS));
    var step = inputs.detentIndex(config.joint(PendantDisplaySignals.STEP));
    var enabled = inputs.pressed(config.joint(PendantDisplaySignals.WHEEL_ENABLE));
    var signature = [counts, axis, step, keys, enabled ? 1 : 0].join(",");
    // The controller answers over the next frames (the link comes up, a label arrives every 100 ms): keep drawing for a while after
    // the start and after every change of the controls.
    if (signature != lastSignature || !started) {
      lastSignature = signature;
      busyUntil = nowMs + SETTLE_MS;
    }
    lastNowMs = nowMs;
    pendant.takeJog();
    pendant.tick(nowMs, counts, axis, step, keys, enabled, true);
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
