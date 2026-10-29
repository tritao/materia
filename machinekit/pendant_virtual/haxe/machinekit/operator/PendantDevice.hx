package machinekit.operator;

import haxe.io.Bytes;
import materia.project.LiveDisplay;

/**
 * One simulated pendant, shared by everything in a scene that shows a part of it (the OLED now, the key LEDs next): the firmware's
 * own core and the real operator service behind a `VirtualPendant`, fed from the model's joints through the display's bindings.
 * `acquire` gives every output of one pendant the same device, so a pendant is simulated once however many outputs it has;
 * each output calls `update` with the frame's time and the first call of a frame does the work.
 */
class PendantDevice {
  /** The 100 PPR wheel, read in quadrature. */
  static inline var COUNTS_PER_TURN:Int = 400;

  static final shared:Map<String, PendantDevice> = [];

  final key:String;
  final config:LiveDisplayConfig;
  final pendant:VirtualPendant;
  final profile:OperatorProfile;
  final jogs:Array<LiveJog> = [];
  var users = 0;
  var updatedAt = -1;

  /** The device for this profile and these bindings, started on first use and shared until every user has released it. */
  public static function acquire(config:LiveDisplayConfig):PendantDevice {
    var names = [for (name in config.bindings.keys()) name];
    names.sort(Reflect.compare);
    var key = config.profile + "|" + [for (name in names) name + "=" + config.bindings.get(name)].join(",");
    var device = shared.get(key);
    if (device == null) {
      device = new PendantDevice(key, config);
      shared.set(key, device);
    }
    device.users++;
    return device;
  }

  function new(key:String, config:LiveDisplayConfig) {
    this.key = key;
    this.config = config;
    pendant = new VirtualPendant(0);
    var profile = ReferenceOperatorProfiles.named(config.profile);
    this.profile = profile;
    if (profile == null) throw 'The pendant display has no operator profile called "${config.profile}"';
    var compiled = PendantCompiler.compile(profile, [1, 10, 100], 50, 250);
    pendant.attachController(0, compiled.profile, compiled.reasons, compiled.labels);
  }

  /** How many pendants are running. */
  public static function active():Int {
    var count = 0;
    for (_ in shared) count++;
    return count;
  }

  /** Gives the device up; the last one to do so stops it. */
  public function release():Void {
    if (--users <= 0) shared.remove(key);
  }

  /**
   * Runs the pendant up to `nowMs` with the controls as the model's joints are now. The core runs on its own 10 ms timeline up to
   * there (see `pendant_virtual_tick`), so the link coming up, the labels arriving and the display dimming and blanking happen at
   * their times whenever this is next called. Calling it again with the same time does nothing.
   */
  public function update(nowMs:Int, inputs:LiveInputs):Void {
    if (nowMs == updatedAt) return;
    updatedAt = nowMs;
    var counts = Math.round(inputs.position(config.joint(PendantDisplaySignals.WHEEL)) / 360 * COUNTS_PER_TURN);
    var keys = 0;
    var names = PendantDisplaySignals.keys();
    for (bit in 0...names.length) if (inputs.pressed(config.joint(names[bit]))) keys |= 1 << bit;
    var axis = inputs.detentIndex(config.joint(PendantDisplaySignals.AXIS));
    var step = inputs.detentIndex(config.joint(PendantDisplaySignals.STEP));
    var enabled = inputs.pressed(config.joint(PendantDisplaySignals.WHEEL_ENABLE));
    // The E-stop's head pushed in opens the chain; a model without one leaves it closed.
    var chainClosed = !inputs.pressed(config.joint(PendantDisplaySignals.ESTOP));
    pendant.tick(nowMs, counts, axis, step, keys, enabled, chainClosed);
    // What the controller let through to the host: a jog of one of the profile's axes, which the model binds to a joint (or not).
    while (true) {
      var jog = pendant.takeJog();
      if (jog == null) break;
      var joint = config.joint(PendantDisplaySignals.jog(profile.axes[jog.axis].id));
      if (joint != "") jogs.push(new LiveJog(joint, jog.distance * PendantJogDriver.UNIT));
    }
  }

  /** The jogs since they were last taken, for the model's joints. */
  public function takeJogs():Array<LiveJog> {
    return jogs.splice(0, jogs.length);
  }

  /** The key LEDs, in the order of `PendantDisplaySignals.keys()`: bit n is key n's LED. */
  public function leds(nowMs:Int):Int return pendant.leds(nowMs);

  public function displayPower(nowMs:Int):Int return pendant.displayPower(nowMs);

  public function checksum(nowMs:Int):Int return pendant.checksum(nowMs);

  public function pixels(nowMs:Int):Bytes return pendant.pixels(nowMs);
}
