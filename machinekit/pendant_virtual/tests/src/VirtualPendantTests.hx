import haxe.io.Bytes;
import machinekit.operator.PendantDisplaySignals;
import machinekit.operator.VirtualPendant;

class VirtualPendantTests {
  static function check(condition:Bool, what:String):Void {
    if (!condition) throw what;
  }

  static function lit(pixels:Bytes):Int {
    var count = 0;
    for (i in 0...pixels.length) if (pixels.get(i) != 0) count++;
    return count;
  }

  /** The pendant layout's display: one display on the glass, framed on the active area, bound to the pendant's joints. */
  static function checkModelDisplay():Void {
    var module = new machinekit.hmi.OledModule();
    var plainBodies = 0;
    for (body in module.bodies()) if (body.id == "glass") {
      var plain = body.build();
      check(cadkit.threads.Details.renderData(plain.shape).length == 0, "an undriven module's glass is plain");
      plain.close();
      plainBodies++;
    }
    check(plainBodies == 1, "the module has a glass body");
    var layout = machinekit.pendant.ReferencePendant.layout();
    var oled:machinekit.hmi.OledModule = null;
    for (placed in layout.controls) if (placed.id == "display") oled = cast placed.part;
    check(oled != null, "the layout has a display");
    var found = 0;
    for (body in oled.bodies()) {
      if (body.id != "glass") continue;
      var glass = body.build();
      var details = cadkit.threads.Details.renderData(glass.shape);
      glass.close();
      check(details.length == 1, "the glass has one detail");
      var d = details[0];
      check(d.kind == materia.project.SurfaceDetailRenderData.SurfaceDetailKinds.DISPLAY, "it is a display");
      check(d.source == PendantDisplaySignals.SOURCE, "it names the pendant OLED source");
      check(d.profile == machinekit.pendant.ReferencePendant.PROFILE, "it names its operator profile");
      check(Math.abs(d.width - 57.01) < 1e-6 && Math.abs(d.height - 29.49) < 1e-6, "the active area's size");
      // x to the right and z toward the operator, so y is up; the active area is centred on the module's origin.
      check(Math.abs(d.xDirection[0] - 1) < 1e-9 && Math.abs(d.direction[2] - 1) < 1e-9, "the picture reads upright");
      check(Math.abs(d.origin[0]) < 1e-6 && Math.abs(d.origin[1]) < 1e-6, "centred on the module's origin");
      var joints = [for (j in machinekit.pendant.ReferencePendant.assembly().derived().definition.joints) j.id => true];
      var expected = [
        PendantDisplaySignals.AXIS,
        PendantDisplaySignals.STEP,
        PendantDisplaySignals.WHEEL,
        PendantDisplaySignals.WHEEL_ENABLE,
        PendantDisplaySignals.ESTOP
      ].concat(PendantDisplaySignals.keys());
      check(d.bindings.length == expected.length, "every signal is bound: " + d.bindings.length);
      for (binding in d.bindings) {
        check(expected.indexOf(binding.signal) >= 0, "a known signal: " + binding.signal);
        check(joints.exists(binding.joint), "a bound joint exists in the assembly: " + binding.joint);
      }
      materia.project.SurfaceDetailRenderData.SurfaceDetailRenderDataValues.validate(d);
      found++;
    }
    check(found == 1, "the display module has a glass body");
  }

  /** A model whose joints are all at rest, as a display driver reads them. */
  static function restingInputs():materia.project.LiveDisplay.LiveInputs {
    return new RestingInputs();
  }

  static function brightest(rgba:Bytes):Int {
    var most = 0;
    for (i in 0...Std.int(rgba.length / 4)) most = Std.int(Math.max(most, rgba.get(i * 4 + 2)));
    return most;
  }

  /** The display driver follows the pendant's own timeline: labels arrive, then it dims and blanks, whenever it is next called. */
  static function checkDriverTimeline():Void {
    var bindings = new Map<String, String>();
    for (binding in machinekit.pendant.ReferencePendant.displayBindings()) bindings.set(binding.signal, binding.joint);
    var config = new materia.project.LiveDisplay.LiveDisplayConfig(
      PendantDisplaySignals.SOURCE,
      machinekit.pendant.ReferencePendant.PROFILE,
      bindings
    );
    var driver = new machinekit.operator.PendantDisplayDriver(config);
    var inputs = restingInputs();
    var first = driver.advance(0, inputs);
    check(first != null && first.length == 128 * 64 * 4, "the driver draws a first picture");
    var now = 0;
    var changes = 0;
    while (now < 3000) {
      now += 16;
      if (driver.advance(now, inputs) != null) changes++;
    }
    check(changes >= 3, "the link comes up and the labels arrive while it runs: " + changes + " pictures");
    check(driver.advance(3000, inputs) == null, "nothing changes while nothing happens");
    // Pushing the E-stop's head opens the chain the pendant reads back, and it says so; releasing it clears the alert.
    var stopped = driver.advance(3010, new PressedInputs(bindings.get(PendantDisplaySignals.ESTOP)));
    check(stopped != null, "pushing the E-stop changes the picture");
    check(driver.advance(3020, inputs) != null, "releasing the E-stop changes it back");
    // The editor draws nothing for a while: the next call finds the display dimmed, and later blank.
    var dimmed = driver.advance(15000, inputs);
    check(dimmed != null && brightest(dimmed) > 0 && brightest(dimmed) < 120, "dimmed after ten seconds idle");
    var blank = driver.advance(70000, inputs);
    check(blank != null && brightest(blank) == 0, "blank after a minute idle");
  }

  static function main():Void {
    checkModelDisplay();
    checkDriverTimeline();
    var pendant = new VirtualPendant(0);
    var pixels = pendant.pixels(0);
    check(pixels.length == VirtualPendant.WIDTH * VirtualPendant.HEIGHT, "the OLED is 128x64");
    check(lit(pixels) > 100, "NO LINK is drawn");
    check(pendant.checksum(0) != 0, "the frame has a checksum");
    check(pendant.displayPower(0) == VirtualPendant.POWER_ON, "awake at power-up");

    check(pendant.tick(10, 0, 3, 1, 0, true, true), "a state frame is due at 10 ms");
    check(pendant.frame().length == 8, "PENDANT_STATE is 8 bytes");
    check(!pendant.tick(11, 0, 3, 1, 0, true, true), "none is due a millisecond later");
    check(!pendant.onFrame(12, 0x202, Bytes.alloc(0)), "an empty controller frame is refused");
    check(pendant.leds(12) == 0, "no LEDs without a controller");

    check(lit(pendant.pixels(70000)) == 0, "blanked after a minute idle");
    check(pendant.displayPower(70000) == VirtualPendant.POWER_OFF, "reports the display off");
    pendant.close();

    // A controller attached, with a profile compiled the way the host does: the link comes up and the screen changes.
    var compiled = machinekit.operator.PendantCompiler.compile(
      machinekit.operator.ReferenceOperatorProfiles.hobber(),
      [1, 10, 100],
      50,
      250
    );
    var linked = new VirtualPendant(0);
    var bare = linked.checksum(0);
    linked.attachController(0, compiled.profile, compiled.reasons, compiled.labels);
    var now = 0;
    while (now < 2000) {
      linked.tick(now, 0, 5, 0, 0, true, true);
      now += 16;
    }
    check(linked.checksum(now) != bare, "the screen follows the controller");
    check(linked.takeJog() == 0, "no wheel turns, no jog");
    linked.close();
    trace("virtual pendant tests passed");
  }
}

/** Every joint at rest: nothing pressed, every position the first. */
private class RestingInputs implements materia.project.LiveDisplay.LiveInputs {
  public function new() {
  }

  public function position(joint:String):Float return 0.0;

  public function detentIndex(joint:String):Int return 0;

  public function pressed(joint:String):Bool return false;
}

/** Every joint at rest except one that is pressed. */
private class PressedInputs extends RestingInputs {
  final joint:String;

  public function new(joint:String) {
    super();
    this.joint = joint;
  }

  override public function pressed(id:String):Bool return id == joint;
}
