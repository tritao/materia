package tests;

import haxe.Int64;
import robotkit.time.ClockMapping;
import robotkit.time.ClockMappings;

class ClockMappingTests {
  static var assertions = 0;
  static function check(value:Bool, message:String):Void {
    assertions++;
    if (!value) throw message;
  }
  static function equal(value:Int64, expected:Int64, message:String):Void
    check(Int64.compare(value, expected) == 0, message + ': got ' + Int64.toStr(value));
  static function throws(action:Void->Void, fragment:String):Void {
    var message = "";
    try action() catch (error:Dynamic) message = Std.string(error);
    check(message.indexOf(fragment) >= 0, 'Expected "$fragment", got "$message"');
  }

  public static function run():Int {
    var registry = new ClockMappings();
    var epoch = Int64.parseString("1700000000000000000");
    var identity = registry.map(epoch, "camera", "camera");
    check(identity != null, "identity mapping exists without a registry entry");
    if (identity != null) {
      equal(identity.valueNs, epoch, "identity keeps a large epoch exact");
      equal(identity.errorBoundNs, Int64.ofInt(0), "identity has zero bound");
    }
    check(registry.map(epoch, "camera", "host") == null,
      "missing relationship is never guessed");

    var hostAnchor = Int64.add(epoch, Int64.ofInt(1000));
    registry.add(new ClockMapping("camera", "host", Int64.ofInt(1000),
      1000.0, Int64.ofInt(20), hostAnchor, "calibration"));
    var atAnchor = registry.map(epoch, "camera", "host");
    check(atAnchor != null, "explicit mapping is available at its anchor");
    if (atAnchor != null) equal(atAnchor.valueNs, hostAnchor, "offset uses target-clock anchor");
    check(registry.map(Int64.sub(epoch, Int64.ofInt(1)), "camera", "host") == null,
      "target-clock validFrom rejects earlier values");
    var later = registry.map(Int64.add(epoch, Int64.ofInt(2000000000)), "camera", "host");
    check(later != null, "skewed mapping covers a later interval");
    if (later != null) {
      equal(later.valueNs, Int64.add(hostAnchor, Int64.ofInt(2000002000)),
        "1000 ppb adds 2000 ns over two seconds");
      equal(later.errorBoundNs, Int64.ofInt(20), "one edge keeps its bound");
    }
    check(registry.map(hostAnchor, "host", "camera") == null,
      "inverse mapping is never inferred");

    registry.add(new ClockMapping("host", "world", Int64.ofInt(-50), 0,
      Int64.ofInt(7), Int64.sub(hostAnchor, Int64.ofInt(50)), "sync session"));
    var chained = registry.map(epoch, "camera", "world");
    check(chained != null, "explicit chain maps across two clocks");
    if (chained != null) {
      equal(chained.valueNs, Int64.sub(hostAnchor, Int64.ofInt(50)),
        "chain composes offsets");
      equal(chained.errorBoundNs, Int64.ofInt(27), "chain sums error bounds");
    }
    registry.add(new ClockMapping("camera", "world", Int64.ofInt(900), 0,
      Int64.ofInt(50), Int64.add(epoch, Int64.ofInt(900)), "alternate"));
    var selected = registry.map(epoch, "camera", "world");
    check(selected != null, "valid alternate path is considered");
    if (selected != null) equal(selected.errorBoundNs, Int64.ofInt(27),
      "least-error explicit path wins");
    registry.add(new ClockMapping("camera", "world", Int64.ofInt(951), 0,
      Int64.ofInt(27), Int64.add(epoch, Int64.ofInt(951)), "replacement"));
    check(registry.map(epoch, "camera", "world") != null,
      "one-nanosecond rounding difference is tolerated");
    registry.add(new ClockMapping("camera", "world", Int64.ofInt(952), 0,
      Int64.ofInt(27), Int64.add(epoch, Int64.ofInt(952)), "replacement"));
    check(registry.map(epoch, "camera", "world") == null,
      "larger equal-bound disagreement is ambiguous");
    var staged = new ClockMappings();
    staged.add(new ClockMapping("a", "b", Int64.ofInt(5), 0,
      Int64.ofInt(2), Int64.ofInt(5), "first"));
    staged.add(new ClockMapping("b", "c", Int64.ofInt(7), 0,
      Int64.ofInt(3), Int64.ofInt(20), "second"));
    check(staged.map(Int64.ofInt(7), "a", "c") == null &&
      staged.map(Int64.ofInt(8), "a", "c") != null,
      "validFrom on the second edge is enforced midway through a chain");

    var longMapping = new ClockMapping("sim", "host", Int64.ofInt(0), 100000.0,
      Int64.ofInt(1), Int64.ofInt(0), "long run", 10.0,
      Int64.parseString("30003000000000"));
    var longResult = longMapping.map(Int64.parseString("30000000000000"));
    check(longResult != null, "eight-hour correction is representable");
    if (longResult != null) {
      equal(longResult.valueNs, Int64.parseString("30003000000000"),
        "100 ppm over 8.3 h corrects three seconds without Int32 wrap");
      equal(longResult.errorBoundNs, Int64.ofInt(300001),
        "skew uncertainty grows with anchor distance");
    }
    check(longMapping.map(Int64.parseString("30000000000001")) == null,
      "validity window expires");
    var zeroSkew = new ClockMapping("a", "b", Int64.ofInt(0), 0,
      Int64.ofInt(0), Int64.ofInt(0), "test");
    check(zeroSkew.map(Int64.parseString("9007199254740992")) != null,
      "zero skew accepts deltas beyond floating-point exact range");
    var negative = new ClockMapping("a", "b", Int64.ofInt(0), -100000.0,
      Int64.ofInt(0), Int64.ofInt(0), "test");
    var negativeResult = negative.map(Int64.parseString("30000000000000"));
    check(negativeResult != null, "negative skew maps long intervals");
    if (negativeResult != null)
      equal(negativeResult.valueNs, Int64.parseString("29997000000000"),
        "negative skew correction is not truncated");
    var nearMax = new ClockMapping("a", "b", Int64.ofInt(0), 1000000000.0,
      Int64.ofInt(0), Int64.parseString("9223372036854775800"), "test");
    check(nearMax.map(Int64.parseString("9223372036854775807")) == null,
      "positive overflow is rejected");
    check(ClockMapping.checkedAdd(Int64.parseString("-9223372036854775808"),
      Int64.ofInt(-1)) == null, "negative overflow is rejected");

    var boundOnly = new ClockMappings();
    boundOnly.add(new ClockMapping("a", "b", Int64.ofInt(0), 0,
      Int64.ofInt(3), Int64.ofInt(0), "test"));
    boundOnly.add(new ClockMapping("b", "a", Int64.ofInt(0), 0,
      Int64.ofInt(3), Int64.ofInt(0), "test"));
    check(boundOnly.map(Int64.ofInt(10), "a", "missing") == null,
      "cycles do not invent a destination");
    throws(function() new ClockMapping("a", "a", Int64.ofInt(0), 0,
      Int64.ofInt(0), Int64.ofInt(0), "test"), "distinct clocks");
    throws(function() new ClockMapping("a", "b", Int64.ofInt(0), 0,
      Int64.ofInt(-1), Int64.ofInt(0), "test"), "non-negative bound");
    throws(function() new ClockMapping("a", "b", Int64.ofInt(0), -1000000000.0,
      Int64.ofInt(0), Int64.ofInt(0), "test"), "positive rate");

    Sys.println('RobotKit clock mapping tests passed ($assertions assertions)');
    return assertions;
  }
}
