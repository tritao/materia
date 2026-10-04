import motionkit.event.EventValue;
import processkit.ProcessRateSchedule;

class ProcessRateScheduleTests {
  static var assertions:Int = 0;

  public static function run():Void {
    // The middle centimetre takes ten times longer as a wrist-limited corner turns.
    var distances = [0.0, 0.01, 0.02, 0.03];
    var times = [0.0, 1.0, 11.0, 12.0];
    var events = ProcessRateSchedule.section("wire", 2.0, distances, times);
    check(events.length == 3, "every timed interval gets its rate");
    var deposited = 0.0;
    for (i in 0...events.length) switch events[i].value {
      case Analog(rate):
        check(Math.abs(rate * (times[i + 1] - times[i]) - 0.02) < 1e-12,
          "a slow interval deposits the same quantity per metre");
        deposited += rate * (times[i + 1] - times[i]);
      case _: throw "Expected an analog rate";
    }
    check(Math.abs(deposited - 0.06) < 1e-12, "total deposited quantity follows length");
    var compact = ProcessRateSchedule.section("wire", 2.0, distances, times, 1);
    check(compact.length == 1, "the device event budget bounds the schedule");
    switch compact[0].value {
      case Analog(rate): check(Math.abs(rate * 12.0 - 0.06) < 1e-12,
        "coalescing preserves deposited quantity");
      case _: throw "Expected an analog rate";
    }
    var later = ProcessRateSchedule.section("wire", 2.0, [0.03, 0.04], [0.0, 2.0]);
    check(later[0].distance == 0.03, "a later section keeps whole-path progress");
    rejects([0.0, 0.01], [0.0, 0.0]);
    rejects([0.01, 0.0], [0.0, 1.0]);
    rejects([0.0, 0.01], [0.0]);
    Sys.println('ProcessKit rate schedule tests passed ($assertions assertions)');
  }

  static function rejects(distances:Array<Float>, times:Array<Float>):Void {
    var failed = false;
    try ProcessRateSchedule.section("wire", 2.0, distances, times) catch (_:Dynamic) failed = true;
    check(failed, "invalid timing is rejected before scheduling outputs");
  }

  static function check(ok:Bool, message:String):Void {
    if (!ok) throw message;
    assertions++;
  }
}
