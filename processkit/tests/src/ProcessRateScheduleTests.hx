import motionkit.event.EventValue;
import motionkit.event.PathEvent;
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
    var normal = [new PathEvent(0.0, "wire", EventValue.Analog(8.0)),
      new PathEvent(0.004, "wire", EventValue.Analog(6.0))];
    var recovery = ProcessRateSchedule.overlap(normal, 0.002, 1.0);
    check(recovery.length == 3 && recovery[1].distance == 0.002,
      "the normal dose returns exactly where previous deposition ended");
    switch recovery[0].value {
      case Analog(rate): check(rate == 1.0, "already deposited material gets only maintenance feed");
      case _: throw "Expected an analog rate";
    }
    switch recovery[1].value {
      case Analog(rate): check(rate == 8.0, "fresh seam restores its validated rate");
      case _: throw "Expected an analog rate";
    }
    check(recovery[2] == normal[1], "later joint-limit rate changes are preserved");
    var exact = ProcessRateSchedule.overlap(normal, 0.004, 1.0);
    check(exact.length == 2 && exact[1] == normal[1], "a boundary coinciding with a rate change is not duplicated");
    var early=ProcessRateSchedule.timedSection("wire",2,[0.0,0.01],[0.0,1.0],[],false,8,0.015,1);
    check(early.length==1 && early[0].distance==0,"covered section emits maintenance without an out-of-section restore");
    var late=ProcessRateSchedule.timedSection("wire",2,[0.01,0.02,0.03],[0.0,2.0,3.0],
      [new PathEvent(0.02,"arc",EventValue.Digital(true))],true,8,0.015,1);
    check(late.length==5 && late[1].distance==0.015 && late[4].distance==0.03,
      "later timed section restores normal feed inside the overlap and emits its final rate");
    var internalRejected=false;
    try ProcessRateSchedule.timedSection("wire",2,[0.01,0.02],[0.0,1.0],
      [new PathEvent(0.01,"wire",EventValue.Analog(3))],true,8) catch (_:Dynamic) internalRejected=true;
    check(internalRejected,"section boundaries cannot hide an internal continuous-process transition");
    var roundedEnd=ProcessRateSchedule.timedSection("wire",2,[0.01,0.03],[0.0,1.0],
      [new PathEvent(0.03-1e-15,"wire",EventValue.Analog(0))],true,8);
    check(roundedEnd.length==2 && roundedEnd[1].distance==0.03,
      "whole-path endpoint roundoff still replaces the authored final rate");
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
