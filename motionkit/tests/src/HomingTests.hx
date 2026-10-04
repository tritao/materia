import haxe.Int64;
import motionkit.robot.HomingAxis;
import motionkit.robot.HomingCycle;
import motionkit.robot.HomingDriver;
import motionkit.robot.HomingDriver.HomingObservation;
import motionkit.robot.HomingDriver.HomingSwitchObservation;
import robotkit.model.JointSwitch;
import robotkit.runtime.HomingSideControl;

/** Cycle sequencing and cleanup; physical endpoint behavior is covered by native simkit. */
class HomingTests {
  static function check(value:Bool, message:String):Void { if (!value) throw message; }
  static function approach(driver:HomingFixture, cycle:HomingCycle):Void {
    cycle.start();
    driver.next(-0.001, true, false, 1, 0); cycle.update(0.01);
    driver.next(-0.001, true, false, 1, 0); cycle.update(0.01);
    driver.next(0.0, false, false, 1, 0); cycle.update(0.01);
    driver.next(0.002, false, false, 1, 0); cycle.update(0.01);
    driver.next(0.002, false, false, 1, 0); cycle.update(0.01);
    check(cycle.status() == "Approach", "Home must release and stop before slow approach");
    driver.events = [];
  }
  public static function run():Void {
    var driver = new HomingFixture(), cycle = driver.cycle();
    approach(driver, cycle);
    driver.next(-0.001, true, false, 2, 0); cycle.update(0.01);
    check(driver.events.join(",") == "hold:left", "First captured side must hold independently");
    driver.next(-0.002, true, true, 2, 1); cycle.update(0.01);
    driver.calibrationReady = false;
    driver.next(-0.002, true, true, 2, 1); cycle.update(0.01);
    check(driver.events.join(",") == "hold:left,hold:right,stop",
      "Home must keep side holds until the whole endpoint is ready for calibration");
    driver.calibrationReady = true;
    driver.next(-0.002, true, true, 2, 1); cycle.update(0.01);
    check(driver.events.join(",") == "hold:left,hold:right,stop,release,latch:left,latch:right,calibrate,end,return",
      "Home must stop, release, latch and calibrate all sides before returning");
    check(driver.leaderCaptureSeen == -0.002, "Leader latch must include travel after first side held");
    driver.next(0.0, false, false, 2, 1); cycle.update(0.01);
    check(cycle.isComplete(), "Referenced return must finish homing");

    driver = new HomingFixture(); cycle = driver.cycle(); approach(driver, cycle);
    driver.next(-0.001, true, false, 1, 0);
    var rejected = false;
    try cycle.update(0.01) catch (_:Dynamic) rejected = true;
    check(rejected && cycle.status() == "Fault" && driver.events.indexOf("end") >= 0,
      "An old edge capture must fault and close squaring");

    driver = new HomingFixture(); cycle = driver.cycle(); cycle.start();
    driver.failStop = true;
    try cycle.cancel() catch (_:Dynamic) {}
    check(cycle.status() == "Fault" && driver.events.indexOf("end") >= 0,
      "Cancellation must close squaring even when stopping fails");
    Sys.println("homing: side hold order, compensated capture, stale edge and cancellation cleanup");
  }
}

class HomingFixture implements HomingDriver implements HomingSideControl {
  public var events:Array<String> = [];
  public var failStop:Bool = false;
  public var calibrationReady:Bool = true;
  public var leaderCaptureSeen:Float = 0.0;
  var tick:Int = 1;
  var position:Float = 0.0;
  var left:Bool = false;
  var right:Bool = false;
  var leftEdges:Int = 0;
  var rightEdges:Int = 0;
  public function new() {}
  public function cycle():HomingCycle {
    var contacts = [new JointSwitch("left", "y", "leftFrame", "home", -1, -0.001, 0.0002, 0.00001, 1, "leftMotor"),
      new JointSwitch("right", "y", "rightFrame", "home", -1, -0.001, 0.0002, 0.00001, 2, "rightMotor")];
    return new HomingCycle(this, [new HomingAxis("y", 0, contacts, 0.1, 1.0, 0.0, 0.1, 0.01, 0.0, 0.01)], this);
  }
  public function next(position:Float, left:Bool, right:Bool, leftEdges:Int, rightEdges:Int):Void {
    tick++; this.position = position; this.left = left; this.right = right;
    this.leftEdges = leftEdges; this.rightEdges = rightEdges;
  }
  public function observe(joint:Int):HomingObservation {
    var sequence = Int64.ofInt(tick), time = Int64.ofInt(tick * 10000000);
    return new HomingObservation(position, 0.0, [
      new HomingSwitchObservation("left", left, sequence, time, "fixture", leftEdges > 0 ? -0.001 : null, leftEdges),
      new HomingSwitchObservation("right", right, sequence, time, "fixture", rightEdges > 0 ? -0.002 : null, rightEdges)], time, "fixture", calibrationReady);
  }
  public function velocity(joint:Int, velocity:Float, acceleration:Float):Void events.push("velocity");
  public function stop(joint:Int, acceleration:Float):Void { events.push("stop"); if (failStop) throw "stop failed"; }
  public function latch(id:String, position:Float, leaderCounterPosition:Null<Float>):Void {
    events.push("latch:" + id);
    if (id == "left" && leaderCounterPosition != null) leaderCaptureSeen = leaderCounterPosition;
  }
  public function returnHome(joint:Int, position:Float, velocity:Float, acceleration:Float):Void events.push("return");
  public function beginSquaring(ids:Array<String>):Void events.push("begin");
  public function endSquaring():Void events.push("end");
  public function hold(id:String):Void events.push("hold:" + id);
  public function releaseAll():Void events.push("release");
  public function calibrate(ids:Array<String>):Void events.push("calibrate");
  public function leaderCapture(id:String, sideCapture:Float):Float return id == "left" ? sideCapture - 0.001 : sideCapture;
}

function main():Void HomingTests.run();
