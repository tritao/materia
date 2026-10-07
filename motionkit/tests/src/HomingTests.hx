import haxe.Int64;
import motionkit.robot.HomingAxis;
import motionkit.robot.HomingCycle;
import motionkit.robot.HomingDriver;
import motionkit.robot.HomingDriver.HomingObservation;
import motionkit.robot.HomingDriver.HomingSwitchObservation;
import robotkit.model.JointSwitch;
import robotkit.runtime.HomingSideControl;
import robotkit.runtime.HomingControlReadiness;

/** Cycle sequencing and cleanup; physical endpoint behavior is covered by native simkit. */
class HomingTests {
  static function check(value:Bool, message:String):Void { if (!value) throw message; }
  static function approach(driver:HomingFixture, cycle:HomingCycle):Void {
    cycle.start();
    driver.next(-0.001, true, false, 1, 0); cycle.update(0.01);
    driver.next(-0.001, true, false, 1, 0); cycle.update(0.01);
    for (_ in 0...6) {
      if (cycle.status() != "Backoff") break;
      driver.next(driver.moves[driver.moves.length - 1], false, false, 1, 0); cycle.update(0.01);
    }
    check(cycle.status() == "Approach", "Home must release and stop before slow approach");
    check(driver.moves.length > 0 && driver.speeds.length == 2 && driver.speeds[1] <= driver.speeds[0],
      "Cycle uses finite backoff plans between search and capture-aware approach");
    driver.events = [];
  }
  public static function run():Void {
    // Fast switch release must not create a long precision-speed return.
    var contact = new JointSwitch("home", "z", "frame", "home", -1, -0.001, 0.0002, 0.00002, 1);
    var axis = new HomingAxis("z", 0, [contact], 1.0, 2.0, 0.0, 0.5, 0.01, 0.0, 0.02);
    check(axis.dynamics.stoppingDistance(axis.seekSpeed) <= 0.009 + 1e-12,
      "Search speed accounts for latency, acceleration and jerk before the end stop");
    check(axis.capturedApproachSpeed > axis.latchSpeed,
      "Captured edges permit a faster approach than sampled positions");
    var slow = new motionkit.robot.HomingDynamics(2.0, 20.0, 0.1, 0.02);
    check(slow.speedForRoom(0.001, 1.0) < axis.dynamics.speedForRoom(0.001, 1.0),
      "A longer response and tighter jerk limit reduce safe switch approach speed");
    function home(id:String, after:Array<String>):HomingAxis {
      return new HomingAxis("axis-" + id, id == "lift" ? 0 : id == "rail" ? 1 : 2,
        [new JointSwitch("switch-" + id, id, "frame", "home", -1, -0.001, 0.0002, 0.00002, 1, null, after)],
        0.1, 1.0, 0.0, 0.5, 0.01, 0.0, 0.01);
    }
    var ordered = motionkit.robot.HomingSequence.order([home("rail", ["lift"]), home("lift", [])]);
    check(ordered[0].id == "axis-lift", "Authored dependencies determine order without XYZ naming");
    for (invalid in [[home("rail", ["missing"])], [home("rail", ["lift"]), home("lift", ["rail"])]]) {
      var rejected = false;
      try motionkit.robot.HomingSequence.order(invalid) catch (_:Dynamic) rejected = true;
      check(rejected, "Homing rejects missing prerequisites and dependency cycles");
    }
    var active = new HomingFixture(); active.next(-0.001, true, true, 1, 1);
    active.cycle().start();
    check(active.moves.length == 1 && active.speeds.length == 0,
      "An already-active switch starts with a finite release plan");
    var sampled = new HomingFixture(); sampled.captured = false;
    var sampledCycle = sampled.cycle(); approach(sampled, sampledCycle);
    check(sampled.speeds[1] <= 0.00001 / 0.01 * 0.25 + 1e-12,
      "Digital-only switches retain their sampled-position precision budget");
    sampled.next(-0.002, true, true, 2, 1); sampledCycle.update(0.01);
    check(sampledCycle.status() == "StopAfterLatch", "Sampled switches still latch from a fresh observed position");
    var omitted = new HomingFixture(), omittedCycle = omitted.cycle(); approach(omitted, omittedCycle);
    omitted.omitCapture = true; omitted.next(-0.002, true, true, 2, 1);
    var rejectedCapture = false;
    try omittedCycle.update(0.01) catch (_:Dynamic) rejectedCapture = true;
    check(rejectedCapture && omittedCycle.status() == "Fault", "A captured-edge source cannot silently fall back to sampled positions");
    var missing = new HomingFixture(), missingCycle = missing.cycle(); missingCycle.start();
    missing.next(-0.01, false, false, 0, 0, true);
    var missingRejected = false;
    try missingCycle.update(0.01) catch (_:Dynamic) missingRejected = true;
    check(missingRejected && missingCycle.status() == "Fault", "A search ending without a switch faults instead of waiting forever");
    var stuck = new HomingFixture(); stuck.next(-0.001, true, true, 1, 1);
    var stuckCycle = stuck.cycle(); stuckCycle.start();
    for (_ in 0...2000) {
      if (!stuckCycle.isActive()) break;
      stuck.next(stuck.moves[stuck.moves.length - 1], true, true, 1, 1);
      try stuckCycle.update(0.01) catch (_:Dynamic) {}
    }
    check(stuckCycle.status() == "Fault" && stuck.events.indexOf("end") >= 0,
      "A switch stuck active faults at bounded release travel and closes the squaring scope");
    var changed = new HomingFixture(), changedCycle = changed.cycle(); approach(changed, changedCycle);
    changed.captured = false; changed.next(-0.001, true, false, 2, 0);
    var changedRejected = false;
    try changedCycle.update(0.01) catch (_:Dynamic) changedRejected = true;
    check(changedRejected && changedCycle.status() == "Fault", "Capture capabilities cannot change during homing");
    var delayed = new DelayedHomingFixture(), delayedCycle = delayed.cycle();
    approach(delayed, delayedCycle);
    delayed.delayHolds = true;
    delayed.next(-0.001, true, false, 2, 0); delayedCycle.update(0.01);
    delayedCycle.update(0.01);
    check(delayedCycle.status() == "Approach", "A pending control must wait without consuming the same cached capture twice");
    delayed.next(-0.0015, true, false, 2, 0); delayedCycle.update(0.01);
    check(delayed.holdPolls > 0 && delayedCycle.status() == "Approach",
      "First-side hold acknowledgment must progress while the second switch is open");
    delayed.next(-0.002, true, true, 2, 1); delayedCycle.update(0.01);
    check(delayedCycle.status() == "AwaitSideHolds" && delayed.events.indexOf("stop") < 0,
      "Slow approach must await both side hold acknowledgments before stopping");
    delayed.delayHolds = false;
    delayed.next(-0.002, true, true, 2, 1); delayedCycle.update(0.01);
    check(delayedCycle.status() == "StopAfterLatch" && delayed.events.indexOf("stop") >= 0,
      "Accepted side holds must allow the latch stop");

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
    driver.calibrationReady = false;
    driver.next(0.0, false, false, 2, 1); cycle.update(0.01);
    check(!cycle.isComplete(), "Home must wait for the return plan to drain before completion");
    driver.calibrationReady = true;
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
  public var speeds:Array<Float> = [];
  public var moves:Array<Float> = [];
  public var captured:Bool = true;
  public var omitCapture:Bool = false;
  var moving:Bool = false;
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
  public function next(position:Float, left:Bool, right:Bool, leftEdges:Int, rightEdges:Int, rest:Bool = false):Void {
    if (rest) moving = false;
    tick++; this.position = position; this.left = left; this.right = right;
    this.leftEdges = leftEdges; this.rightEdges = rightEdges;
  }
  public function observe(joint:Int):HomingObservation {
    var sequence = Int64.ofInt(tick), time = Int64.ofInt(tick * 10000000);
    return new HomingObservation(position, 0.0, [
      new HomingSwitchObservation("left", left, sequence, time, "fixture", captured && !omitCapture && leftEdges > 0 ? -0.001 : null, captured ? leftEdges : null, captured),
      new HomingSwitchObservation("right", right, sequence, time, "fixture", captured && !omitCapture && rightEdges > 0 ? -0.002 : null, captured ? rightEdges : null, captured)], time, "fixture", calibrationReady && !moving);
  }
  public function velocity(joint:Int, velocity:Float, acceleration:Float):Void {
    moving = true; speeds.push(Math.abs(velocity)); events.push("velocity");
  }
  public function moveTo(joint:Int, position:Float, velocity:Float, acceleration:Float):Void {
    moves.push(position); events.push("move");
  }
  public function stop(joint:Int, acceleration:Float):Void { moving = false; events.push("stop"); if (failStop) throw "stop failed"; }
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

class DelayedHomingFixture extends HomingFixture implements HomingControlReadiness {
  public var delayHolds:Bool = false;
  public var holdPolls:Int = 0;
  var waiting:Bool = false;
  public function new() { super(); }
  override public function hold(id:String):Void {
    super.hold(id);
    waiting = true;
  }
  public function controlsReady():Bool {
    if (waiting) {
      holdPolls++;
      if (delayHolds) return false;
      waiting = false;
    }
    return true;
  }
}

function main():Void HomingTests.run();
