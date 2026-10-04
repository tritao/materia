import haxe.Int64;
import motionkit.event.EventValue;
import motionkit.path.OrientationPolicy;
import motionkit.path.PoseLine;
import motionkit.path.PosePath;
import motionkit.path.PoseWaypoint;
import motionkit.kinematics.Pose3;
import motionkit.program.MotionOp;
import motionkit.robot.MotionSession;
import processkit.ChannelProcessDevice;
import processkit.FeedChangePolicy;
import processkit.ProcessRecipe;
import processkit.ProcessRun;
import processkit.ProcessRunState;
import robotkit.tool.ChannelToolAdapter;
import robotkit.tool.SimulatedSprayer;
import robotkit.execution.FiredProcessEvent;
import robotkit.execution.ProcessEventValue;

class ProcessKitTests {
  static var assertions = 0;

  public static function main():Void {
    paintFaultRecovery();
    dispenseAdapt();
    rejectOverride();
    pauseOverride();
    WelderProcessTests.run();
    WeldPlanningTests.run();
    ProcessRateScheduleTests.run();
    WeldWeaveTests.run();
    WeldPassesTests.run();
    Sys.println('ProcessKit tests passed ($assertions assertions)');
  }

  static function paintFaultRecovery():Void {
    var sprayer = new SimulatedSprayer();
    var adapter = new ChannelToolAdapter();
    adapter.bindSprayerFlow("paint.flow", sprayer, 1.0);
    var clock = Int64.ofInt(100);
    var device = new ChannelProcessDevice(adapter, "paint.flow", function() return clock);
    var recipe = new ProcessRecipe(0.05, 0.2, 0.1, 0.03,
      OrientationPolicy.Interpolated, 0.05, 2.0, 0.02, 0.05,
      FeedChangePolicy.Adapt);
    var session = new MotionSession();
    var run = new ProcessRun(recipe, straightPath(), device, "paint.flow", session);
    run.start();
    run.update(0.0);
    check(run.state == ProcessRunState.Ready, "Paint waits for a ready device");
    var first = run.takeProgram();
    check(first.ops.length == 2, "Paint compiles approach and process operations");
    check(firstRate(first) == 0.2, "Paint flow matches quantity per distance");
    run.applyRecords([new FiredProcessEvent(Int64.ofInt(1), "paint.flow",
      ProcessEventValue.Analog(0.2), Int64.ofInt(100), Int64.ofInt(100), 1)]);
    check(sprayer.flow() == 0.2, "Channel adapter applies the scheduled paint flow");
    device.setFault("nozzle blocked");
    run.update(0.6);
    check(run.state == ProcessRunState.ControlledInterruption && sprayer.flow() == 0.0,
      "Fault makes the tool safe and interrupts the process");
    device.setFault(null);
    session.begin();
    run.update(0.6);
    check(run.state == ProcessRunState.ControlledInterruption,
      "Process recovery waits while the arm is running");
    session.hold();
    run.update(0.6);
    check(run.state == ProcessRunState.ControlledInterruption,
      "Process recovery waits while the arm is slowing to a hold");
    session.rest();
    run.update(0.6);
    check(run.state == ProcessRunState.Recovery, "Cleared fault enters recovery");
    session.stop(motionkit.robot.StopDisposition.Discard, []);
    var blockedWhileStopping = false;
    try run.takeProgram() catch (_:Dynamic) blockedWhileStopping = true;
    check(blockedWhileStopping,
      "Process recovery cannot start while motion is stopping");
    session.rest();
    session.reject();
    var blockedWhileFaulted = false;
    try run.takeProgram() catch (_:Dynamic) blockedWhileFaulted = true;
    check(blockedWhileFaulted,
      "Process recovery cannot start from a faulted session");
    session.reset();
    var resumed = run.takeProgram();
    check(Math.abs(run.lastProgramStart - 0.55) < 1e-9,
      "Recovery backs off from the interrupted path distance");
    check(firstX(resumed) >= 0.55 - 1e-9 && firstX(resumed) <= 0.55 + 1e-9,
      "Recovery reapproaches the backed-off path pose");
    var overlap = 0;
    for (cell in 0...1000) {
      var x = (cell + 0.5) / 1000.0;
      if (x <= 0.6 && x >= run.lastProgramStart) overlap++;
    }
    check(Math.abs(overlap / 1000.0 - 0.05) <= 0.001,
      "Recovery overlaps only the configured backoff span");
    run.update(1.0);
    check(run.state == ProcessRunState.Completion,
      "Recovered paint pass completes");
    check(run.transitions.length == 7,
      "Preparation, ready, active, interruption, recovery, active and completion are logged");
  }

  static function dispenseAdapt():Void {
    var device = simulatedDevice("dispense.rate");
    var recipe = new ProcessRecipe(0.05, 0.2, 0.1, 0.01,
      OrientationPolicy.Interpolated, 0.02, 0.4, 0.0, 0.03,
      FeedChangePolicy.Adapt);
    var run = new ProcessRun(recipe, straightPath(), device, "dispense.rate",
      new MotionSession());
    run.start(); run.update(0.0);
    var original = run.takeProgram();
    check(Math.abs(firstRate(original) / 0.1 - 0.4) < 0.008,
      "Nominal dispense quantity per distance is within 2%");
    run.requestFeed(0.15, 0.4);
    check(run.state == ProcessRunState.ControlledInterruption,
      "Adapt safely interrupts before changing feed");
    run.update(0.4);
    var adapted = run.takeProgram();
    check(Math.abs(run.lastProgramStart - 0.4) < 1e-9,
      "Feed adaptation resumes at the current distance");
    check(Math.abs(firstRate(adapted) / 0.15 - 0.4) < 0.008,
      "Adapt keeps dispense quantity per distance within 2%");
  }

  static function rejectOverride():Void {
    var recipe = new ProcessRecipe(0.05, 0.2, 0.1, 0.01,
      OrientationPolicy.Interpolated, 0.02, 0.4, 0.0, 0.03,
      FeedChangePolicy.Reject);
    var run = new ProcessRun(recipe, straightPath(),
      simulatedDevice("reject.rate"), "reject.rate", new MotionSession());
    run.start(); run.update(0.0); run.takeProgram();
    var rejected = false;
    try run.requestFeed(0.15, 0.2) catch (_:Dynamic) rejected = true;
    check(rejected && run.state == ProcessRunState.Active,
      "Reject refuses a feed override without changing the active run");
  }

  static function pauseOverride():Void {
    var recipe = new ProcessRecipe(0.05, 0.2, 0.1, 0.01,
      OrientationPolicy.Interpolated, 0.02, 0.4, 0.0, 0.03,
      FeedChangePolicy.Pause);
    var run = new ProcessRun(recipe, straightPath(),
      simulatedDevice("pause.rate"), "pause.rate", new MotionSession());
    run.start(); run.update(0.0);
    run.requestFeed(0.15, 0.0);
    var blocked = false;
    try run.takeProgram() catch (_:Dynamic) blocked = true;
    check(blocked && run.state == ProcessRunState.Ready,
      "Pause prevents a program from starting at an override feed");
    run.requestFeed(0.1, 0.0);
    run.takeProgram();
    run.requestFeed(0.15, 0.2);
    run.update(0.2);
    check(run.state == ProcessRunState.ControlledInterruption,
      "Pause holds the process while feed differs from nominal");
    run.requestFeed(0.1, 0.2);
    run.update(0.2);
    check(run.state == ProcessRunState.Recovery,
      "Pause recovers when nominal feed returns");
    run.takeProgram();
  }

  static function straightPath():PosePath {
    var start = new PoseWaypoint(new Pose3(0.0, 0.0, 0.0), 0.001, 0.001);
    var end = new PoseWaypoint(new Pose3(1.0, 0.0, 0.0), 0.001, 0.001);
    return new PosePath("work", [new PoseLine(start, end,
      OrientationPolicy.Interpolated, 0.1, 0.1)]);
  }

  static function simulatedDevice(channel:String):ChannelProcessDevice {
    var adapter = new ChannelToolAdapter();
    adapter.bind(channel, function(_) {});
    return new ChannelProcessDevice(adapter, channel,
      function() return Int64.ofInt(0));
  }

  static function firstRate(program:motionkit.program.MotionProgram):Float {
    for (op in program.ops) switch op {
      case MotionOp.FollowPath(_, _, _, events):
        return switch events[0].value {
          case EventValue.Analog(rate): rate;
          case _: -1.0;
        };
      case _:
    }
    return -1.0;
  }

  static function firstX(program:motionkit.program.MotionProgram):Float {
    for (op in program.ops) switch op {
      case MotionOp.MoveL(pose, _, _, _): return pose.x;
      case _:
    }
    return -1.0;
  }

  static function check(value:Bool, message:String):Void {
    assertions++;
    if (!value) throw 'assertion failed: $message';
  }
}
