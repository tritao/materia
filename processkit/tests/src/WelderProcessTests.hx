import haxe.Int64;
import motionkit.event.EventValue;
import motionkit.kinematics.Pose3;
import motionkit.path.OrientationPolicy;
import motionkit.path.PoseLine;
import motionkit.path.PosePath;
import motionkit.path.PoseWaypoint;
import motionkit.program.Blend;
import motionkit.program.InputPredicate;
import motionkit.program.MotionOp;
import motionkit.program.MotionProgram;
import motionkit.robot.MotionSession;
import processkit.ChannelWelderOutputs;
import processkit.FeedChangePolicy;
import processkit.ProcessEngagement;
import processkit.ProcessRecipe;
import processkit.ProcessRun;
import processkit.ProcessRunState;
import processkit.WelderFeedback;
import processkit.WelderOutputs;
import processkit.WelderProcessDevice;
import processkit.tool.WeldArcModel;
import processkit.tool.WeldFault;
import processkit.tool.WeldSensor.WeldReading;
import robotkit.execution.FiredProcessEvent;
import robotkit.execution.ProcessEventValue;

/**
 * A welder behind the outputs and feedback a process device speaks: RobotKit's arc model stands in for the supply,
 * the way a retrofit I/O board or a Modbus supply would, with the tip `height` metres above the work.
 */
class ModelWelder implements WelderOutputs implements WelderFeedback {
  public final model = new WeldArcModel({maxCurrentA: 350, efficiency: 0.88, wireDiameterMm: 1.2, stickoutMm: 15});
  public var height = 0.002;
  public var arc = false;
  public var wireSpeed = 0.0;
  public var voltage = 0.0;
  public var log:Array<String> = [];

  public function new() {}

  public function setArc(on:Bool):Void {
    arc = on;
    log.push("arc " + on);
  }

  public function setWireSpeed(metresPerMinute:Float):Void {
    wireSpeed = metresPerMinute;
    log.push("wire " + metresPerMinute);
  }

  public function setVoltage(volts:Float):Void {
    voltage = volts;
    log.push("voltage " + volts);
  }

  public function reading():WeldReading return model.reading;

  /** Lets `seconds` pass over a flat work whose top is the plane z = 0. */
  public function elapse(seconds:Float):Void {
    for (i in 0...Math.round(seconds / 0.01))
      model.step(0.01, {arcCommanded: arc, wireSpeed: wireSpeed, voltageSet: voltage, supplyReady: true,
        tipDistance: height, wireDistance: height});
  }
}

/** Feedback of a supply that holds a wire-stuck fault. */
class StuckWelder implements WelderFeedback {
  public function new() {}

  public function reading():WeldReading
    return {arc: false, currentA: 0.0, voltageV: 0.0, touch: true, fault: WeldFault.WireStuck, powerW: 0.0};
}

class WelderProcessTests {
  static var assertions = 0;

  static function event(channel:String, value:ProcessEventValue):FiredProcessEvent
    return new FiredProcessEvent(Int64.ofInt(1), channel, value, Int64.ofInt(100), Int64.ofInt(100), 1);

  public static function run():Int {
    assertions = 0;
    var supply = new ModelWelder();
    var channels = {arc: "tool/torch.arc", wireSpeed: "tool/torch.wire_speed", voltage: "tool/torch.voltage"};
    var device = new WelderProcessDevice(supply, supply, channels, {voltage: 24.0});

    check(!device.ready(), "a device that was not prepared is not ready");
    check(device.fault() == null, "and has no fault");
    device.prepare();
    check(device.ready() && supply.voltage == 24.0 && supply.wireSpeed == 0.0 && !supply.arc,
      "prepare sets the voltage and leaves the arc off and the wire still");

    // Fired records work the three outputs: a process run emits the arc as an analog rate.
    device.apply([event(channels.voltage, Analog(26.0)), event(channels.wireSpeed, Analog(8.0)), event(channels.arc, Analog(1.0))]);
    check(supply.voltage == 26.0 && supply.wireSpeed == 8.0 && supply.arc, "the records set voltage, wire speed and arc");
    supply.elapse(0.3);
    check(supply.reading().arc && supply.reading().currentA > 200 && device.ready() && device.fault() == null,
      "the arc burns and the device stays ready");
    device.apply([event(channels.arc, Digital(false))]);
    check(!supply.arc, "a digital off record puts the arc out");
    device.apply([event(channels.arc, Analog(0.0))]);
    check(!supply.arc, "so does an analog rate of zero");

    // Safe: the wire stops first and the arc goes after (burnback), as a program carries the writes out.
    device.apply([event(channels.arc, Digital(true)), event(channels.wireSpeed, Analog(8.0))]);
    supply.log = [];
    device.safe();
    check(supply.log.join(",") == "wire 0,arc false" && !supply.arc && supply.wireSpeed == 0.0,
      "safe stops the wire, then switches the arc off: " + supply.log);

    // A fault from the supply is the device's fault and stops it being ready.
    var open = new ModelWelder();
    open.height = 0.05;
    var failing = new WelderProcessDevice(open, open, channels, {voltage: 24.0});
    failing.prepare();
    failing.apply([event(channels.wireSpeed, Analog(8.0)), event(channels.arc, Digital(true))]);
    open.elapse(1.2);
    check(failing.fault() == "weld: the arc did not ignite" && !failing.ready(), "no arc is the device's fault: " + failing.fault());
    failing.safe();
    open.elapse(0.05);
    check(failing.fault() == null, "the fault clears once the arc is off");
    check(!failing.ready(), "but a device made safe has to be prepared again before it is ready");
    failing.prepare();
    check(failing.ready(), "and prepare makes it ready");

    // Refusals.
    expectFailure(function() device.apply([event("other", Analog(1.0))]), "a record on another channel is refused");
    expectFailure(function() device.apply([event(channels.wireSpeed, Digital(true))]), "a wire speed needs an analog value");
    expectFailure(function() device.apply([event(channels.arc, Process("go", 1.0))]), "the arc takes no process command");
    expectFailure(function() new WelderProcessDevice(supply, supply, {arc: "a", wireSpeed: "a", voltage: "v"}, {voltage: 24.0}),
      "channels must be distinct");
    expectFailure(function() new WelderProcessDevice(supply, supply, channels, {voltage: 0.0}), "a setpoint voltage must be positive");
    runEngagement();
    runRestartPoint();
    runPrepareTimeout();
    runChannelOutputs();
    Sys.println('ProcessKit welder tests passed ($assertions assertions)');
    return assertions;
  }

  /** The motion that closes a program whose exit ends on an output change. */
  static function closing():MotionOp return MotionOp.MoveL(new Pose3(0.18, 0.0, 0.04), "arm-base", 0.08, Blend.ExactStop);

  /**
   * A restart backs up from where the metal stopped, once. A re-strike that fails, over and over, retries the same point
   * and does not move further back each time; a restart that gets some way along and then stops backs up from there.
   */
  static function runRestartPoint():Void {
    var supply = new ModelWelder();
    var channels = {arc: "tool/torch.arc", wireSpeed: "tool/torch.wire_speed", voltage: "tool/torch.voltage"};
    var device = new WelderProcessDevice(supply, supply, channels, {voltage: 24.0});
    var recipe = new ProcessRecipe(0.005, 0.03, 0.0115, 0.0, OrientationPolicy.Interpolated, 0.001, 8.0 / 0.0115, 0.0, 0.01,
      FeedChangePolicy.Reject, new ProcessEngagement([MotionOp.SetOutput(channels.arc, EventValue.Digital(true))],
        [MotionOp.SetOutput(channels.arc, EventValue.Digital(false))]), 0.08);
    var run = new ProcessRun(recipe, seamPath(), device, channels.wireSpeed, new MotionSession());
    run.start();
    run.update(0.0);
    run.takeProgram(closing());
    near(run.lastProgramStart, 0.0, "the first program starts at the path's start");
    run.interruptNow(0.09, "the arc went out");
    run.update(0.09);
    run.takeProgram(closing());
    near(run.lastProgramStart, 0.08, "the restart backs up 10 mm from where the metal stopped");
    for (attempt in 0...4) {
      // The re-strike fails at once, before the torch has moved along the seam: the distance reported is the restart point.
      run.interruptNow(0.08, "the arc did not establish");
      run.update(0.08);
      run.takeProgram(closing());
      near(run.lastProgramStart, 0.08, 'failed re-strike $attempt retries the same point');
    }
    // This one strikes, travels 5 mm and loses the arc: the metal stopped at 0.085, so the next backs up from there.
    run.interruptNow(0.085, "the arc went out");
    run.update(0.085);
    run.takeProgram(closing());
    near(run.lastProgramStart, 0.075, "a restart that got somewhere backs up from where it stopped");
  }

  /** A welder that holds a fault (a wire stuck to the work) never becomes ready: the run says so after its timeout. */
  static function runPrepareTimeout():Void {
    var supply = new ModelWelder();
    var stuck = new StuckWelder();
    var channels = {arc: "tool/torch.arc", wireSpeed: "tool/torch.wire_speed", voltage: "tool/torch.voltage"};
    var device = new WelderProcessDevice(supply, stuck, channels, {voltage: 24.0});
    var recipe = new ProcessRecipe(0.005, 0.03, 0.0115, 0.0, OrientationPolicy.Interpolated, 0.001, 8.0 / 0.0115, 0.0, 0.01,
      FeedChangePolicy.Reject, null, null, 1.0);
    var run = new ProcessRun(recipe, seamPath(), device, channels.wireSpeed, new MotionSession());
    run.start();
    run.update(0.0, 0.5);
    check(run.state == ProcessRunState.Preparation, "a welder in fault is waited for at first");
    run.update(0.0, 0.4);
    check(run.state == ProcessRunState.Preparation, "up to the timeout");
    run.update(0.0, 0.2);
    check(run.state == ProcessRunState.Failed, "and then the run fails");
    var reason = run.failure;
    check(reason != null && reason.indexOf("stuck") >= 0, "with the welder's fault as the reason: " + reason);
    expectFailure(function() run.takeProgram(), "a failed run gives no program");
    // A healthy welder is ready at once, whatever the timeout.
    var healthy = new ProcessRun(recipe, seamPath(), new WelderProcessDevice(supply, supply, channels, {voltage: 24.0}), channels.wireSpeed,
      new MotionSession());
    healthy.start();
    healthy.update(0.0, 0.1);
    check(healthy.state == ProcessRunState.Ready, "a healthy welder prepares within the timeout");
  }

  static function seamPath():PosePath {
    var start = new PoseWaypoint(new Pose3(0.0, 0.0, 0.0), 0.001, 0.001);
    var end = new PoseWaypoint(new Pose3(0.18, 0.0, 0.0), 0.001, 0.001);
    return new PosePath("arm-base", [new PoseLine(start, end, OrientationPolicy.Interpolated, 0.1, 0.0115)]);
  }

  static function kinds(program:MotionProgram):String
    return [for (op in program.ops) switch op {
      case MoveJ(_, _, _): "MoveJ";
      case MoveL(_, _, _, _, _): "MoveL";
      case MoveC(_, _, _, _, _, _): "MoveC";
      case FollowPath(_, _, _, _): "FollowPath";
      case Dwell(_): "Dwell";
      case SetOutput(_, _): "SetOutput";
      case WaitInput(_, _, _): "WaitInput";
    }].join(",");

  /**
   * A process with an engagement: its entry follows the approach move and its exit the path; the path's events do not
   * zero the process output at the end (the exit does); the run does not end at the path's end but when the caller says the
   * program is done; and a restart engages again.
   */
  static function runEngagement():Void {
    var supply = new ModelWelder();
    var channels = {arc: "tool/torch.arc", wireSpeed: "tool/torch.wire_speed", voltage: "tool/torch.voltage"};
    var device = new WelderProcessDevice(supply, supply, channels, {voltage: 24.0});
    var entry = [MotionOp.SetOutput(channels.arc, EventValue.Digital(true)),
      MotionOp.WaitInput("weld.arc_established", InputPredicate.Equals(EventValue.Digital(true)), 2.0), MotionOp.Dwell(0.15)];
    var exit = [MotionOp.Dwell(0.15), MotionOp.SetOutput(channels.wireSpeed, EventValue.Analog(0.0)),
      MotionOp.SetOutput(channels.arc, EventValue.Digital(false))];
    var recipe = new ProcessRecipe(0.005, 0.03, 0.0115, 0.0, OrientationPolicy.FreeAboutTool, 0.001, 8.0 / 0.0115, 0.0, 0.01,
      FeedChangePolicy.Reject, new ProcessEngagement(entry, exit), 0.08);
    var session = new MotionSession();
    var run = new ProcessRun(recipe, seamPath(), device, channels.wireSpeed, session);
    run.start();
    run.update(0.0);
    check(run.state == ProcessRunState.Ready, "a prepared welder is ready");
    // The exit ends on an output change, which a program may not: the caller closes it with a motion.
    expectFailure(function() run.takeProgram(), "a program may not end on an output change");
    var program = run.takeProgram(closing());
    check(kinds(program) == "MoveL,SetOutput,WaitInput,Dwell,FollowPath,Dwell,SetOutput,SetOutput,MoveL",
      "the approach, then the entry, the path and the exit: " + kinds(program));
    check(run.followOp == 4, "the path is the fifth operation");
    switch program.ops[0] {
      case MoveL(_, _, feed, _, freedom):
        near(feed, 0.08, "the approach moves at the recipe's approach speed");
        check(freedom == OrientationPolicy.FreeAboutTool, "the approach preserves the recipe's tool-axis freedom");
      case _: check(false, "the program begins with a straight move");
    }
    switch program.ops[4] {
      case FollowPath(_, _, feed, events):
        near(feed, 0.0115, "the seam is followed at the travel speed");
        check(events.length == 1, "the path sets the wire speed once and does not zero it at the end");
        switch events[0].value {
          case Analog(rate): near(rate, 8.0, "the wire speed is the rate for the travel speed");
          case _: check(false, "the wire speed is an analogue value");
        }
      case _: check(false, "the fifth operation follows the path");
    }
    run.update(0.18);
    check(run.state == ProcessRunState.Active, "at the path's end the run stays active through the exit");
    run.finish();
    check(run.state == ProcessRunState.Completion && !supply.arc && supply.wireSpeed == 0.0, "finish ends the run and makes the device safe");
    expectFailure(function() run.finish(), "a finished run cannot finish again");

    // An interruption at 0.09 m restarts 10 mm back, and the restart engages again.
    var again = new ProcessRun(recipe, seamPath(), device, channels.wireSpeed, session);
    again.start();
    again.update(0.0);
    again.takeProgram(closing());
    again.interruptNow(0.09, "the arc did not establish");
    check(again.state == ProcessRunState.ControlledInterruption && !supply.arc, "an interruption made safe");
    again.update(0.09);
    check(again.state == ProcessRunState.Recovery, "and recovers once the motion is at rest");
    var restart = again.takeProgram(closing());
    near(again.lastProgramStart, 0.08, "the restart backs up the recovery distance");
    check(kinds(restart) == kinds(program), "and engages and disengages again");
    expectFailure(function() again.interruptNow(0.5, "off the path"), "an interruption has to be on the path");

    // A process may use a different re-engagement, without repeating its initial pooling dwell.
    var gentleRecipe = new ProcessRecipe(0.005, 0.03, 0.0115, 0.0, OrientationPolicy.Interpolated, 0.001,
      8.0 / 0.0115, 0.0, 0.01, FeedChangePolicy.Reject,
      new ProcessEngagement(entry, exit, [entry[0], entry[1]]), 0.08);
    var gentle = new ProcessRun(gentleRecipe, seamPath(), device, channels.wireSpeed, session);
    gentle.start();
    gentle.update(0.0);
    check(kinds(gentle.takeProgram(closing())) == kinds(program), "the first engagement retains its dwell");
    gentle.interruptNow(0.09, "arc loss");
    gentle.update(0.09);
    var recovered = gentle.takeProgram(closing());
    check(kinds(recovered) == "MoveL,SetOutput,WaitInput,FollowPath,Dwell,SetOutput,SetOutput,MoveL",
      "recovery omits the start dwell but retains the arc wait and crater exit");
    check(gentle.followOp == 3, "recovery path progress follows the shorter engagement");

    // Without an engagement the run ends at the path's end, as it always did.
    var plain = new ProcessRun(new ProcessRecipe(0.005, 0.03, 0.0115, 0.0, OrientationPolicy.Interpolated, 0.001, 8.0 / 0.0115, 0.0,
      0.01, FeedChangePolicy.Reject), seamPath(), device, channels.wireSpeed, new MotionSession());
    plain.start();
    plain.update(0.0);
    check(kinds(plain.takeProgram()) == "MoveL,FollowPath" && plain.followOp == 1, "a plain process is a move and a path");
    plain.update(0.18);
    check(plain.state == ProcessRunState.Completion, "and ends at the path's end");
  }

  /** The channel outputs hold writes until a program carries them out. */
  static function runChannelOutputs():Void {
    var channels = {arc: "a", wireSpeed: "w", voltage: "v"};
    var outputs = new ChannelWelderOutputs(channels);
    check(outputs.drain().length == 0, "nothing is written until something is");
    outputs.setVoltage(24.0);
    outputs.setArc(false);
    outputs.setWireSpeed(0.0);
    outputs.setWireSpeed(8.0);
    var ops = outputs.drain();
    check(ops.length == 3, "each channel written is one operation, the last write winning");
    check(switch ops[0] {
      case SetOutput("v", Analog(24.0)): true;
      case _: false;
    } && switch ops[1] {
      case SetOutput("w", Analog(8.0)): true;
      case _: false;
    } && switch ops[2] {
      case SetOutput("a", Digital(false)): true;
      case _: false;
    }, "the voltage first, then the wire, then the arc");
    check(outputs.drain().length == 0, "and they are written once");
  }

  static function near(actual:Float, expected:Float, message:String):Void {
    assertions++;
    if (!(Math.abs(actual - expected) < 1e-9)) throw '$message: expected $expected, got $actual';
  }

  static function expectFailure(action:Void -> Void, message:String):Void {
    var failed = false;
    try action() catch (_:Dynamic) failed = true;
    check(failed, message);
  }

  static function check(value:Bool, message:String):Void {
    assertions++;
    if (!value) throw message;
  }
}
