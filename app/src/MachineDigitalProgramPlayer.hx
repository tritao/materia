package app;

import haxe.Int64;
import materia.project.SceneArtifact.SceneArtifactDigitalProgram;
import RobotKitRuntime;
import robotkit.core.RobotCommand;
import robotkit.core.StopMode;
import robotkit.execution.ExecutionPlanSubmission;
import robotkit.execution.ProcessEventValue;
import robotkit.execution.ProcessTimedEvent;
import robotkit.execution.TrajectorySegment;

/** Executes timed machine outputs while the runtime owns passive and process-driven joints. */
class MachineDigitalProgramPlayer implements SessionMember {
  final robot:AssemblyRobot;
  final authored:SceneArtifactDigitalProgram;
  final events:Array<ProcessTimedEvent>;
  final duration:Int64;
  var planId:Int64 = Int64.ofInt(1);
  var started:Bool = false;
  var finished:Bool = false;
  var fired:Int = 0;
  public var passes(default, null):Int = 0;
  public var failure(default, null):Null<String> = null;

  public function new(authored:SceneArtifactDigitalProgram, robot:AssemblyRobot) {
    this.authored = authored;
    this.robot = robot;
    duration = nanoseconds(authored.durationSec);
    if (Int64.compare(duration, Int64.ofInt(0)) <= 0 || authored.events == null || authored.events.length == 0)
      throw "Machine digital program needs a positive duration and output events";
    events = [];
    var last = new Map<String, Bool>();
    var channelTimes = new Map<String, Int64>();
    var previous = Int64.ofInt(0);
    for (event in authored.events) {
      var time = nanoseconds(event.timeSec);
      if (Int64.compare(time, previous) < 0 || Int64.compare(time, duration) >= 0)
        throw "Machine output events must be ordered before the program end";
      var lastTime = channelTimes.get(event.channel);
      if (lastTime != null && Int64.compare(lastTime, time) == 0)
        throw 'Machine output "${event.channel}" is assigned twice at one runtime time';
      channelTimes.set(event.channel, time);
      var declarations = [for (channel in robot.blueprint.channels) if (channel.id == event.channel) channel];
      if (declarations.length != 1) throw 'Machine output channel "${event.channel}" is missing';
      if (declarations[0].keepOnStop) throw 'Machine output channel "${event.channel}" must become safe on stop';
      switch declarations[0].safeValue {
        case Digital(_):
        case _: throw 'Machine output channel "${event.channel}" is not digital';
      }
      events.push(new ProcessTimedEvent(time, event.channel, Digital(event.value)));
      last.set(event.channel, event.value);
      previous = time;
    }
    for (channel in robot.blueprint.channels) if (last.exists(channel.id)) {
      switch channel.safeValue {
        case Digital(value):
          if (last.get(channel.id) != value) throw 'Machine output "${channel.id}" does not end at its safe value';
        case _:
      }
    }
  }

  static function nanoseconds(seconds:Float):Int64 {
    var value = seconds * 1e9;
    if (!Math.isFinite(value) || value < 0 || value > 9007199254740991.0)
      throw "Machine output time cannot be represented in nanoseconds";
    // Conversion truncates positive values; keep rounding in Float to avoid a 32-bit intermediate.
    return Int64.fromFloat(value + 0.5);
  }

  public function feed():Void {
    if (failure != null || finished) return;
    var snapshot = robot.runtime.snapshot();
    if (snapshot.faultCode != 0 || snapshot.safety == RobotKitRuntimeConstants.RK_SAFETY_FAULT ||
        snapshot.safety == RobotKitRuntimeConstants.RK_SAFETY_EMERGENCY_STOP) {
      fail('Machine output runtime fault ${snapshot.faultCode}, safety ${snapshot.safety}');
      return;
    }
    if (!started) {
      var anchor = [for (joint in 0...snapshot.setpoint.length) snapshot.setpoint.get(joint)];
      var zero = [for (_ in anchor) 0.0];
      var segment = new TrajectorySegment(Int64.ofInt(0), duration, [for (position in anchor) [position]]);
      try {
        var plan = new ExecutionPlanSubmission(planId, snapshot.modelRevision,
          snapshot.calibrationRevision, RobotKitRuntimeConstants.RK_PLAN_CAPABILITY_TRAJECTORY_QUEUE |
          RobotKitRuntimeConstants.RK_PLAN_CAPABILITY_EVENTS, anchor, zero, zero, [segment],
          null, null, null, null, null, true, events);
        robot.robot.submit(RobotCommand.ExecutionPlan(plan));
      } catch (error:Dynamic) {
        fail('Machine output plan submission failed: ${Std.string(error)}');
        return;
      }
      started = true;
      return;
    }
    var applied = robot.runtime.pollEvents();
    if (applied.overflow) { fail("Machine output event history overflowed"); return; }
    for (event in applied.events) if (Int64.compare(event.planId, planId) == 0) {
      if (fired >= events.length) { fail("Machine output runtime reported an unexpected event"); return; }
      var expected = events[fired];
      var agrees = switch [expected.value, event.value] {
        case [Digital(a), Digital(b)]: a == b;
        case _: false;
      };
      if (event.channel != expected.channel || Int64.compare(event.scheduledTimeNs, expected.timeNs) != 0 || !agrees) {
        fail('Machine output event $fired disagrees with its submitted plan');
        return;
      }
      fired++;
    }
    var complete = !snapshot.trajectoryActive && snapshot.trajectoryQueueDepth == 0 &&
      snapshot.sessionState == RobotKitRuntimeConstants.RK_SESSION_IDLE;
    // The native runtime clears activePlanId when its queue completes.
    if (Int64.compare(snapshot.activePlanId, planId) != 0 &&
        !(complete && Int64.compare(snapshot.activePlanId, Int64.ofInt(0)) == 0)) {
      fail("Machine output runtime changed plan ownership");
      return;
    }
    if (complete) {
      if (fired != events.length) { fail("Machine output plan ended without firing all output events"); return; }
      passes++;
      if (authored.loop) {
        started = false;
        fired = 0;
        planId = Int64.add(planId, Int64.ofInt(1));
      } else finished = true;
    }
  }

  function fail(message:String):Void {
    failure = message;
    robot.robot.stop(StopMode.Emergency);
  }

  public function beforeReset():Void { if (started && !finished) robot.robot.stop(StopMode.Emergency); }
  public function reset():Void {
    planId = Int64.ofInt(1); started = false; finished = false;
    fired = 0; passes = 0; failure = null;
  }
  public function present():Void {}
}
