package tests;

import haxe.Int64;
import nativekit.ffi.NativeKit;
import RobotKitRuntime;
import NativeKitRuntime;
import robotkit.client.RobotClient;

/** Exercises controller leases, observer fanout, and reconnect sessions. */
class RobotSessionIntegration {
  public static function runDevice(host:String, port:Int):Void {
    var runtime = NativeKitRuntime.start();
    var first = new RobotClient("device-controller", "controller");
    var second = new RobotClient("device-reconnect", "controller");
    var failure:Dynamic = null;
    try {
      first.connectWithEvents(host, port, runtime.events);
      waitFor(first, function() return first.hasControlLease() && first.latestState != null,
        "device controller did not connect");
      if (safetyOf(first) != RobotKitRuntimeConstants.RK_SAFETY_READY)
        throw "RKD6 device did not start ready";
      var syncStart = NativeKit.nk_time_now_ns();
      while (Int64.compare(Int64.sub(NativeKit.nk_time_now_ns(), syncStart),
          Int64.ofInt(1200000000)) < 0) {
        if (!first.poll()) first.wait(0.01);
      }
      first.submitPlan(new robotkit.world.ExecutionPlanSubmission(
        Int64.ofInt(901), Int64.ofInt(1), Int64.ofInt(0), 0,
        [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0],
        [new robotkit.world.TrajectorySegment(Int64.ofInt(0), Int64.ofInt(1000000000),
          [[0.0, 0.1], [0.0, 0.0], [0.0, 0.0]])], null, null,
        [0.01, 0.01, 0.01], null, null, true));
      waitFor(first, function() return first.latestState != null &&
        first.latestState.q.length > 0 && first.latestState.q[0] > 0.05,
        'RKD6 plan did not reach minimal device: safety=${safetyOf(first)} '
          + 'position=${positionOf(first)} fault=${first.lastFault}');
      first.close();
      second.connectWithEvents(host, port, runtime.events);
      waitFor(second, function() return second.hasControlLease() && second.latestState != null,
        "device reconnect did not receive control lease");
      waitFor(second, function() return safetyOf(second) ==
        RobotKitRuntimeConstants.RK_SAFETY_FAULT || safetyOf(second) ==
        RobotKitRuntimeConstants.RK_SAFETY_EMERGENCY_STOP,
        "old plan survived controller disconnect");
      Sys.println("robotd RKD6 minimal-device integration passed");
    } catch (error:Dynamic) failure = error;
    first.close();
    second.close();
    runtime.dispose();
    if (failure != null) throw failure;
  }

  public static function runLeaseTimeout(host:String, port:Int):Void {
    var runtime = NativeKitRuntime.start();
    var controller = new RobotClient("silent-controller", "controller");
    var replacement = new RobotClient("replacement-controller", "controller");
    var failure:Dynamic = null;
    try {
      controller.connectWithEvents(host, port, runtime.events);
      waitFor(controller, function() return controller.hasControlLease() &&
        controller.latestState != null, "lease test controller did not connect");
      var welcome = controller.welcome;
      if (welcome == null || welcome.leaseTimeoutMs <= 0)
        throw "robotd did not advertise a control lease timeout";
      var timeoutMs = welcome.leaseTimeoutMs;

      // Keep the normal event loop alive beyond one lease period to prove the
      // client renews while its TCP connection is otherwise idle.
      controller.stop("lease heartbeat test setup", true);
      waitFor(controller, function() return safetyOf(controller) ==
        RobotKitRuntimeConstants.RK_SAFETY_EMERGENCY_STOP,
        "lease test could not establish a stopped starting state");
      controller.resetSafety();
      waitFor(controller, function() return safetyOf(controller) ==
        RobotKitRuntimeConstants.RK_SAFETY_READY,
        "lease test controller could not reset safety");
      controller.sendJointTarget(0, 2, 1.0);
      waitFor(controller, function() return velocityOf(controller) == 1.0,
        "lease test velocity command did not reach robotd");
      var activeStart = NativeKit.nk_time_now_ns();
      var activeDuration = Int64.fromFloat(timeoutMs * 1200000.0);
      while (Int64.compare(Int64.sub(NativeKit.nk_time_now_ns(), activeStart),
          activeDuration) < 0) {
        var hadEvent = controller.poll();
        if (!hadEvent) controller.wait(0.01);
      }
      if (!controller.hasControlLease() || velocityOf(controller) != 1.0)
        throw "active RobotClient did not keep its control lease renewed";

      // A silent controller must also lose a plan that is still in flight.
      controller.stop("prepare lease plan", false);
      waitFor(controller, function() {
        var state = controller.latestState;
        return state != null &&
          state.safety == RobotKitRuntimeConstants.RK_SAFETY_READY &&
          state.sessionState == RobotKitRuntimeConstants.RK_SESSION_IDLE &&
          Math.abs(state.dq[0]) < 1e-3;
      },
        "lease test did not settle before plan submission");
      var settled = controller.latestState;
      if (settled == null) throw "lease test has no settled state";
      var anchor = settled.q.copy();
      var anchorSequence = settled.sequence;
      controller.sendJointTargets([for (joint in 0...anchor.length)
        robotkit.world.JointTarget.position(joint, anchor[joint])]);
      waitFor(controller, function() {
        var state = controller.latestState;
        return state != null &&
          Int64.compare(state.sequence, anchorSequence) > 0 &&
          state.sessionState == RobotKitRuntimeConstants.RK_SESSION_IDLE &&
          Math.abs(state.q[0] - anchor[0]) < 1e-6;
      },
        "lease test did not establish a commanded position anchor");
      // A state frame can precede the runtime's next owner cycle. Permit one
      // cycle of position drift while keeping velocity and acceleration exact.
      controller.submitPlan(new robotkit.world.ExecutionPlanSubmission(
        Int64.ofInt(801), Int64.ofInt(1), Int64.ofInt(0), 0,
        anchor, [0.0, 0.0, 0.0], [0.0, 0.0, 0.0],
        [for (index in 0...4) new robotkit.world.TrajectorySegment(
          Int64.fromFloat(index * 2000000000.0), Int64.ofInt(2000000000),
          [[anchor[0] + index * 0.02, 0.01], [anchor[1], 0.0],
            [anchor[2], 0.0]])], null, null,
        [for (_ in 0...anchor.length) 0.02], null, null, false));
      waitFor(controller, function() return controller.latestState != null &&
        Int64.compare(controller.latestState.activePlanId, Int64.ofInt(801)) == 0,
        'lease test plan did not start: fault=${controller.lastFault} id=${controller.latestState == null ? "null" : Std.string(controller.latestState.activePlanId)} safety=${controller.latestState == null ? -1 : controller.latestState.safety} q=${controller.latestState == null ? -1.0 : controller.latestState.q[0]}');

      // Do not poll, wait, or send anything during the silent interval. The
      // controller socket stays open locally while robotd's monotonic timer
      // expires the lease and applies emergency stop.
      Sys.sleep((timeoutMs + 500) / 1000.0);
      if (!controller.isConnected())
        throw "silent controller observed a close before lease timeout";
      var closeWaitStart = NativeKit.nk_time_now_ns();
      var closeWaitLimit = Int64.fromFloat(1500000000.0);
      while (controller.isConnected() && Int64.compare(Int64.sub(
          NativeKit.nk_time_now_ns(), closeWaitStart), closeWaitLimit) < 0) {
        try {
          var hadEvent = controller.poll();
          if (!hadEvent) controller.wait(0.01);
        } catch (_:Dynamic) {}
      }
      if (controller.isConnected())
        throw "robotd left the silent controller socket open after lease expiry";
      controller.close();

      replacement.connectWithEvents(host, port, runtime.events);
      waitFor(replacement, function() return replacement.hasControlLease() &&
        replacement.latestState != null,
        "new controller did not receive the expired lease");
      waitFor(replacement, function() return safetyOf(replacement) ==
        RobotKitRuntimeConstants.RK_SAFETY_EMERGENCY_STOP &&
          velocityOf(replacement) == 0.0,
        'lease expiry state mismatch: safety=${safetyOf(replacement)} '
          + 'velocity=${velocityOf(replacement)} fault=${replacement.lastFault}');
      var stoppedState = replacement.latestState;
      if (stoppedState == null) throw "new controller has no stopped state";
      var stoppedSequence = stoppedState.sequence;
      replacement.sendJointTarget(0, 2, 0.5);
      waitFor(replacement, function() return hasStateAfter(replacement, stoppedSequence) &&
        safetyOf(replacement) == RobotKitRuntimeConstants.RK_SAFETY_EMERGENCY_STOP &&
        velocityOf(replacement) == 0.0,
        "new lease owner moved the robot before resetting safety");
      replacement.resetSafety();
      waitFor(replacement, function() return safetyOf(replacement) ==
        RobotKitRuntimeConstants.RK_SAFETY_READY,
        "new lease owner could not explicitly reset the expired safety stop");
      Sys.println('robotd lease heartbeat and timeout test passed (${timeoutMs}ms)');
    } catch (error:Dynamic) failure = error;
    controller.close();
    replacement.close();
    runtime.dispose();
    if (failure != null) throw failure;
  }

  public static function runLocalOwner(host:String, port:Int):Void {
    var first = new RobotClient("denied-controller", "controller");
    var observer = new RobotClient("local-observer", "observer");
    var runtime = NativeKitRuntime.start();
    var failure:Dynamic = null;
    try {
      first.connectWithEvents(host, port, runtime.events);
      waitFor(first, function() return first.isReady(), "local-owner server did not greet client");
      if (first.hasControlLease()) throw "remote controller stole local behavior ownership";
      var rejected = false;
      try first.sendJointTarget(0, 2, 1.0) catch (_:Dynamic) rejected = true;
      if (!rejected) throw "remote command was accepted while local behavior owned control";
      observer.connectWithEvents(host, port, runtime.events);
      waitFor(observer, function() return observer.isReady(), "local observer did not connect");
      if (observer.hasControlLease()) throw "observer obtained local behavior control";
      var initial = positionOf(observer);
      waitFor(observer, function() return positionOf(observer) != initial,
        "local behavior stopped producing state after denied controller request");
      first.close();
      var afterClose = positionOf(observer);
      waitFor(observer, function() return positionOf(observer) != afterClose,
        "denied controller disconnect stopped local behavior");
      Sys.println("robotd local behavior ownership test passed");
    } catch (error:Dynamic) failure = error;
    first.close();
    observer.close();
    runtime.dispose();
    if (failure != null) throw failure;
  }

  public static function run(host:String, port:Int):Void {
    var observer = new RobotClient("observer", "observer");
    var controller = new RobotClient("controller", "controller");
    var runtime = NativeKitRuntime.start();
    var observerState = false;
    observer.stateListener = function(_) observerState = true;
    var failure:Dynamic = null;
    var stage = "start";
    try {
      /* Connect the observer first to prove it does not permanently consume
       * the controller slot. The server promotes it out of the provisional
       * slot after reading Hello. */
      stage = "observer connect";
      observer.connectWithEvents(host, port, runtime.events);
      stage = "observer ready";
      waitFor(observer, function() return observer.isReady(), "observer did not become ready");
      if (observer.hasControlLease())
        throw "observer unexpectedly received the control lease";

      stage = "controller connect";
      controller.connectWithEvents(host, port, runtime.events);
      stage = "controller ready";
      waitFor(controller, function() return controller.hasControlLease(),
        "controller did not receive the control lease");
      if (Int64.compare(sessionOf(observer), sessionOf(controller)) == 0)
        throw "observer and controller reused a session ID";

      var observerCommandRejected = false;
      try {
        observer.sendJointTarget(0, 1, 0.25);
      } catch (_:Dynamic) {
        observerCommandRejected = true;
      }
      if (!observerCommandRejected)
        throw "observer command was not rejected by the client lease boundary";
      var observerPlanRejected = false;
      try observer.submitPlan(new robotkit.world.ExecutionPlanSubmission(
        Int64.ofInt(800), Int64.ofInt(1), Int64.ofInt(0), 0,
        [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0],
        [new robotkit.world.TrajectorySegment(Int64.ofInt(0),
          Int64.ofInt(100000000), [[0.0, 0.0], [0.0, 0.0], [0.0, 0.0]])]))
      catch (_:Dynamic) observerPlanRejected = true;
      if (!observerPlanRejected)
        throw "observer plan was not rejected by the client lease boundary";

      stage = "controller command";
      controller.sendJointTarget(0, 1, 0.5);
      waitFor(controller, function() return controller.latestState != null
        && controller.latestState.q.length > 0 && controller.latestState.q[0] == 0.5,
        "controller command did not reach the runtime");
      waitFor(observer, function() return observerState,
        "observer did not receive read-only state fanout");
      controller.submitPlan(new robotkit.world.ExecutionPlanSubmission(
        Int64.ofInt(802), Int64.ofInt(1), Int64.ofInt(0), 0,
        [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0],
        [new robotkit.world.TrajectorySegment(Int64.ofInt(0),
          Int64.ofInt(100000000), [[0.0, 0.0], [0.0, 0.0], [0.0, 0.0]])]));
      waitFor(controller, function() return controller.lastFault != null,
        "rejected plan did not return a fault");
      var planFault = controller.lastFault;
      if (planFault == null || planFault.code != RobotKitRuntimeConstants.RK_ERROR_INVALID_STATE)
        throw "rejected plan lost its native error code";

      var oldSession = sessionOf(controller);
      stage = "controller disconnect";
      controller.close();
      waitFor(observer, function() return observer.latestState != null &&
        observer.latestState.safety == RobotKitRuntimeConstants.RK_SAFETY_EMERGENCY_STOP,
        "controller disconnect did not latch emergency stop");
      var reconnect = new RobotClient("reconnected", "controller");
      try {
        stage = "reconnect connect";
        reconnect.connectWithEvents(host, port, runtime.events);
        stage = "reconnect ready";
        // Welcome (with the lease) and the first State are separate messages
        // that may arrive in different reads, so wait for both before
        // checking the state the new controller inherited.
        waitFor(reconnect, function() return reconnect.hasControlLease() &&
          reconnect.latestState != null,
          "reconnected controller did not receive the lease and first state");
        if (Int64.compare(oldSession, sessionOf(reconnect)) == 0)
          throw "reconnect reused the old controller session ID";
        if (reconnect.latestState == null || reconnect.latestState.safety !=
            RobotKitRuntimeConstants.RK_SAFETY_EMERGENCY_STOP)
          throw "new controller did not inherit latched emergency stop";
        stage = "reconnect reset";
        reconnect.resetSafety();
        waitFor(reconnect, function() return reconnect.latestState != null &&
          reconnect.latestState.safety == RobotKitRuntimeConstants.RK_SAFETY_READY,
          "new controller could not explicitly reset safety");
        stage = "reconnect command";
        reconnect.sendJointTarget(0, 1, 0.25);
        waitFor(reconnect, function() return reconnect.latestState != null &&
          reconnect.latestState.q.length > 0 && reconnect.latestState.q[0] == 0.25,
          "reconnected controller command did not reach runtime");
        Sys.println('robotd session test passed: observer=${sessionOf(observer)} '
          + 'controller=${oldSession} reconnect=${sessionOf(reconnect)}');
      } catch (error:Dynamic) {
        failure = '$stage: $error';
      }
      reconnect.close();
    } catch (error:Dynamic) {
      failure = '$stage: $error';
    }
    controller.close();
    observer.close();
    runtime.dispose();
    if (failure != null) {
      Sys.println('robotd session test failure: ${Std.string(failure)}');
      throw failure;
    }
  }

  static function waitFor(client:RobotClient, condition:Void->Bool, failure:String):Void {
    var start = NativeKit.nk_time_now_ns();
    var timeout = Int64.fromFloat(5000000000.0);
    while (Int64.compare(Int64.sub(NativeKit.nk_time_now_ns(), start), timeout) < 0) {
      var hadEvent = client.poll();
      if (condition()) return;
      if (!hadEvent) client.wait(0.01);
    }
    throw failure;
  }

  static function sessionOf(client:RobotClient):Int64 {
    var welcome = client.welcome;
    if (welcome == null) throw "RobotClient has no Welcome session";
    return welcome.sessionId;
  }

  static function hasStateAfter(client:RobotClient, sequence:Int64):Bool {
    var state = client.latestState;
    return state != null && Int64.compare(state.sequence, sequence) > 0;
  }

  static function positionOf(client:RobotClient):Float {
    var state = client.latestState;
    return state == null || state.q.length == 0 ? -1000.0 : state.q[0];
  }

  static function velocityOf(client:RobotClient):Float {
    var state = client.latestState;
    return state == null || state.dq.length == 0 ? -1000.0 : state.dq[0];
  }

  static function safetyOf(client:RobotClient):Int {
    var state = client.latestState;
    return state == null ? -1 : state.safety;
  }
}
