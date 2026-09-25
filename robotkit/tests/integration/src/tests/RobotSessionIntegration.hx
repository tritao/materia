package tests;

import haxe.Int64;
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
      if (safetyOf(first) != RobotKitRuntimeConstants.RK_SAFETY_EMERGENCY_STOP)
        throw "device did not start latched safe";
      first.resetSafety();
      waitFor(first, function() return first.latestState != null &&
        first.latestState.safety == RobotKitRuntimeConstants.RK_SAFETY_READY,
        "device did not accept safety reset");
      first.sendJointTarget(0, 2, 1.0);
      waitFor(first, function() return first.latestState != null &&
        first.latestState.dq.length > 0 && first.latestState.dq[0] == 1.0,
        'velocity target did not reach Rust device: safety=${safetyOf(first)} '
          + 'velocity=${velocityOf(first)} fault=${first.lastFault}');
      first.close();
      second.connectWithEvents(host, port, runtime.events);
      waitFor(second, function() return second.hasControlLease() && second.latestState != null,
        "device reconnect did not receive control lease");
      if (safetyOf(second) != RobotKitRuntimeConstants.RK_SAFETY_EMERGENCY_STOP ||
          velocityOf(second) != 0.0)
        throw "old velocity intent survived disconnect";
      second.resetSafety();
      waitFor(second, function() return second.latestState != null &&
        second.latestState.safety == RobotKitRuntimeConstants.RK_SAFETY_READY,
        "reconnected controller could not reset safety");
      second.sendJointTarget(0, 1, 0.25);
      waitFor(second, function() return second.latestState != null &&
        second.latestState.q.length > 0 && second.latestState.q[0] == 0.25,
        "reconnected position command did not reach Rust device");
      Sys.println("robotd RKD5 Rust-device integration passed");
    } catch (error:Dynamic) failure = error;
    first.close();
    second.close();
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

      stage = "controller command";
      controller.sendJointTarget(0, 1, 0.5);
      waitFor(controller, function() return controller.latestState != null
        && controller.latestState.q.length > 0 && controller.latestState.q[0] == 0.5,
        "controller command did not reach the runtime");
      waitFor(observer, function() return observerState,
        "observer did not receive read-only state fanout");

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
        waitFor(reconnect, function() return reconnect.hasControlLease(),
          "reconnected controller did not receive the lease");
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
    for (_ in 0...500) {
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
