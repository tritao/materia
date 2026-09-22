package tests;

import haxe.Int64;
import NativeKitRuntime;
import robotkit.client.RobotClient;

/** Exercises controller leases, observer fanout, and reconnect sessions. */
class RobotSessionIntegration {
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
      var reconnect = new RobotClient("reconnected", "controller");
      try {
        stage = "reconnect connect";
        reconnect.connectWithEvents(host, port, runtime.events);
        stage = "reconnect ready";
        waitFor(reconnect, function() return reconnect.hasControlLease(),
          "reconnected controller did not receive the lease");
        if (Int64.compare(oldSession, sessionOf(reconnect)) == 0)
          throw "reconnect reused the old controller session ID";
        stage = "reconnect command";
        reconnect.sendJointTarget(0, 1, 0.25);
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
}
