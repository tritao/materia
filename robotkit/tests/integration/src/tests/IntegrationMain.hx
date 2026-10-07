package tests;

class IntegrationMain {
  public static function main():Void {
    var arguments = Sys.args();
    // Workspace tests invoke the entry without arguments. The TCP client
    // needs the managed robotd fixture; world-tcp.sh calls back with --port
    // and --camera-fixture so that invocation runs the client directly.
    if (arguments.length == 0) {
      runFixture();
      return;
    }
    var port = parsePort(arguments);
    var host = parseHost(arguments);
    if (arguments.indexOf("--outbound-scheduler") >= 0) OutboundSchedulerIntegration.run(port);
    else if (arguments.indexOf("--perception") >= 0) PerceptionTcpIntegration.run(host, port);
    else if (arguments.indexOf("--subscriptions") >= 0) SubscriptionIntegration.run(host, port);
    else if (arguments.indexOf("--smoke") >= 0) RobotClientSmoke.run(host, port);
    else if (arguments.indexOf("--lease-timeout") >= 0)
      RobotSessionIntegration.runLeaseTimeout(host, port);
    else if (arguments.indexOf("--device") >= 0)
      RobotSessionIntegration.runDevice(host, port);
    else if (arguments.indexOf("--local-owner") >= 0)
      RobotSessionIntegration.runLocalOwner(host, port);
    else if (arguments.indexOf("--sessions") >= 0) RobotSessionIntegration.run(host, port);
    else if (arguments.indexOf("--restart-check") >= 0)
      WorldTcpIntegration.runRestartCheck(host, port);
    else WorldTcpIntegration.run(host, port, arguments.indexOf("--camera-fixture") >= 0);
  }

  static function runFixture():Void {
    var directory = sys.FileSystem.fullPath(Sys.getCwd());
    while (true) {
      var script = haxe.io.Path.join([directory, "robotkit", "tests", "world-tcp.sh"]);
      if (sys.FileSystem.exists(script)) {
        Sys.println("RobotKit TCP integration: starting the managed robotd fixture");
        var status = Sys.command("bash", [script]);
        if (status != 0) Sys.exit(status);
        return;
      }
      var parent = haxe.io.Path.directory(directory);
      if (parent == directory || parent.length == 0)
        throw "Cannot locate robotkit/tests/world-tcp.sh; run the integration suite inside the repository";
      directory = parent;
    }
  }

  static function parseHost(arguments:Array<String>):String {
    for (argument in arguments) {
      if (argument.indexOf("--host=") == 0) {
        var value = argument.substr(7);
        if (value.length == 0) throw "RobotKit integration test requires a nonempty --host";
        return value;
      }
    }
    return "127.0.0.1";
  }

  static function parsePort(arguments:Array<String>):Int {
    for (argument in arguments) {
      if (argument.indexOf("--port=") == 0) {
        var value = Std.parseInt(argument.substr(7));
        if (value == null || value <= 0 || value > 65535) throw "RobotKit integration test requires a valid --port";
        return value;
      }
    }
    return 17890;
  }
}
