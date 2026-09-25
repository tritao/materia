package tests;

class IntegrationMain {
  public static function main():Void {
    var arguments = Sys.args();
    var port = parsePort(arguments);
    var host = parseHost(arguments);
    if (arguments.indexOf("--smoke") >= 0) RobotClientSmoke.run(host, port);
    else if (arguments.indexOf("--lease-timeout") >= 0)
      RobotSessionIntegration.runLeaseTimeout(host, port);
    else if (arguments.indexOf("--device") >= 0)
      RobotSessionIntegration.runDevice(host, port);
    else if (arguments.indexOf("--local-owner") >= 0)
      RobotSessionIntegration.runLocalOwner(host, port);
    else if (arguments.indexOf("--sessions") >= 0) RobotSessionIntegration.run(host, port);
    else if (arguments.indexOf("--restart-check") >= 0)
      WorldTcpIntegration.runRestartCheck(host, port);
    else WorldTcpIntegration.run(host, port);
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
