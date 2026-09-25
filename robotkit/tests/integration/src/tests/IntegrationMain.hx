package tests;

class IntegrationMain {
  public static function main():Void {
    var arguments = Sys.args();
    var port = parsePort(arguments);
    if (arguments.indexOf("--smoke") >= 0) RobotClientSmoke.run("127.0.0.1", port);
    else if (arguments.indexOf("--device") >= 0)
      RobotSessionIntegration.runDevice("127.0.0.1", port);
    else if (arguments.indexOf("--local-owner") >= 0)
      RobotSessionIntegration.runLocalOwner("127.0.0.1", port);
    else if (arguments.indexOf("--sessions") >= 0) RobotSessionIntegration.run("127.0.0.1", port);
    else if (arguments.indexOf("--restart-check") >= 0)
      WorldTcpIntegration.runRestartCheck("127.0.0.1", port);
    else WorldTcpIntegration.run("127.0.0.1", port);
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
