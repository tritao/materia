package robotd;

class Main {
  public static function main():Void {
    var host = new RobotHost(Sys.args());
    host.run();
  }
}
