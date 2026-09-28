import toolpathkit.ToolpathKit;

class ToolpathKitTests {
  static function main():Void {
    var kit = new ToolpathKit();
    if (kit == null) throw "ToolpathKit failed to initialize";
    Sys.println("ToolpathKit: 0 assertions");
  }
}
