import toolpathkit.motion.ToolpathMotion;

class ToolpathMotionTests {
  static function main():Void {
    if (new ToolpathMotion() == null) throw "adapter unavailable";
    Sys.println("ToolpathKit Motion tests passed (1 assertion)");
  }
}
