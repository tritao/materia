package tests;

import app.HobbingStock;
import stockkit.hobbing.HobbingAxes;
import materia.project.SceneArtifact.SceneArtifactHobbing;

/** A belt may settle before spin-up; actual reversals after spindle readiness remain an error. */
class HobbingSpindleSamplingTests {
  static var checks = 0;
  static function check(ok:Bool, label:String):Void {
    checks++;
    if (!ok) throw label;
  }
  public static function main():Int {
    run();
    return 0;
  }
  public static function run():Void {
    checks = 0;
    var config:SceneArtifactHobbing = {
      module:1, teeth:40, width:4, pressureAngle:20 * Math.PI / 180, starts:1, gashes:4,
      outside:32, length:40, wall:0.6, radialStep:0.5, axialStep:1, tolerance:0.001,
      feed:5, cuttingSpeed:300, axes:["X", "Y", "Z", "A", "B", "C"], stock:"stock", controllerHz:4000
    };
    var stock = new HobbingStock(config, [0.0, 0, 0]);
    try {
      stock.follow(0, new HobbingAxes(100, 0, 100, 0, 0, 0), false);
      stock.follow(1, new HobbingAxes(100, 0, 100, 0, -0.01, 0), false);
      check(stock.stream.peakFrames == 0, "Pre-spin belt settling does not start cutting phase windows");
      stock.follow(2, new HobbingAxes(100, 0, 100, 0, -0.02, 0), true);
      check(stock.stream.peakFrames == 1, "Readiness starts measured sampling at any initial spindle phase");
      stock.follow(3, new HobbingAxes(35, 0, 100, 0, 0.1, 0), true);
      stock.follow(4, new HobbingAxes(35, 0, 100, 0, 0.2, 0), false);
      check(stock.stream.lastTime == 4, "Sampling stays active when measured speed leaves the ready band");
      var reversed = false;
      try stock.follow(5, new HobbingAxes(35, 0, 100, 0, 0.15, 0), false) catch (_:Dynamic) reversed = true;
      check(reversed, "Actual reversal after spindle readiness still rejects the cut");
    } catch (error:Dynamic) {
      stock.dispose();
      throw error;
    }
    stock.dispose();
    Sys.println('Hobbing spindle sampling: $checks checks passed');
  }
}
