package motionkit.planner;

import motionkit.path.NativeJointPath;

/** Reachability timing using the native TOPP-RA Seidel LP backend. */
class ToppraPathTiming implements PathTimingBackend {
  public final loweringTolerance:Float;

  public function new(?loweringTolerance:Float = 1e-6) {
    if (!Math.isFinite(loweringTolerance) || loweringTolerance <= 0.0)
      throw "TOPP-RA lowering tolerance must be finite and positive";
    this.loweringTolerance = loweringTolerance;
  }

  public function time(path:JointPathSamples, limits:PathTimingLimits):TimedPath {
    if (path == null || limits == null) throw "TOPP-RA needs a path and limits";
    if (limits.maxVelocity.length != path.jointCount)
      throw "TOPP-RA joint-limit count does not match the path";
    if (limits.speedCaps.length != 0 && limits.speedCaps.length != path.s.length - 1)
      throw "TOPP-RA needs one speed cap per path span";
    var nativePath = new NativeJointPath(path);
    try {
      var timed = nativePath.time(limits, loweringTolerance);
      var law = timed.law;
      try {
        var bindings = law.bindingConstraints();
        var result = new TimedPath(timed.trajectory, function(distance:Float):Float
          return law.distanceToTime(distance), bindings, function():Void law.dispose(),
          function(seconds:Array<Float>):Array<Float> return law.timesToDistances(seconds));
        nativePath.dispose();
        return result;
      } catch (error:Dynamic) {
        timed.trajectory.dispose();
        law.dispose();
        throw error;
      }
    } catch (error:Dynamic) {
      nativePath.dispose();
      throw error;
    }
  }
}
