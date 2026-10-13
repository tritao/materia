package motionkit.planner;

import motionkit.path.NativeJointPath;

/** Reachability timing using the native TOPP-RA Seidel LP backend. */
class ToppraPathTiming implements PathTimingBackend {
  public final loweringTolerance:Float;
  /**
    Paths timed in more than one window, and how many times their windows' boundary speeds had to be lowered before
    they could be timed (each lowering costs speed; three are a rest at every boundary). Diagnostics of this instance's
    plans, for a caller that times on one thread.
  **/
  public var windowedPaths(default, null):Int = 0;
  public var boundaryLowerings(default, null):Int = 0;

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
      if (timed.windows > 1) windowedPaths++;
      boundaryLowerings += timed.boundaryLowerings;
      var law = timed.law;
      try {
        var result = new TimedPath(timed.trajectory, function(distance:Float):Float
          return law.distanceToTime(distance), [], function():Void law.dispose(),
          function(seconds:Array<Float>):Array<Float> return law.timesToDistances(seconds),
          function():Array<motionkit.planner.BindingConstraint> return law.bindingConstraints());
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
