package motionkit.planner;

import motionkit.path.NativeJointPath;
import motionkit.path.PathTimeLaw;
import motionkit.trajectory.Trajectory;

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

  public function prepareStream(path:JointPathSamples, limits:PathTimingLimits,
      maxPendingWindows:Int):PathTimingStream {
    validate(path, limits);
    var nativePath = new NativeJointPath(path);
    try {
      var stream = nativePath.prepareTimingStream(limits, loweringTolerance, maxPendingWindows);
      stream.windowObserver = windows -> { if (windows > 1) windowedPaths++; };
      nativePath.dispose();
      return stream;
    } catch (error:Dynamic) { nativePath.dispose(); throw error; }
  }

  @:allow(motionkit.planner.PathTimingStream)
  static function ownResult(law:PathTimeLaw, trajectory:Trajectory):TimedPath {
    try return new TimedPath(trajectory, distance -> law.distanceToTime(distance), [],
      () -> law.dispose(), seconds -> law.timesToDistances(seconds), () -> law.bindingConstraints());
    catch (error:Dynamic) { trajectory.dispose(); law.dispose(); throw error; }
  }

  static function validate(path:JointPathSamples, limits:PathTimingLimits):Void {
    if (path == null || limits == null) throw "TOPP-RA needs a path and limits";
    if (limits.maxVelocity.length != path.jointCount)
      throw "TOPP-RA joint-limit count does not match the path";
    if (limits.speedCaps.length != 0 && limits.speedCaps.length != path.s.length - 1)
      throw "TOPP-RA needs one speed cap per path span";
  }

  /** Whole-path timing is the same stream drained to completion, retaining its exact distance laws. */
  public function time(path:JointPathSamples, limits:PathTimingLimits):TimedPath {
    var stream = prepareStream(path,limits,path.s.length-1);
    var series = new TimedPathSeries();
    try {
      var done = false;
      while (!done) switch stream.step() {
        case Waiting:
        case Done: done = true;
        case Ready(preview):
          var timed = preview.commitTiming();
          try series.append(timed,preview.startDistance,preview.endDistance,preview.firstStage);
          catch (error:Dynamic) { timed.releaseDistanceMap();timed.trajectory.dispose();throw error; }
      }
      stream.dispose();
      return series.finish();
    } catch (error:Dynamic) { stream.dispose();series.dispose();throw error; }
  }
}
