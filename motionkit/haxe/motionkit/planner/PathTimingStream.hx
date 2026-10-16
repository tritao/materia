package motionkit.planner;

import MotionKitNative;
import TrajectoryCore;
import motionkit.path.PathTimeLaw;
import motionkit.trajectory.Trajectory;

enum TimingStreamStep {
  Waiting;
  Ready(preview:PathTimingPreview);
  Done;
}

/** Owned candidate. Peek for certification, then commit to transfer its result.
 * Discarding an uncommitted candidate disposes both its motion and distance map.
 */
@:allow(motionkit.planner.PathTimingStream)
class PathTimingPreview {
  public final windowIndex:Int;
  public final totalWindows:Int;
  public final firstSpan:Int;
  public final spanCount:Int;
  public final firstStage:Int;
  public final startDistance:Float;
  public final endDistance:Float;
  public final startSpeed:Float;
  public final endSpeed:Float;
  final stream:PathTimingStream;
  final token:Ownedmk_path_timing_preview_handle;
  var result:Null<TimedPath>;
  var disposed = false;

  function new(stream:PathTimingStream, token:Ownedmk_path_timing_preview_handle,
      progress:mk_path_timing_stream_progress, result:TimedPath) {
    this.stream = stream;
    this.token = token;
    this.result = result;
    windowIndex = progress.get_window_index(); totalWindows = progress.get_total_windows();
    firstSpan = progress.get_first_span(); spanCount = progress.get_span_count(); firstStage = progress.get_first_stage();
    startDistance = progress.get_start_s(); endDistance = progress.get_end_s();
    startSpeed = progress.get_start_speed(); endSpeed = progress.get_end_speed();
  }

  /** Borrowed until commitTiming transfers it, or dispose releases it. */
  public function peek():TimedPath {
    if (disposed || result == null) throw "Timing preview has been released";
    return result;
  }

  /** Retire stable raw timing into unpublished transform context. This never certifies execution. */
  public function commitTiming():TimedPath {
    var owned = peek();
    stream.commit(token.borrow());
    result = null;
    disposed = true;
    token.close();
    return owned;
  }

  public function dispose():Void {
    if (disposed) return;
    disposed = true;
    token.close();
    var owned = result;
    result = null;
    if (owned != null) { owned.releaseDistanceMap(); owned.trajectory.dispose(); }
  }
}

/** One planning worker's bounded native timing queue. Returned candidates own
 * their results independently of the source path and this stream's lifetime.
 */
@:allow(motionkit.path.NativeJointPath)
@:allow(motionkit.planner.ToppraPathTiming)
@:allow(motionkit.planner.PathTimingStream.PathTimingPreview)
class PathTimingStream {
  final owner:Ownedmk_path_timing_stream_handle;
  final requireFeasible:Bool;
  var windowObserver:Null<Int->Void> = null;
  var disposed = false;
  var started = false;
  var selectedJoints:Null<Array<Int>>;

  /** Select physical output joints while retaining all timing constraints, before the first step. */
  public function selectJoints(indices:Array<Int>):PathTimingStream {
    if (disposed || started) throw "Timing output joints must be configured before stepping";
    if (indices == null || indices.length == 0) throw "Timing output selection needs indices";
    for (index in indices) if (index < 0) throw "Timing output joint index is negative";
    selectedJoints = indices.copy();
    return this;
  }

  function new(owner:Ownedmk_path_timing_stream_handle, requireFeasible:Bool = false) {
    this.owner = owner; this.requireFeasible = requireFeasible;
  }

  public function step():TimingStreamStep {
    if (disposed) throw "Path timing stream has been disposed";
    var initial = !started;
    started = true;
    var created = MotionKitNative.mk_path_timing_stream_step(owner.borrow());
    if (created.status != TrajectoryCoreConstants.MK_OK && created.out_report.get_derivative()!=0)
      throw '${requireFeasible ? "Refined process" : "Streamed path"} timing requires an adjustment: joint ${created.out_report.get_joint()}, derivative ${created.out_report.get_derivative()}, ${created.out_report.get_value()} > ${created.out_report.get_limit()}, sustainable feed scale ${created.out_report.get_sustainable_scale()}';
    if (created.status != TrajectoryCoreConstants.MK_OK)
      throw 'path.timing.stream.step failed with MotionKit error ${created.status}';
    var observer = windowObserver;
    if (initial && observer != null) observer(created.out_progress.get_total_windows());
    switch created.out_progress.get_state() {
      case MotionKitNativeConstants.MK_TIMING_STREAM_WAITING: return Waiting;
      case MotionKitNativeConstants.MK_TIMING_STREAM_DONE: return Done;
      case MotionKitNativeConstants.MK_TIMING_STREAM_READY:
        var result:Null<TimedPath> = null;
        try {
          result = ToppraPathTiming.ownResult(new PathTimeLaw([], created.out_law), new Trajectory(created.out_trajectory));
          if (selectedJoints != null) result = result.takeSelectedJoints(selectedJoints);
          return Ready(new PathTimingPreview(this, created.out_preview, created.out_progress, result));
        } catch (error:Dynamic) {
          created.out_preview.close();
          if (result != null) { result.releaseDistanceMap(); result.trajectory.dispose(); }
          throw error;
        }
      case _: throw "Native timing stream returned an unknown state";
    }
  }

  function commit(token:mk_path_timing_preview_handle):Void {
    if (disposed) throw "Path timing stream has been disposed";
    var status = MotionKitNative.mk_path_timing_stream_commit(owner.borrow(), token);
    if (status != TrajectoryCoreConstants.MK_OK)
      throw 'path.timing.stream.commit failed with MotionKit error $status';
  }

  public function dispose():Void {
    if (disposed) return;
    disposed = true;
    owner.close();
  }
}
