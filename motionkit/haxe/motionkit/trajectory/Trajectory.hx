package motionkit.trajectory;

import TrajectoryCore;
import MotionKitNative;
import haxe.Int64;

/** Owns a native piecewise-polynomial joint trajectory. */
@:allow(motionkit.trajectory.ExecutionPlan)
@:allow(motionkit.path.NativeJointPath)
class Trajectory implements TimedJointTrajectory {
  final owner:Ownedmk_trajectory_handle;
  var disposed:Bool = false;
  /** Requested joint acceleration for controlled stops; independent of physical ratings. */
  public var controlAcceleration:Null<Array<Float>> = null;

  private function new(owner:Ownedmk_trajectory_handle) {
    this.owner = owner;
  }

  /** Builds a native trajectory from already-timed polynomial segments. */
  public static function fromSegments(segments:Array<{timeFromStartNs:Int64,
      durationNs:Int64, coefficients:Array<Array<Float>>}>):Trajectory {
    if (segments == null || segments.length == 0 || segments[0].coefficients == null)
      throw "Native trajectory needs segments";
    var count = segments[0].coefficients.length;
    var created = TrajectoryCore.mk_trajectory_create(count);
    check(created.status, "trajectory.create");
    var result = new Trajectory(created.out_trajectory);
    try {
      for (segment in segments) {
        if (segment.coefficients == null || segment.coefficients.length != count)
          throw "Native trajectory joint count changed";
        var degree = segment.coefficients[0].length - 1;
        if (degree < 0 || degree > TrajectoryCoreConstants.MK_MAX_DEGREE)
          throw "Native trajectory degree is out of range";
        var value = new mk_segment();
        value.set_struct_size(mk_segment.size());
        value.set_t0_ns(segment.timeFromStartNs);
        value.set_duration_ns(segment.durationNs);
        value.set_joint_count(count);
        value.set_degree(degree);
        for (joint in 0...count) {
          if (segment.coefficients[joint].length != degree + 1)
            throw "Native trajectory degree changed within a segment";
          var coefficients = new mk_joint_coefficients();
          for (index in 0...(degree + 1))
            coefficients.set_value(index, segment.coefficients[joint][index]);
          value.set_coefficients(joint, coefficients);
        }
        check(TrajectoryCore.mk_trajectory_append_segment(result.owner.borrow(), value),
          "trajectory.appendSegment");
      }
      return result;
    } catch (error:Dynamic) {
      result.dispose();
      throw error;
    }
  }

  /** Builds a degree-1 trajectory directly from authored positions. */
  public static function fromPositionSamples(times:Array<Float>, positions:Array<Array<Float>>):Trajectory {
    if (times == null || positions == null || times.length < 2 ||
        times.length != positions.length || positions[0] == null)
      throw "Native trajectory needs at least two position samples";
    var count = positions[0].length;
    if (count < 1 || count > TrajectoryCoreConstants.MK_MAX_JOINTS)
      throw "Native trajectory exceeds the joint limit";
    var samples:Array<mk_sample> = [];
    var previous = Int64.ofInt(-1);
    for (index in 0...times.length) {
      if (positions[index] == null || positions[index].length != count)
        throw "Native trajectory position count changed";
      var time = nanoseconds(times[index]);
      if (Int64.compare(time, previous) <= 0) {
        var duplicate = Int64.compare(time, previous) == 0;
        if (duplicate)
          for (joint in 0...count)
            if (positions[index][joint] != positions[index - 1][joint]) duplicate = false;
        if (duplicate) continue;
        throw "Native trajectory has conflicting or decreasing sample times after nanosecond rounding";
      }
      var value = new mk_sample();
      value.set_struct_size(mk_sample.size());
      value.set_time_ns(time);
      value.set_joint_count(count);
      for (joint in 0...count) {
        if (!Math.isFinite(positions[index][joint])) throw "Non-finite trajectory position";
        value.set_position(joint, positions[index][joint]);
      }
      samples.push(value);
      previous = time;
    }
    if (samples.length < 2) throw "Native trajectory needs two distinct sample times";
    var created = TrajectoryCore.mk_trajectory_from_samples(count, samples);
    check(created.status, "trajectory.fromSamples");
    return new Trajectory(created.out_trajectory);
  }

  /**
   * Offline, jerk-limited synchronized state-to-state motion. With `phase`,
   * every joint follows one profile scaled to its distance, so a move from
   * rest to rest is a straight line in joint space (a planned edge).
   * `targetVelocity` (default rest) lets the motion pass its target moving.
   */
  public static function generateStateToState(currentPosition:Array<Float>,
      currentVelocity:Array<Float>, currentAcceleration:Array<Float>, targetPosition:Array<Float>,
      maximumVelocity:Array<Float>, maximumAcceleration:Array<Float>,
      maximumJerk:Array<Float>, ?phase:Bool = false, ?targetVelocity:Array<Float>):Trajectory {
    if (currentPosition == null || currentPosition.length < 1 ||
        currentPosition.length > TrajectoryCoreConstants.MK_MAX_JOINTS)
      throw "Invalid generated trajectory joint count";
    var count = currentPosition.length;
    for (values in [currentVelocity, currentAcceleration, targetPosition,
        maximumVelocity, maximumAcceleration, maximumJerk])
      if (values == null || values.length != count)
        throw "Generated trajectory vector count mismatch";
    if (targetVelocity != null && targetVelocity.length != count) throw "Generated trajectory vector count mismatch";
    var request = new mk_state_to_state_request();
    request.set_struct_size(mk_state_to_state_request.size());
    request.set_joint_count(count);
    request.set_synchronization(phase == true ? MotionKitNativeConstants.MK_SYNCHRONIZATION_PHASE
      : MotionKitNativeConstants.MK_SYNCHRONIZATION_TIME);
    request.set_control_mode(MotionKitNativeConstants.MK_CONTROL_POSITION);
    for (joint in 0...count) {
      request.set_current_position(joint, currentPosition[joint]);
      request.set_current_velocity(joint, currentVelocity[joint]);
      request.set_current_acceleration(joint, currentAcceleration[joint]);
      request.set_target_position(joint, targetPosition[joint]);
      request.set_target_velocity(joint, targetVelocity == null ? 0.0 : targetVelocity[joint]);
      request.set_target_acceleration(joint, 0.0);
      request.set_max_velocity(joint, maximumVelocity[joint]);
      request.set_max_acceleration(joint, maximumAcceleration[joint]);
      request.set_max_jerk(joint, maximumJerk[joint]);
    }
    var created = MotionKitNative.mk_generate_state_to_state(request);
    check(created.status, 'trajectory.generateStateToState (Ruckig ${created.out_ruckig_result})');
    return new Trajectory(created.out_trajectory);
  }

  /** Copies polynomial coefficients for submission to an executor. */
  public function segments():Array<{timeFromStartNs:Int64, durationNs:Int64,
      coefficients:Array<Array<Float>>}> {
    ensureLive();
    var count = TrajectoryCore.mk_trajectory_segment_count(owner.borrow());
    check(count.status, "trajectory.segmentCount");
    var result = [];
    for (index in 0...count.out_segment_count) {
      var native = new mk_segment();
      native.set_struct_size(mk_segment.size());
      check(TrajectoryCore.mk_trajectory_get_segment(owner.borrow(), index, native),
        "trajectory.segment");
      var coefficients:Array<Array<Float>> = [];
      for (joint in 0...native.get_joint_count()) {
        var source = native.get_coefficients(joint);
        coefficients.push([for (degree in 0...(native.get_degree() + 1))
          source.get_value(degree)]);
      }
      result.push({timeFromStartNs: native.get_t0_ns(),
        durationNs: native.get_duration_ns(), coefficients: coefficients});
    }
    return result;
  }

  /**
    A copy whose coupled joints are exactly the sum of their terms (`follower = ratio * leader + offset`, summed
    over its terms): `followers`, `leaders`, `ratios` and `offsets` list the terms grouped by follower, each
    follower after the joints it follows. Done in native memory: a long trajectory has millions of coefficients.
  **/
  public function projectCouplings(followers:Array<Int>, leaders:Array<Int>, ratios:Array<Float>,
      offsets:Array<Float>):Trajectory {
    ensureLive();
    if (followers.length != leaders.length || followers.length != ratios.length || followers.length != offsets.length)
      throw "Coupling terms must list a follower, leader, ratio and offset each";
    var created = MotionKitNative.mk_project_couplings(owner.borrow(), followers, leaders, ratios, offsets);
    check(created.status, "trajectory.projectCouplings");
    return new Trajectory(created.out_trajectory);
  }

  /**
    A copy in which runs of segments are one quintic each where that quintic stays within the per-joint tolerances of
    what it replaces (see `mk_merge_segments`): a path lowered to hundreds of short segments a second becomes a few
    hundred, which fit a robot's queue and a device's link. Joints that follow others are to be projected again after.
  **/
  public function mergeSegments(positionTolerance:Array<Float>, velocityTolerance:Array<Float>,
      accelerationTolerance:Array<Float>, velocityLimit:Array<Float>, accelerationLimit:Array<Float>,
      positionLower:Array<Float>, positionUpper:Array<Float>, maxSeconds:Float):Trajectory {
    ensureLive();
    var count = positionTolerance.length;
    if (velocityTolerance.length != count || accelerationTolerance.length != count || velocityLimit.length != count ||
        accelerationLimit.length != count || positionLower.length != count || positionUpper.length != count)
      throw "Merging segments needs a position, velocity and acceleration tolerance and limit and position bounds for each joint";
    var created = MotionKitNative.mk_merge_segments(owner.borrow(), positionTolerance, velocityTolerance,
      accelerationTolerance, velocityLimit, accelerationLimit, positionLower, positionUpper, maxSeconds);
    check(created.status, "trajectory.mergeSegments");
    return new Trajectory(created.out_trajectory);
  }

  /** The most impulses one joint's shaper may have (`MK_MAX_SHAPER_IMPULSES`). */
  public static inline var MAX_SHAPER_IMPULSES:Int = 8;

  /**
    A copy convolved with an input shaper (see `mk_shape_trajectory`): joint `j` is the sum of its original motion
    delayed by each of `delays[j]`, weighted by `amplitudes[j]` (positive, adding up to 1). The motion is held at rest
    beyond its ends, so the copy is as much longer as the longest delay. Joints that follow others are to be projected
    again afterwards.
  **/
  public function shape(delays:Array<Array<Float>>, amplitudes:Array<Array<Float>>):Trajectory {
    ensureLive();
    var count = delays.length;
    if (amplitudes.length != count) throw "Shaping needs delays and amplitudes for each joint";
    var counts:Array<Int> = [], flatDelays:Array<Float> = [], flatAmplitudes:Array<Float> = [];
    for (joint in 0...count) {
      var impulses = delays[joint].length;
      if (impulses < 1 || impulses > MAX_SHAPER_IMPULSES || amplitudes[joint].length != impulses)
        throw "A joint's shaper needs between one and " + MAX_SHAPER_IMPULSES + " impulses, each with a delay and an amplitude";
      counts.push(impulses);
      for (slot in 0...MAX_SHAPER_IMPULSES) {
        flatDelays.push(slot < impulses ? delays[joint][slot] : 0.0);
        flatAmplitudes.push(slot < impulses ? amplitudes[joint][slot] : 0.0);
      }
    }
    var created = MotionKitNative.mk_shape_trajectory(owner.borrow(), counts, flatDelays, flatAmplitudes);
    check(created.status, "trajectory.shape");
    return new Trajectory(created.out_trajectory);
  }

  /**
    A copy in which joint `joint` runs `leadSeconds` ahead of itself, the lead taken on its velocity averaged over
    `windowSeconds`: an extruder's pressure advance. The joints that follow it are to be projected again afterwards.
  **/
  public function advanceJoint(joint:Int, leadSeconds:Float, windowSeconds:Float):Trajectory {
    ensureLive();
    var created = MotionKitNative.mk_advance_joint(owner.borrow(), joint, leadSeconds, windowSeconds);
    check(created.status, "trajectory.advanceJoint");
    return new Trajectory(created.out_trajectory);
  }

  public function evaluate(timeSeconds:Float):TrajectoryState {
    ensureLive();
    var state = TrajectoryStateBuffer.acquire();
    try {
      check(TrajectoryCore.mk_trajectory_evaluate(owner.borrow(), nanoseconds(timeSeconds), state),
        "trajectory.evaluate");
      var result = TrajectoryStateBuffer.copyOut();
      TrajectoryStateBuffer.release();
      return result;
    } catch (error:Dynamic) {
      TrajectoryStateBuffer.release();
      throw error;
    }
  }

  /** Degree-1 uses chord differences; degree 0 and degree >= 2 use analytic derivatives. */
  public function estimatePathDerivatives(timeSeconds:Float,
      windowSeconds:Float):PathDerivativeEstimate {
    ensureLive();
    var estimate = new mk_path_derivative_estimate();
    estimate.set_struct_size(mk_path_derivative_estimate.size());
    check(TrajectoryCore.mk_trajectory_estimate_path_derivatives(owner.borrow(),
      nanoseconds(timeSeconds), nanoseconds(windowSeconds), estimate),
      "trajectory.estimatePathDerivatives");
    return new PathDerivativeEstimate(estimate);
  }

  public function durationSeconds():Float {
    ensureLive();
    var result = TrajectoryCore.mk_trajectory_duration_ns(owner.borrow());
    check(result.status, "trajectory.duration");
    return Int64.toFloat(result.out_duration_ns) * 1e-9;
  }

  public function duration():Float return durationSeconds();

  public function jointCount():Int {
    ensureLive();
    var result = TrajectoryCore.mk_trajectory_joint_count(owner.borrow());
    check(result.status, "trajectory.jointCount");
    return result.out_joint_count;
  }

  public function validate(limits:ValidationLimits):ValidationReport {
    ensureLive();
    if (limits.jointCount != jointCount()) throw "Validation joint count mismatch";
    var report = new mk_validation_report();
    report.set_struct_size(mk_validation_report.size());
    check(TrajectoryCore.mk_validate(owner.borrow(), limits.nativeValue(), report),
      "trajectory.validate");
    return new ValidationReport(report);
  }

  public function dispose():Void {
    if (disposed) return;
    disposed = true;
    owner.close();
  }

  public static function nanoseconds(seconds:Float):Int64 {
    if (!Math.isFinite(seconds) || seconds < 0.0 || seconds > 2e9)
      throw "Trajectory time must be finite and non-negative";
    // Round only the fractional second as a 32-bit value. Rounding the full
    // nanosecond Float can overflow Std.int at 2.147 s and can misround a
    // decimal time such as 2.21 s by one nanosecond.
    var whole = Std.int(seconds);
    var fraction = Std.int((seconds - whole) * 1e9 + 0.5);
    if (fraction == 1000000000) {
      whole++;
      fraction = 0;
    }
    var digits = Std.string(fraction);
    while (digits.length < 9) digits = "0" + digits;
    return Int64.parseString(Std.string(whole) + digits);
  }

  static function check(status:Int, operation:String):Void {
    if (status != TrajectoryCoreConstants.MK_OK)
      throw '$operation failed with MotionKit error $status';
  }

  function ensureLive():Void {
    if (disposed) throw "Native trajectory has been disposed";
  }
}
