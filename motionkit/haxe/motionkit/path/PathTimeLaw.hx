package motionkit.path;

import MotionKitNative;
import haxe.Int64;
import motionkit.planner.BindingConstraint;
import motionkit.planner.BindingConstraint.BindingConstraintKind;

/** One constant-acceleration span of a path-distance time law. */
class PathTimeStage {
  public final startNs:Int64;
  public final durationNs:Int64;
  public final startDistance:Float;
  public final speed:Float;
  public final acceleration:Float;

  public function new(startNs:Int64, durationNs:Int64, startDistance:Float,
      speed:Float, acceleration:Float) {
    this.startNs = startNs;
    this.durationNs = durationNs;
    this.startDistance = startDistance;
    this.speed = speed;
    this.acceleration = acceleration;
  }
}

/** Native piecewise-quadratic distance law. */
class PathTimeLaw {
  final owner:Ownedmk_time_law_handle;
  var disposed:Bool = false;

  public function new(stages:Array<PathTimeStage>, ?nativeOwner:Ownedmk_time_law_handle) {
    if (nativeOwner != null) {
      owner = nativeOwner;
      return;
    }
    if (stages == null || stages.length == 0) throw "Time law needs stages";
    var native:Array<mk_time_stage> = [];
    for (stage in stages) {
      var value = new mk_time_stage();
      value.set_struct_size(mk_time_stage.size());
      value.set_start_ns(stage.startNs);
      value.set_duration_ns(stage.durationNs);
      value.set_start_s(stage.startDistance);
      value.set_speed(stage.speed);
      value.set_acceleration(stage.acceleration);
      native.push(value);
    }
    var created = MotionKitNative.mk_time_law_create(native);
    if (created.status != MotionKitNativeConstants.MK_OK)
      throw 'timeLaw.create failed with MotionKit error ${created.status}';
    owner = created.out_law;
  }

  public function bindingConstraints():Array<BindingConstraint> {
    var result = MotionKitNative.mk_time_law_binding_count(borrow());
    if (result.status != MotionKitNativeConstants.MK_OK)
      throw 'timeLaw.bindingCount failed with MotionKit error ${result.status}';
    var bindings:Array<BindingConstraint> = [];
    for (index in 0...result.out_count) {
      var native = new mk_timing_binding();
      native.set_struct_size(mk_timing_binding.size());
      var status = MotionKitNative.mk_time_law_get_binding(borrow(), index, native);
      if (status != MotionKitNativeConstants.MK_OK)
        throw 'timeLaw.binding failed with MotionKit error $status';
      var kind = switch native.get_kind() {
        case MotionKitNativeConstants.MK_TIMING_BINDING_JOINT_VELOCITY:
          BindingConstraintKind.JointVelocity;
        case MotionKitNativeConstants.MK_TIMING_BINDING_JOINT_ACCELERATION:
          BindingConstraintKind.JointAcceleration;
        case _: BindingConstraintKind.SpeedCap;
      };
      bindings.push(new BindingConstraint(native.get_stage_index(),
        native.get_kind() == MotionKitNativeConstants.MK_TIMING_BINDING_FEED_CAP
          ? -1 : native.get_joint(), kind, native.get_limit()));
    }
    return bindings;
  }

  public function distanceToTime(distance:Float):Float {
    var result = MotionKitNative.mk_path_distance_to_time(borrow(), distance);
    if (result.status != MotionKitNativeConstants.MK_OK)
      throw 'timeLaw.distanceToTime failed with MotionKit error ${result.status}';
    return result.out_seconds;
  }

  /**
   * Path distance reached at each time (seconds from the law's epoch): the inverse of `distanceToTime`,
   * evaluated in closed form. Times outside the law clamp to its first and last distance.
   */
  public function timesToDistances(seconds:Array<Float>):Array<Float> {
    if (seconds.length == 0) return [];
    var result = MotionKitNative.mk_path_times_to_distances(borrow(), seconds);
    if (result.status != MotionKitNativeConstants.MK_OK)
      throw 'timeLaw.timesToDistances failed with MotionKit error ${result.status}';
    return result.out_distances;
  }

  public function dispose():Void {
    if (disposed) return;
    disposed = true;
    owner.close();
  }

  public function borrow():mk_time_law_handle {
    if (disposed) throw "Time law has been disposed";
    return owner.borrow();
  }
}
