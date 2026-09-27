package motionkit.path;

import MotionKitNative;
import haxe.Int64;

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

  public function new(stages:Array<PathTimeStage>) {
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

  public function distanceToTime(distance:Float):Float {
    var result = MotionKitNative.mk_path_distance_to_time(borrow(), distance);
    if (result.status != MotionKitNativeConstants.MK_OK)
      throw 'timeLaw.distanceToTime failed with MotionKit error ${result.status}';
    return result.out_seconds;
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
