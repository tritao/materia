package motionkit.axis;

import robotkit.model.RobotModel;
import robotkit.runtime.RobotRuntimeBlueprint;
import robotkit.runtime.RobotRuntimeCompiler;

/** Compiled RobotKit model plus the semantic logical-axis view. */
class MotionSystemBlueprint {
  public final model:RobotModel;
  public final runtime:RobotRuntimeBlueprint;
  public final axes:Array<MotionAxisBlueprint>;
  public final fixedTimestepSeconds:Float;

  public function new(model:RobotModel, runtime:RobotRuntimeBlueprint,
      axes:Array<MotionAxisBlueprint>, ?fixedTimestepSeconds:Float = 0.01) {
    if (model == null || runtime == null) throw "Motion system needs a robot model and runtime blueprint";
    if (axes == null || axes.length == 0) throw "Motion system needs at least one logical axis";
    if (!Math.isFinite(fixedTimestepSeconds) || fixedTimestepSeconds <= 0.0)
      throw "Motion system timestep must be finite and positive";
    var ids = new Map<String, Bool>();
    for (axis in axes) {
      if (axis == null) throw "Motion system cannot contain a null axis";
      if (ids.exists(axis.id)) throw 'Motion system contains duplicate axis "${axis.id}"';
      ids.set(axis.id, true);
    }
    this.model = model;
    this.runtime = runtime;
    this.axes = axes.copy();
    this.fixedTimestepSeconds = fixedTimestepSeconds;
  }

  public static function fromRobotModel(model:RobotModel, axes:Array<MotionAxisBlueprint>,
      ?revision:Int = 1, ?fixedTimestepSeconds:Float = 0.01):MotionSystemBlueprint {
    return new MotionSystemBlueprint(model, RobotRuntimeCompiler.compile(model, revision),
      axes, fixedTimestepSeconds);
  }
}
