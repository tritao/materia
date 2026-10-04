package motionkit.robot;

import haxe.Int64;
import motionkit.trajectory.ValidationLimits;
import robotkit.manipulation.KinematicGroup;
import robotkit.model.JointLimits;
import robotkit.model.RobotModel;
import robotkit.model.SteadyLoads;
import motionkit.robot.PlanCheck.PlanCheckOptions;

/** One snapshot of drive-derived planning limits, in the planner's joint order. */
class PlanningLimits {
  public final jointIds:Array<String>;
  public final bounds:Array<JointLimits>;
  public final velocity:Array<Float>;
  public final acceleration:Array<Float>;
  public final jerk:Array<Float>;
  /** Defaults used where the model does not supply a quantity; never hardware ratings. */
  public final assumptions:Array<String>;
  public final modelRevision:Int64;
  public final calibrationRevision:Int64;
  final model:RobotModel;
  final steady:SteadyLoads;

  public static function of(model:RobotModel, jointIds:Array<String>, ?steady:SteadyLoads,
      ?defaultAcceleration:Float = 2.0, ?jointJerk:Array<Float>,
      ?modelRevision:Int64, ?calibrationRevision:Int64):PlanningLimits {
    return new PlanningLimits(model, jointIds, steady == null ? new SteadyLoads() : steady,
      defaultAcceleration, jointJerk, modelRevision, calibrationRevision);
  }

  /** The same helper over a group's driving joints, preserving its solver order. */
  public static function ofGroup(group:KinematicGroup, ?steady:SteadyLoads,
      ?defaultAcceleration:Float = 2.0, ?jointJerk:Array<Float>,
      ?modelRevision:Int64, ?calibrationRevision:Int64):PlanningLimits {
    if (group == null) throw "Planning limits need a kinematic group";
    return of(group.robot, [for (id in group.jointIds()) Std.string(id)], steady,
      defaultAcceleration, jointJerk, modelRevision, calibrationRevision);
  }

  function new(model:RobotModel, jointIds:Array<String>, steady:SteadyLoads,
      defaultAcceleration:Float, jointJerk:Null<Array<Float>>,
      modelRevision:Null<Int64>, calibrationRevision:Null<Int64>) {
    if (model == null || jointIds == null || jointIds.length == 0)
      throw "Planning limits need a model and joints in planner order";
    positive(defaultAcceleration, "default acceleration");
    if (jointJerk != null && jointJerk.length != jointIds.length)
      throw "Planning jerk limits need one value per joint";
    this.model = model;
    this.modelRevision = modelRevision == null ? Int64.ofInt(1) : modelRevision;
    this.calibrationRevision = calibrationRevision == null ? Int64.ofInt(0) : calibrationRevision;
    this.jointIds = jointIds.copy();
    this.steady = steady.copy();
    bounds = []; velocity = []; acceleration = []; jerk = []; assumptions = [];
    var seen = new Map<String, Bool>();
    for (index in 0...jointIds.length) {
      var id = jointIds[index];
      if (seen.exists(id)) throw 'Planning joint "$id" is repeated';
      seen.set(id, true);
      var bound = model.coupledLimits(id, this.steady);
      var speed = bound.velocity;
      if (speed == null) throw 'Planning joint "$id" has no velocity limit';
      positive(speed, 'joint "$id" velocity');
      var accel = bound.maxAcceleration;
      if (accel == null) {
        accel = defaultAcceleration;
        bound.maxAcceleration = accel;
        assumptions.push('joint "$id" acceleration $accel (assumed default)');
      }
      positive(accel, 'joint "$id" acceleration');
      var rate = jointJerk == null ? 20.0 : jointJerk[index];
      positive(rate, 'joint "$id" jerk');
      if (jointJerk == null) assumptions.push('joint "$id" jerk $rate (assumed default)');
      bounds.push(bound);
      velocity.push(speed); acceleration.push(accel); jerk.push(rate);
    }
  }

  public function requireGroup(group:KinematicGroup):Void {
    var ids = [for (id in group.jointIds()) Std.string(id)];
    if (ids.length != jointIds.length) throw "Planning limits do not match the group's joints";
    for (index in 0...ids.length)
      if (ids[index] != jointIds[index]) throw "Planning limits do not match the group's joint order";
  }

  public function validation(?modelRevision:Int64, ?calibrationRevision:Int64):ValidationLimits {
    var result = new ValidationLimits(jointIds.length,
      modelRevision == null ? this.modelRevision : modelRevision,
      calibrationRevision == null ? this.calibrationRevision : calibrationRevision);
    for (index in 0...jointIds.length) {
      var bound = bounds[index];
      if (bound.lower < bound.upper) result.position(index, bound.lower, bound.upper);
      result.velocity(index, velocity[index]);
      result.acceleration(index, acceleration[index]);
      result.jerk(index, jerk[index]);
    }
    return result;
  }

  public function startTolerances(position:Float):StartTolerances
    return new StartTolerances([for (_ in jointIds) position],
      [for (value in acceleration) value * 0.01], [for (value in jerk) value * 0.01]);

  /** Check the original drive model with the same steady loads used to derive its caps. */
  public function check(?options:PlanCheckOptions):PlanCheck {
    var settings = options == null ? new PlanCheckOptions() : options.copy();
    settings.steady = steady.copy();
    return new PlanCheck(model, jointIds, settings);
  }

  static function positive(value:Float, label:String):Void {
    if (!Math.isFinite(value) || value <= 0.0)
      throw 'Planning $label limit must be finite and positive';
  }
}
