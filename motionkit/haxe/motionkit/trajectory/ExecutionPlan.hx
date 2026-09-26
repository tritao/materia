package motionkit.trajectory;

import MotionKitNative;
import haxe.Int64;

/** Validated, immutable native trajectory with explicit start-state assumptions. */
class ExecutionPlan {
  final owner:Ownedmk_plan_handle;
  var disposed:Bool = false;
  public final report:ValidationReport;
  public final planId:Int64;
  public final modelRevision:Int64;
  public final calibrationRevision:Int64;
  public final trajectoryRevision:Int64;
  public final requiredCapabilities:Int64;
  public final planningAuthority:Int;
  public final durationSeconds:Float;

  private function new(owner:Ownedmk_plan_handle, report:ValidationReport) {
    this.owner = owner;
    this.report = report;
    var info = new mk_plan_info();
    info.set_struct_size(mk_plan_info.size());
    check(MotionKitNative.mk_plan_get_info(owner.borrow(), info), "plan.info");
    planId = info.get_plan_id();
    modelRevision = info.get_model_revision();
    calibrationRevision = info.get_calibration_revision();
    trajectoryRevision = info.get_trajectory_revision();
    requiredCapabilities = info.get_required_capabilities();
    planningAuthority = info.get_planning_authority();
    durationSeconds = Std.parseFloat(Int64.toStr(info.get_duration_ns())) * 1e-9;
  }

  /** Does not infer derivatives from degree-1 chords; callers supply the authored state. */
  public static function create(trajectory:Trajectory, limits:ValidationLimits, planId:Int64,
      positions:Array<Float>, velocities:Array<Float>, accelerations:Array<Float>,
      positionTolerances:Array<Float>, velocityTolerances:Array<Float>,
      accelerationTolerances:Array<Float>):ExecutionPlan {
    var count = trajectory.jointCount();
    if (count != limits.jointCount || positions.length != count || velocities.length != count ||
        accelerations.length != count || positionTolerances.length != count ||
        velocityTolerances.length != count || accelerationTolerances.length != count)
      throw "Plan start-state joint count mismatch";
    var start = new mk_start_state();
    start.set_struct_size(mk_start_state.size());
    start.set_joint_count(count);
    for (joint in 0...count) {
      start.set_position(joint, positions[joint]);
      start.set_velocity(joint, velocities[joint]);
      start.set_acceleration(joint, accelerations[joint]);
      start.set_position_tolerance(joint, positionTolerances[joint]);
      start.set_velocity_tolerance(joint, velocityTolerances[joint]);
      start.set_acceleration_tolerance(joint, accelerationTolerances[joint]);
    }
    var spec = new mk_plan_spec();
    spec.set_struct_size(mk_plan_spec.size());
    spec.set_plan_id(planId);
    spec.set_model_revision(limits.modelRevision);
    spec.set_calibration_revision(limits.calibrationRevision);
    spec.set_required_capabilities(Int64.ofInt(MotionKitNativeConstants.MK_CAP_TIMED_TRAJECTORY));
    spec.set_planning_authority(MotionKitNativeConstants.MK_AUTHORITY_MATERIA);
    spec.set_start_state(start);
    var nativeReport = new mk_validation_report();
    nativeReport.set_struct_size(mk_validation_report.size());
    var created = MotionKitNative.mk_plan_create(trajectory.owner.borrow(), spec,
      limits.nativeValue(), nativeReport);
    if (created.status == MotionKitNativeConstants.MK_ERROR_LIMIT)
      throw new PlanLimitError(new ValidationReport(nativeReport));
    check(created.status, "plan.create");
    return new ExecutionPlan(created.out_plan, new ValidationReport(nativeReport));
  }

  public function evaluate(timeSeconds:Float):TrajectoryState {
    if (disposed) throw "Native plan has been disposed";
    var state = new mk_trajectory_state();
    state.set_struct_size(mk_trajectory_state.size());
    check(MotionKitNative.mk_plan_evaluate(owner.borrow(),
      Int64.fromFloat(Math.floor(timeSeconds * 1e9 + 0.5)), state), "plan.evaluate");
    var positions:Array<Float> = [];
    var velocities:Array<Float> = [];
    var accelerations:Array<Float> = [];
    var jerks:Array<Float> = [];
    for (joint in 0...state.get_joint_count()) {
      positions.push(state.get_position(joint));
      velocities.push(state.get_velocity(joint));
      accelerations.push(state.get_acceleration(joint));
      jerks.push(state.get_jerk(joint));
    }
    return new TrajectoryState(positions, velocities, accelerations, jerks);
  }

  public function dispose():Void {
    if (disposed) return;
    disposed = true;
    owner.close();
  }

  static function check(status:Int, operation:String):Void {
    if (status != MotionKitNativeConstants.MK_OK)
      throw '$operation failed with MotionKit error $status';
  }
}
