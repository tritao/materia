package motionkit.robot;

import motionkit.kinematics.IkTolerance;
import motionkit.path.OrientationPolicy;
import motionkit.kinematics.Pose3;
import robotkit.manipulation.KinematicGroup;

/**
 * A cell's redundancy: the values of its external axes (a rail, a
 * positioner). The arm solves for the tool at given external values; the
 * search makes the external axes cheaper to move than the arm, so they bring
 * the work to a comfortable arm.
 */
class ExternalAxesParameterization implements RedundancyParameterization {
  public final group:KinematicGroup;
  var orientationPreference:Null<Pose3> = null;
  var preferTargetOrientation:Bool = false;
  var posturePreference:Null<Array<Float>> = null;
  final indices:Array<Int>;
  final rates:Array<Float>;
  /** Cost factor of an external axis's motion against the arm's. */
  public final externalCost:Float;

  /** `angularRate` (rad per metre of path) and `linearRate` (m per metre) set the lattice resolution. */
  public function new(group:KinematicGroup, ?angularRate:Float = 6.0, ?linearRate:Float = 1.0, ?externalCost:Float = 0.1) {
    if (group == null) throw "An external-axes parameterization needs a group";
    indices = [for (i in 0...group.dofCount()) if (group.external[i]) i];
    if (indices.length == 0) throw "The group has no external axes";
    if (!(angularRate > 0.0) || !(linearRate > 0.0) || !(externalCost > 0.0))
      throw "External-axis rates and cost must be positive";
    this.group = group;
    this.externalCost = externalCost;
    rates = [];
    for (index in indices) {
      var id = group.jointIds()[index];
      var angular = true;
      for (joint in group.robot.joints) if (joint != null && joint.id == id)
        angular = joint.type != robotkit.model.JointType.Prismatic;
      rates.push(angular ? angularRate : linearRate);
    }
  }

  public function preferringOrientation(preference:Null<Pose3>, ?preferTarget:Bool = false):Void {
    orientationPreference = preference;
    preferTargetOrientation = preferTarget;
  }

  public function preferringPosture(preference:Null<Array<Float>>):Void
    posturePreference = preference == null ? null : preference.copy();

  public function dimension():Int return indices.length;

  public function valuesAt(q:Array<Float>):Null<Array<Float>> return [for (index in indices) q[index]];

  public function valuesJacobian(q:Array<Float>):Null<Array<Float>> {
    var n = group.dofCount();
    var rows = [for (_ in 0...indices.length * n) 0.0];
    for (k in 0...indices.length) rows[k * n + indices[k]] = 1.0;
    return rows;
  }

  public function solveAt(target:Pose3, seed:Array<Float>, values:Array<Float>, tolerance:IkTolerance, ?freedom:OrientationPolicy):Null<Array<Float>> {
    for (k in 0...indices.length) {
      var limits = group.group.limitsOf(indices[k]);
      if (limits.lower < limits.upper && (values[k] < limits.lower || values[k] > limits.upper)) return null;
    }
    var task = ToolFreedom.of(target, freedom, tolerance.orientation);
    var options = task.options(tolerance, orientationPreference == null && preferTargetOrientation ? target : orientationPreference);
    if (posturePreference != null) options.preferring(posturePreference);
    var result = group.solve(RedundancyPoses.transform(task.target), seed,
      options.holding(indices, values));
    return result.converged ? result.q : null;
  }

  public function solveNear(target:Pose3, seed:Array<Float>, tolerance:IkTolerance, ?freedom:OrientationPolicy):Null<Array<Float>> {
    var task = ToolFreedom.of(target, freedom, tolerance.orientation);
    var options = task.options(tolerance, orientationPreference == null && preferTargetOrientation ? target : orientationPreference);
    if (posturePreference != null) options.preferring(posturePreference);
    var result = group.solve(RedundancyPoses.transform(task.target), seed, options);
    return result.converged ? result.q : null;
  }

  public function ratesPerMetre():Array<Float> return rates.copy();
  public function periodic(index:Int):Bool return false;
  public function costFactors():Array<Float> return [for (i in 0...group.dofCount()) group.external[i] ? externalCost : 1.0];
}
