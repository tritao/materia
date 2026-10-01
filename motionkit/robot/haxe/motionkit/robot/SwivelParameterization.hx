package motionkit.robot;

import motionkit.kinematics.IkTolerance;
import motionkit.kinematics.Pose3;
import robotkit.manipulation.KinematicGroup;

/** A 7-axis arm's redundancy: its swivel angle (`robotkit.manipulation.ArmSwivel`). */
class SwivelParameterization implements RedundancyParameterization {
  public final group:KinematicGroup;
  /** Swivel resolution per metre of path (radians). */
  public final rate:Float;

  public function new(group:KinematicGroup, ?rate:Float = 5.0) {
    if (group == null || !group.redundant()) throw "A swivel parameterization needs a redundant arm";
    if (!(rate > 0.0)) throw "Swivel rate must be positive";
    this.group = group;
    this.rate = rate;
  }

  public function dimension():Int return 1;

  public function valuesAt(q:Array<Float>):Null<Array<Float>> {
    var angle = group.swivelAngle(q);
    return Math.isFinite(angle) ? [angle] : null;
  }

  public function solveAt(target:Pose3, seed:Array<Float>, values:Array<Float>, tolerance:IkTolerance):Null<Array<Float>> {
    var result = group.solve(RedundancyPoses.transform(target), seed, RedundancyPoses.options(tolerance).atSwivel(values[0], true));
    return result.converged ? result.q : null;
  }

  public function solveNear(target:Pose3, seed:Array<Float>, tolerance:IkTolerance):Null<Array<Float>> {
    var angle = group.swivelAngle(seed);
    if (!Math.isFinite(angle)) return null;
    var result = group.solve(RedundancyPoses.transform(target), seed, RedundancyPoses.options(tolerance).atSwivel(angle, false));
    return result.converged ? result.q : null;
  }

  public function ratesPerMetre():Array<Float> return [rate];
  public function periodic(index:Int):Bool return true;
  public function costFactors():Array<Float> return [for (_ in 0...group.dofCount()) 1.0];
}
