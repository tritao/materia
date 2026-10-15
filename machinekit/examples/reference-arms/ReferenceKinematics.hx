import motionkit.kinematics.IkTolerance;
import motionkit.kinematics.KinematicsSolver;
import motionkit.kinematics.Pose3;
import motionkit.path.OrientationPolicy;
import motionkit.robot.EaikKinematics;
import motionkit.robot.ManipulatorKinematics;
import robotkit.manipulation.KinematicGroup;

/** Existing EAIK and DLS backends, preserving imperfect vendor geometry.
 * Numerical fallback is local and unlabelled; it does not promise all IK branches.
 */
class ReferenceKinematics extends ManipulatorKinematics {
  public final exact:Null<EaikKinematics>;
  public final unsupportedDiagnostic:Null<String>;
  public var numericFallbacks(default,null):Int = 0;

  public function new(group:KinematicGroup) {
    super(group);
    var solver:Null<EaikKinematics> = null, diagnostic:Null<String> = null;
    try solver = new EaikKinematics(group) catch (error:Dynamic) {
      var message = Std.string(error);
      if (!StringTools.startsWith(message,"EAIK has no supported decomposition")) throw error;
      diagnostic = message;
    }
    exact = solver; unsupportedDiagnostic = diagnostic;
  }

  override public function fork():KinematicsSolver return new ReferenceKinematics(manipulator);

  override public function solvePose(target:Pose3,seed:Array<Float>,tolerance:IkTolerance,
      ?freedom:OrientationPolicy):Null<Array<Float>> {
    if (exact != null) {
      var solution = exact.solvePose(target,seed,tolerance,freedom);
      if (solution != null) return solution;
    }
    numericFallbacks++;
    return super.solvePose(target,seed,tolerance,freedom);
  }

  override public function sampleCandidates(target:Pose3,maxCount:Int,tolerance:IkTolerance,
      ?freedom:OrientationPolicy):Array<Array<Float>> {
    if (exact != null) {
      var candidates = exact.sampleCandidates(target,maxCount,tolerance,freedom);
      if (candidates.length > 0) return candidates;
    }
    numericFallbacks++;
    return super.sampleCandidates(target,maxCount,tolerance,freedom);
  }
}
