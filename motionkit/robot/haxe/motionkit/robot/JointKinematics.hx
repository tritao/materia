package motionkit.robot;

import motionkit.kinematics.KinematicsSolver;
import motionkit.kinematics.Pose3;
import motionkit.kinematics.IkTolerance;
import motionkit.kinematics.Twist6;
import motionkit.kinematics.PathRequest;

/** Joint-program planning for machines without a Cartesian tool frame. */
class JointKinematics implements KinematicsSolver {
  final count:Int;
  public function new(count:Int) {
    if (count < 1) throw "Joint kinematics needs at least one joint";
    this.count = count;
  }
  public function jointCount():Int return count;
  public function fork():KinematicsSolver return this;
  public function forward(q:Array<Float>):Pose3 throw "This machine exposes joint motion only";
  public function solvePose(target:Pose3, seed:Array<Float>, tolerance:IkTolerance):Null<Array<Float>>
    throw "This machine exposes joint motion only";
  public function sampleCandidates(target:Pose3, maxCount:Int, tolerance:IkTolerance):Array<Array<Float>>
    throw "This machine exposes joint motion only";
  public function solveDifferential(q:Array<Float>, twist:Twist6, ?redundancyRate:Array<Float>):Null<Array<Float>>
    throw "This machine exposes joint motion only";
  public function solvePath(request:PathRequest):Array<Null<Array<Float>>>
    throw "This machine exposes joint motion only";
}
