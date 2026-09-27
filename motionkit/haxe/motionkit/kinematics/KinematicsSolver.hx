package motionkit.kinematics;

/** Transport-neutral forward, inverse and differential kinematics contract. */
interface KinematicsSolver {
  function jointCount():Int;
  function forward(q:Array<Float>):Pose3;
  function solvePose(target:Pose3, seed:Array<Float>,
    tolerance:IkTolerance):Null<Array<Float>>;
  function sampleCandidates(target:Pose3, maxCount:Int,
    tolerance:IkTolerance):Array<Array<Float>>;
  function solveDifferential(q:Array<Float>, twist:Twist6):Null<Array<Float>>;
}
