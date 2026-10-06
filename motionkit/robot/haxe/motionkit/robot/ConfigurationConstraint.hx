package motionkit.robot;

import motionkit.kinematics.KinematicsSolver;
import motionkit.kinematics.SixAxisConfiguration;
import motionkit.kinematics.Pose3;

/** Verify a physical state using labelled IK, including external-group adapters. */
class ConfigurationConstraint {
  final backend:AnalyticIk;
  public final configuration:SixAxisConfiguration;
  public function new(solver:KinematicsSolver,configuration:SixAxisConfiguration) {
    if(configuration==null)throw "Configuration constraint requires a pin";
    if(Std.isOfType(solver,ManipulatorKinematics))
      backend=BranchIk.of(cast(solver,ManipulatorKinematics).manipulator);
    else if(Std.isOfType(solver,OpwKinematics))backend=cast solver;
    else throw "Configuration pin requires compiled labelled six-axis kinematics";
    if(backend.family()!="OPW" && backend.family()!="UR6R")
      throw "Configuration pin requires a labelled six-axis geometric backend";
    this.configuration=new SixAxisConfiguration(configuration.shoulder,configuration.elbow,configuration.wrist,configuration.turns);
  }
  public function candidates(pose:Pose3,seed:Array<Float>):Array<Array<Float>>
    return [for(branch in backend.branches(pose,seed))if(configuration.accepts(branch.configuration))branch.q];
  public function accepts(q:Array<Float>):Bool {
    for(candidate in candidates(backend.forward(q),q)){
      var same=candidate.length==q.length;
      if(same)for(joint in 0...q.length)if(Math.abs(candidate[joint]-q[joint])>1e-6)same=false;
      if(same)return true;
    }
    return false;
  }
  public function require(q:Array<Float>):Void {
    if(!accepts(q))throw 'Physical state violates configuration pin ${configuration.label()}';
  }
  /** Check the generated execution states at controller ticks and segment boundaries.
   * This is a sampled geometric check, not a continuous branch certificate. */
  public function checkTrajectory(trajectory:motionkit.trajectory.Trajectory,period:Float):Void {
    var duration=trajectory.durationSeconds(),ticks=Math.ceil(duration/period);
    for(index in 0...(ticks+1))require(trajectory.evaluate(Math.min(duration,index*period)).positions);
    for(segment in trajectory.segments()){
      require(trajectory.evaluate(haxe.Int64.toFloat(segment.timeFromStartNs)*1e-9).positions);
      require(trajectory.evaluate(haxe.Int64.toFloat(haxe.Int64.add(segment.timeFromStartNs,segment.durationNs))*1e-9).positions);
    }
  }
}
