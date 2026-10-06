package motionkit.robot;

import motionkit.kinematics.KinematicsSolver;
import motionkit.kinematics.PathRequest;
import motionkit.path.PosePath;
import motionkit.path.PoseMath;
import motionkit.planner.JointPathSamples;
import motionkit.robot.AnalyticPathRefiner.RefinementTarget;
import robotkit.manipulation.ArmClearance;

/** Reuse globally selected geometry for execution timing. No inverse solving or
 * fallback selection: a different task, grid or start must be selected again.
 * The checking planner still validates actual generated/timed trajectories. */
class SelectedJointPathPlanner implements JointPathPlanner {
  final solver:KinematicsSolver;
  final checking:JointPathPlanner;
  var records:Array<SelectedPath>;
  public function new(solver:KinematicsSolver,checking:JointPathPlanner,
      paths:Array<PosePath>,curves:Array<JointPathSamples>) {
    if(solver==null || checking==null || paths==null || curves==null || paths.length!=curves.length)
      throw "Selected path reuse requires kinematics, motion checking and aligned paths/curves";
    this.solver=solver;this.checking=checking;records=[];
    for(i in 0...paths.length){
      var path=paths[i],curve=curves[i];
      if(path==null || curve==null || curve.jointCount!=solver.jointCount() || curve.start()!=0 ||
          Math.abs(curve.end()-path.length())>1e-12)
        throw "Selected curve must cover its complete authored section";
      var provider=new PosePathRefinement(path);
      records.push({frame:path.frameId,geometry:geometryOf(path),curve:copy(curve),tasks:[for(s in curve.s)snapshot(provider.at(Math.min(path.length(),s)))]});
    }
  }
  public function withConfiguration(configuration:motionkit.kinematics.SixAxisConfiguration):JointPathPlanner {
    var guard=new ConfigurationConstraint(solver,configuration);
    for(record in records)for(q in record.curve.q)guard.require(q);
    var pinned=new SelectedJointPathPlanner(solver,checking.withConfiguration(configuration),[],[]);
    pinned.records=records;
    return pinned;
  }
  public function withSolver(solver:KinematicsSolver):JointPathPlanner {
    var worker=new SelectedJointPathPlanner(solver,checking.withSolver(solver),[],[]);
    // Private snapshots are read-only; each returned curve is copied.
    worker.records=records;
    return worker;
  }
  public function allowsFreeStart():Bool return false;
  public function retreatTarget():Null<Array<Float>> return null;
  public function checkMotion(trajectory:motionkit.trajectory.Trajectory):Null<ArmClearance.ClearanceViolation>
    return checking.checkMotion(trajectory);
  public function plan(path:PosePath,request:PathRequest,?pinStart:Bool,
      ?entryCheck:(Array<Float>,Array<Float>)->Null<ArmClearance.ClearanceViolation>,
      ?exitCheck:Array<Float>->Null<ArmClearance.ClearanceViolation>):JointPathSamples {
    if(path==null || request==null || pinStart==false)
      throw "Selected curve reuse requires its pinned execution start";
    for(j in 0...request.startQ.length)if(!Math.isFinite(request.startQ[j]) ||
        !Math.isFinite(request.maxJump[j]) || request.maxJump[j]<=0 ||
        !Math.isFinite(request.velocity[j]) || request.velocity[j]<=0)
      throw "Selected curve execution requires finite start and positive limits";
    var geometry=geometryOf(path),provider=new PosePathRefinement(path);
    var mismatches:Array<String> = [];
    for(record in records){
      var curve=record.curve;
      if(record.frame!=path.frameId || record.geometry!=geometry)continue;
      if(curve.s.length!=request.distances.length || curve.jointCount!=request.startQ.length || Math.abs(curve.end()-path.length())>1e-12){
        mismatches.push('grid/count: retained=${curve.s.length}, requested=${request.distances.length}, lengths=${curve.end()}/${path.length()}');continue;
      }
      var matches=true;
      for(j in 0...curve.jointCount)if(Math.abs(curve.q[0][j]-request.startQ[j])>1e-7){
        mismatches.push('start joint $j: retained=${curve.q[0][j]}, requested=${request.startQ[j]}');matches=false;
      }
      for(i in 0...curve.s.length){
        if(!Math.isFinite(request.distances[i]) || Math.abs(curve.s[i]-request.distances[i])>1e-12){matches=false;break;}
        var authored=provider.at(Math.min(path.length(),curve.s[i])),stored=record.tasks[i];
        if(!StructuredJointPathPlanner.sameFreedom(authored.freedom,stored.freedom) ||
            !StructuredJointPathPlanner.sameFreedom(authored.freedom,request.freedoms[i]) ||
            PoseMath.distance(authored.pose,stored.pose)>1e-12 ||
            ToolFreedom.orientationError(authored.pose,stored.pose,authored.freedom)>1e-10 ||
            PoseMath.distance(authored.pose,request.poses[i])>request.tolerance.position ||
            ToolFreedom.orientationError(authored.pose,request.poses[i],authored.freedom)>request.tolerance.orientation)
          matches=false;
        for(axis in 0...6)if(Math.abs(authored.velocity[axis]-stored.velocity[axis])>1e-9 ||
            Math.abs(authored.acceleration[axis]-stored.acceleration[axis])>1e-9 ||
            Math.abs(authored.accelerationBefore[axis]-stored.accelerationBefore[axis])>1e-9)matches=false;
      }
      if(!matches){if(mismatches.length==0)mismatches.push("authored task or derivatives differ");continue;}
      for(i in 0...curve.q.length){
        var actual=solver.forward(curve.q[i]);
        if(PoseMath.distance(actual,request.poses[i])>request.tolerance.position ||
            ToolFreedom.orientationError(actual,request.poses[i],request.freedoms[i])>request.tolerance.orientation)
          throw "Selected curve no longer satisfies execution kinematics";
        if(i>0)for(j in 0...curve.jointCount)if(Math.abs(curve.q[i][j]-curve.q[i-1][j])>request.maxJump[j]+1e-12)
          throw "Selected curve exceeds execution continuity bounds";
      }
      return new JointPathSamples(request.distances,curve.q,curve.qPrime,curve.qDoublePrime,curve.qDoublePrimeBefore);
    }
    throw "No retained selected curve matches the execution task, grid and start"+
      (mismatches.length==0 ? " (authored geometry differs)" : ": "+mismatches.join("; "));
  }
  static function geometryOf(path:PosePath):String
    return haxe.Json.stringify({frame:path.frameId,primitives:[for(primitive in path.primitives)geometryPrimitive(primitive)]});
  static function geometryPrimitive(primitive:motionkit.path.PosePrimitive):Dynamic {
    if(Std.isOfType(primitive,motionkit.path.PoseSlice)){
      var slice=cast(primitive,motionkit.path.PoseSlice);
      return {type:"slice",from:slice.from,to:slice.to,source:geometryPrimitive(slice.source)};
    }
    return {type:Std.isOfType(primitive,motionkit.path.PoseLine)?"line":
      Std.isOfType(primitive,motionkit.path.PoseArc)?"arc":
      Std.isOfType(primitive,motionkit.path.PoseWeave)?"weave":
      throw "Selected curve reuse requires a known pose primitive",data:primitive};
  }
  static function snapshot(task:RefinementTarget):RefinementTarget {
    var freedom=switch task.freedom {case Cone(axis,angle):motionkit.path.OrientationPolicy.Cone(axis.copy(),angle);default:task.freedom;};
    return new RefinementTarget(task.pose,freedom,task.velocity,task.acceleration,task.accelerationBefore);
  }
  static function copy(curve:JointPathSamples):JointPathSamples
    return new JointPathSamples(curve.s,curve.q,curve.qPrime,curve.qDoublePrime,curve.qDoublePrimeBefore);
}
private typedef SelectedPath = {
  final frame:String;
  final geometry:String;
  final curve:JointPathSamples;
  final tasks:Array<RefinementTarget>;
}
