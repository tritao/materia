import motionkit.robot.MachineKitRobotCompiler;
import toolpathkit.motion.ToolpathMotionBinding;
import toolpathkit.motion.MachineBinding;
import toolpathkit.motion.ToolpathChannels;
import motionkit.robot.ProgramCompiler.PendingMotion;
import motionkit.robot.PathMergeBudget;
import motionkit.robot.CouplingProjection;
import motionkit.path.PosePath;
import motionkit.path.GeometricPath;
import motionkit.path.LineSegment;
import motionkit.path.PathPoint;
import motionkit.planner.JointPathSamples;
import motionkit.planner.ToppraPathTiming;
import motionkit.planner.PathTimingLimits;
import toolpathkit.motion.ToolpathPosePrimitive;

class PathMergeBudgetChecks {
  static function check(value:Bool,message:String):Void {if(!value)throw message;}
  public static function fixture():ToolpathMotionBinding {
    var blueprint=MachineKitRobotCompiler.compileGantry(
      new machinekit.gantry.Gantry(new machinekit.gantry.GantrySpec(200,200,200)),0.1,0.4);
    return new ToolpathMotionBinding(new MachineBinding("work","x","y","z",0.08,new ToolpathChannels("")),blueprint);
  }
  public static function run(binding:ToolpathMotionBinding):Void {
    var solver=binding.solver,map=solver.affineMap(),length=0.01;
    if(map==null)throw "Axis solver did not supply its affine map";
    var q0=map.offsets.copy(),q1=[for(j in 0...q0.length)q0[j]+length*map.columns[0][j]];
    var zero=[for(_ in q0)0.0];
    var curve=new JointPathSamples([0,length],[q0,q1],[map.columns[0],map.columns[0]],[zero,zero]);
    var line=new LineSegment(new PathPoint(0,0,0),new PathPoint(length,0,0));
    var path=new PosePath("work",[new ToolpathPosePrimitive(line,0.01,1e-4,0.02)]);
    var timing=new ToppraPathTiming();
    var timed=timing.time(curve,new PathTimingLimits(binding.compiler.maxVelocity,binding.compiler.maxAcceleration,[0.01]));
    try {
      var pending=new PendingMotion(0,q0,q1,timed.trajectory,[],path,curve.s);
      pending.geometryCurve=curve;
      var ids=[for(joint in solver.blueprint.model.joints)joint.id];
      var projection=CouplingProjection.of([for(c in solver.blueprint.model.couplings)
        {leader:ids.indexOf(c.leader),follower:ids.indexOf(c.follower),ratio:c.ratio,offset:c.offset}],q0.length);
      var full=PathMergeBudget.derive(solver,timing,pending,0.01,projection);
      if(full==null)throw "Straight path lost its proven merge budget";
      var sum=0.0;
      for(axis in [solver.x,solver.y,solver.z]){
        var value=full.position[axis.jointIndices[0]]/Math.abs(axis.jointScale(0));sum+=value*value;
      }
      check(sum>0 && Math.sqrt(sum)<=1e-4-Math.sqrt(3.0)*timing.loweringTolerance,"Lowering error was not reserved");
      var slower=PathMergeBudget.derive(solver,timing,pending,0.02,projection);
      if(slower==null)throw "Longer window lost its proven merge budget";
      check(slower.seconds==0.02,"Merge window was not retained");
      for(j in 0...q0.length){
        check(slower.position[j]==full.position[j],"Window changed geometric allowance");
      }
      path.withAuthoredGeometry(new GeometricPath([line]),1e-4,8e-5);
      var rounded=PathMergeBudget.derive(solver,timing,pending,0.01,projection);
      if(rounded==null)throw "Rounded path lost its remaining budget";
      check(rounded.position[solver.x.jointIndices[0]]<full.position[solver.x.jointIndices[0]]/4,
        "Rounded geometry did not spend its allowance");
      check(path.sub(0,1).geometryDeviationBound==8e-5,"Path slice lost the geometric bound");
      path.withAuthoredGeometry(new GeometricPath([line]),1e-4);
      check(PathMergeBudget.derive(solver,timing,pending,0.01,projection)==null,"Unknown geometric error invented a budget");
      path.withAuthoredGeometry(new GeometricPath([line]),1e-4,1e-4);
      check(PathMergeBudget.derive(solver,timing,pending,0.01,projection)==null,"Exhausted tolerance invented a budget");
      Sys.println("Path merge-budget integration checks passed");
      timed.releaseDistanceMap();timed.trajectory.dispose();
    } catch(error:Dynamic){timed.releaseDistanceMap();timed.trajectory.dispose();throw error;}
  }
}
