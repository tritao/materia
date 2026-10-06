package processkit;

import haxe.Int64;
import motionkit.MotionOptions;
import motionkit.event.EventValue;
import motionkit.event.PathEvent;
import motionkit.program.Blend;
import motionkit.program.InputPredicate;
import motionkit.program.MotionOp;
import motionkit.program.MotionProgram;
import motionkit.program.MoveTarget;
import motionkit.planner.JointPathSamples;
import motionkit.robot.CompiledProgram;
import motionkit.robot.ProgramCompiler;
import motionkit.robot.SelectedJointPathPlanner;
import motionkit.robot.StructuredJointPathPlanner;
import robotkit.manipulation.KinematicGroup;
import robotkit.manipulation.ArmClearance;
import processkit.WelderProcessDevice.WelderChannels;
import processkit.WeldPathProblem.WeldPathPhase;

/** One engagement program over globally selected weld sections. Construction
 * does not solve, time or submit motion. Compilation reuses the selection. */
class WeldPathProgram {
  public final program:MotionProgram;
  public final sectionOps:Array<Int>;
  final problem:WeldPathProblem;
  final curves:Array<JointPathSamples>;
  final channels:WelderChannels;
  final lastWeldOp:Int;
  final phases:Array<WeldPathPhase>;
  final quantity:Float;
  final endRate:Float;

  public function new(problem:WeldPathProblem,curves:Array<JointPathSamples>,channels:WelderChannels) {
    if(problem==null || curves==null || curves.length!=problem.sections.length || channels==null)
      throw "Weld execution requires aligned globally selected sections and channels";
    this.problem=problem;this.channels={arc:channels.arc,wireSpeed:channels.wireSpeed,voltage:channels.voltage};
    this.curves=[for(curve in curves)new JointPathSamples(curve.s,curve.q,curve.qPrime,curve.qDoublePrime,curve.qDoublePrimeBefore)];
    var parameters=problem.plan.parameters;
    phases=problem.phases.copy();quantity=parameters.wireSpeed/parameters.travelSpeed;endRate=parameters.wireSpeed;
    var ops:Array<MotionOp> = [
      MotionOp.SetOutput(channels.wireSpeed,EventValue.Analog(0)),
      MotionOp.SetOutput(channels.arc,EventValue.Digital(false)),
      MotionOp.SetOutput(channels.voltage,EventValue.Analog(parameters.voltage)),
      MotionOp.MoveJ(MoveTarget.JointTarget(this.curves[0].q[0].copy()),new MotionOptions(),Blend.ExactStop)
    ];
    sectionOps=[];var engaged=false,finalWeld=-1;
    for(i in 0...problem.sections.length){
      var phase=phases[i],path=problem.sections[i];
      if(phase==Weld && !engaged){
        ops.push(MotionOp.SetOutput(channels.wireSpeed,EventValue.Analog(parameters.wireSpeed)));
        ops.push(MotionOp.SetOutput(channels.arc,EventValue.Digital(true)));
        ops.push(MotionOp.WaitInput(WeldingPlanRunner.ARC_ESTABLISHED,
          InputPredicate.Equals(EventValue.Digital(true)),WeldingPlanRunner.IGNITION_TIMEOUT));
        if(parameters.startDwell>0)ops.push(MotionOp.Dwell(parameters.startDwell));
        engaged=true;
      }
      if(phase==Burnback){
        if(parameters.craterDwell>0)ops.push(MotionOp.Dwell(parameters.craterDwell));
        ops.push(MotionOp.SetOutput(channels.wireSpeed,EventValue.Analog(0)));
      }
      sectionOps.push(ops.length);
      if(phase==Weld)finalWeld=ops.length;
      var events=phase==Burnback ? [new PathEvent(path.length(),channels.arc,EventValue.Digital(false))] : [];
      var feed=phase==Weld?parameters.travelSpeed:phase==Burnback?
        Math.max(WeldPathPlanner.LIFT/parameters.burnback,0.01):path.primitives[0].speedLimit();
      ops.push(MotionOp.FollowPath(path,path.frameId,feed,events));
    }
    lastWeldOp=finalWeld;program=new MotionProgram(ops);
  }

  /** One compiler pass, including generated entry, drive/task/clearance checks
   * and rates derived from the final timed deposition sections. */
  public function compile(compiler:ProgramCompiler,group:KinematicGroup,start:Array<Float>,planId:Int64,
      ?clearance:ArmClearance):CompiledProgram {
    var checking=new StructuredJointPathPlanner(group,null,null,clearance,8,false,
      null,null,null,0,null,problem.contact);
    var selected=new SelectedJointPathPlanner(compiler.solver,checking,problem.sections,curves);
    var execution=compiler.withJointPathPlanner(selected),quantity=this.quantity;
    var sectionOps=this.sectionOps.copy(),phases=this.phases.copy(),channel=channels.wireSpeed;
    var lastOp=lastWeldOp,endRate=this.endRate;
    execution.pathEventSchedule=(op,offset,last,distances,times,events)->{
      var section=sectionOps.indexOf(op);
      return section<0 || phases[section]!=Weld ? events :
        ProcessRateSchedule.timedSection(channel,quantity,distances,times,events,op==lastOp && last,endRate);
    };
    return execution.compile(program,start,planId);
  }
}
