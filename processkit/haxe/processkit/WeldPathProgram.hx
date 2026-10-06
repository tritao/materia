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
  final lengths:Array<Float>;
  final seamOffsets:Array<Float>;
  final coveredPrefix:Float;
  public final ignitionOp:Int;

  public function new(problem:WeldPathProblem,curves:Array<JointPathSamples>,channels:WelderChannels,?interruptedAt:Float) {
    if(problem==null || curves==null || curves.length!=problem.sections.length || channels==null)
      throw "Weld execution requires aligned globally selected sections and channels";
    var recovering=interruptedAt!=null;
    if(!recovering && problem.startDistance!=0)
      throw "A sliced weld requires explicit recovery engagement";
    if(recovering && (!Math.isFinite(cast interruptedAt) || cast(interruptedAt,Float)<problem.startDistance ||
        cast(interruptedAt,Float)>=problem.fullSeamLength))
      throw "Weld recovery interruption lies outside the remaining seam";
    coveredPrefix=recovering ? cast(interruptedAt,Float)-problem.startDistance : 0.0;
    this.problem=problem;this.channels={arc:channels.arc,wireSpeed:channels.wireSpeed,voltage:channels.voltage};
    this.curves=[for(curve in curves){var copy=new JointPathSamples(curve.s,curve.q,curve.qPrime,curve.qDoublePrime,curve.qDoublePrimeBefore);copy.clearanceProof=curve.clearanceProof;copy;}];
    var parameters=problem.plan.parameters;
    phases=problem.phases.copy();quantity=parameters.wireSpeed/parameters.travelSpeed;endRate=parameters.wireSpeed;
    var ops:Array<MotionOp> = [
      MotionOp.SetOutput(channels.wireSpeed,EventValue.Analog(0)),
      MotionOp.SetOutput(channels.arc,EventValue.Digital(false)),
      MotionOp.SetOutput(channels.voltage,EventValue.Analog(parameters.voltage)),
      MotionOp.MoveJ(MoveTarget.JointTarget(this.curves[0].q[0].copy()),new MotionOptions(),Blend.ExactStop)
    ];
    lengths=[for(path in problem.sections)path.length()];seamOffsets=[];
    var seamProgress=0.0;
    for(i in 0...phases.length){seamOffsets.push(seamProgress);if(phases[i]==Weld)seamProgress+=lengths[i];}
    sectionOps=[];var engaged=false,finalWeld=-1,ignition=-1;
    for(i in 0...problem.sections.length){
      var phase=phases[i],path=problem.sections[i];
      if(phase==Weld && !engaged){
        ops.push(MotionOp.SetOutput(channels.wireSpeed,EventValue.Analog(recovering?processkit.tool.WeldArcModel.MIN_WIRE_SPEED:parameters.wireSpeed)));
        ops.push(MotionOp.SetOutput(channels.arc,EventValue.Digital(true)));
        ignition=ops.length;
        ops.push(MotionOp.WaitInput(WeldingPlanRunner.ARC_ESTABLISHED,
          InputPredicate.Equals(EventValue.Digital(true)),WeldingPlanRunner.IGNITION_TIMEOUT));
        if(!recovering && parameters.startDwell>0)ops.push(MotionOp.Dwell(parameters.startDwell));
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
    ignitionOp=ignition;lastWeldOp=finalWeld;program=new MotionProgram(ops);
  }

  /** Pure progress mapping; barriers use the adopted compilation's block
   * metadata. Air/lift travel never counts toward deposited seam distance. */
  public function progress(compiled:CompiledProgram,current:motionkit.robot.ManipulatorProgress,
      completed:Bool=false):WeldPathProgress {
    if(compiled==null || current==null)throw "Weld progress requires its compilation and motion cursor";
    var end=problem.fullSeamLength,start=problem.startDistance;
    if(completed)return new WeldPathProgress(Complete,end);
    var section=sectionOps.indexOf(current.op);
    if(section>=0){
      var distance=Math.max(0.0,Math.min(lengths[section],current.pathDistance));
      return switch phases[section] {
        case Approach:new WeldPathProgress(Approaching,start);
        case Weld:new WeldPathProgress(Depositing,Math.min(end,start+seamOffsets[section]+distance));
        case Burnback:new WeldPathProgress(BurningBack,end);
        case Retreat:new WeldPathProgress(Retreating,end);
      };
    }
    if(current.barrier!=null)return switch current.barrier {
      case WaitInput(_,_,_):new WeldPathProgress(WaitingForArc,start);
      case Dwell(_):
        var welded=false;
        for(block in 0...Std.int(Math.min(compiled.blocks.length,current.block+1)))
          for(op in compiled.blocks[block].opIndices){var index=sectionOps.indexOf(op);
            if(index>=0 && phases[index]==Weld)welded=true;}
        new WeldPathProgress(welded?FillingCrater:Pooling,welded?end:start);
    };
    if(current.op>=ignitionOp && current.op<firstWeldOp())
      return new WeldPathProgress(current.op==ignitionOp?WaitingForArc:Pooling,start);
    if(current.op>lastWeldOp)return new WeldPathProgress(FillingCrater,end);
    return new WeldPathProgress(Approaching,start);
  }
  function firstWeldOp():Int {
    for(i in 0...phases.length)if(phases[i]==Weld)return sectionOps[i];
    throw "Weld program has no deposition section";
  }

  /** One compiler pass, including generated entry, drive/task/clearance checks
   * and rates derived from the final timed deposition sections. */
  public function compile(compiler:ProgramCompiler,group:KinematicGroup,start:Array<Float>,planId:Int64,
      ?clearance:ArmClearance,?configuration:motionkit.kinematics.SixAxisConfiguration):CompiledProgram {
    var checking=new StructuredJointPathPlanner(group,null,null,clearance,8,false,
      null,null,null,0,null,problem.contact,problem.contactNeighborhood);
    var selected=new SelectedJointPathPlanner(compiler.solver,checking,problem.sections,curves);
    var execution=compiler.withJointPathPlanner(selected),quantity=this.quantity;
    var sectionOps=this.sectionOps.copy(),phases=this.phases.copy(),channel=channels.wireSpeed;
    // Cartesian corner paths retain drive-limited timing; redundant serial
    // paths must already sustain the process feed after drive-aware refinement.
    var fixedProcessFeed=!Std.isOfType(motionkit.robot.BranchIk.of(group),motionkit.robot.CartesianAnalyticIk);
    execution.requireFeasiblePath=op->{var section=sectionOps.indexOf(op);return fixedProcessFeed && section>=0 && phases[section]==Weld;};
    var lastOp=lastWeldOp,endRate=this.endRate;
    var overlap=this.coveredPrefix,seamOffsets=this.seamOffsets.copy();
    execution.pathEventSchedule=(op,offset,last,distances,times,events)->{
      var section=sectionOps.indexOf(op);
      return section<0 || phases[section]!=Weld ? events :
        ProcessRateSchedule.timedSection(channel,quantity,distances,times,events,op==lastOp && last,endRate,
          Math.max(0.0,Math.min(lengths[section],overlap-seamOffsets[section])),
          processkit.tool.WeldArcModel.MIN_WIRE_SPEED);
    };
    var began=Sys.time();
    var compiled=execution.compile(configuration==null ? program : new MotionProgram(program.ops,configuration),start,planId);
    if(Sys.getEnv("PROCESS_PATH_PROFILE")=="1"){
      var rows:Array<Dynamic> = [];
      for(block in compiled.blocks)for(i in 0...block.plans.length){
        var section=sectionOps.indexOf(block.opIndices[i]);
        rows.push({op:block.opIndices[i],phase:section<0 ? "entry/dwell" : Std.string(phases[section]),
          lengthMetres:block.pathLengths[i],durationSeconds:block.plans[i].durationSeconds});
      }
      Sys.println("PROCESS_PATH_WELD_CLOCK "+haxe.Json.stringify({compileSeconds:Sys.time()-began,
        authoredTravelSpeed:problem.plan.parameters.travelSpeed,sections:rows,
        maxVelocity:compiler.maxVelocity,maxAcceleration:compiler.maxAcceleration}));
    }
    return compiled;
  }
}

/** Explicit engagement state and authored deposited distance for recovery. */
class WeldPathProgress {
  public final phase:WeldExecutionPhase;
  public final seamDistance:Float;
  public function new(phase:WeldExecutionPhase,seamDistance:Float){this.phase=phase;this.seamDistance=seamDistance;}
}
enum WeldExecutionPhase {
  Approaching;
  WaitingForArc;
  Pooling;
  Depositing;
  FillingCrater;
  BurningBack;
  Retreating;
  Complete;
}
