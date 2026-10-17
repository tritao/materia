package app;

import haxe.Int64;
import camkit.*;
import gearkit.*;
import stockkit.hobbing.HobbingAxes;
import materia.project.SceneArtifact.SceneArtifactHobbing;
import robotkit.runtime.Simulation;
import robotkit.runtime.SimulationStepObserver;
import nativekit.sim.SimSession;
import motionkit.robot.*;
import motionkit.event.EventValue;
import toolpathkit.motion.*;

/** HB8 program executed through the ordinary plan runtime; cuts only measured after-step state. */
class HobbingProgramPlayer implements SessionMember implements SimulationStepObserver {
  public final job:HobbingJob;
  public var stock(default,null):HobbingStock;
  public var failure(default,null):Null<String> = null;
  public var completed(default,null):Bool=false;
  public var elapsed(default,null):Float=0;
  public var syncError(default,null):Float=0;
  final config:SceneArtifactHobbing;
  final robot:AssemblyRobot;
  final simulation:Simulation;
  final session:SimSession;
  final project:ProjectDocumentSession;
  final center:Array<Float>;
  final indices:Array<Int>;
  final offsets:Array<Float>;
  final observerId:Int;
  final compiler:ProgramCompiler;
  final program:motionkit.program.MotionProgram;
  var motion:ManipulatorMotion;
  var started=false;
  var startTime=0.0;
  var disposed=false;
  public function new(config:SceneArtifactHobbing,robot:AssemblyRobot,simulation:Simulation,session:SimSession,project:ProjectDocumentSession) {
    this.config=config;this.robot=robot;this.simulation=simulation;this.session=session;this.project=project;
    indices=[for(id in config.axes)[for(j in 0...robot.model.joints.length)if(robot.model.joints[j].id==id)j][0]];
    var definition=project.projectAssemblyDefinition;if(definition==null)throw "Hobbing needs assembly placement";
    var placement=new cadkit.modeling.AssemblyState(definition,project.projectAssemblyState);
    offsets=[for(i in 0...6)placement.joint(config.axes[i])*(i<3?project.assemblyUnitMetres():1.0)];
    var component=definition.occurrences.filter(o->o.id==config.stock)[0].definition;
    var c=project.assemblyPreviewCenter(component);center=c==null?[0.0,0,0]:c.copy();
    var spec=new GearSpec(config.module,config.teeth,config.width,config.pressureAngle);
    var hob=new HobDesign(ConjugateRack.of(new ToothProfile(spec)),config.starts,config.outside,config.gashes);
    var settings=new HobbingSettings();settings.feedPerRevolution=config.feed;settings.cuttingSpeed=config.cuttingSpeed;
    settings.xAxis=config.axes[0];settings.yAxis=config.axes[1];settings.zAxis=config.axes[2];settings.tiltJoint=config.axes[3];settings.hobJoint=config.axes[4];settings.tableJoint=config.axes[5];
    job=new HobbingJob(spec,hob,config.length,settings);
    var binding=new MachineBinding("hobber-axes",config.axes[0],config.axes[1],config.axes[2],0.02,new ToolpathChannels("","hob"));
    binding.spindleJointId=config.axes[4];binding.auxiliaryJoints=[config.axes[3],config.axes[5]];
    var delays:Map<String,Int> = [config.axes[4]=>0,config.axes[5]=>0]; // physics samples carry their real timestamp; no extra transport delay
    GearBinding.install(binding,robot.model,config.controllerHz,delays,[config.axes[5]]);
    var axes=[for(i in 0...3){var j=robot.model.joints[indices[i]],l=robot.model.coupledLimits(j.id,new robotkit.model.SteadyLoads());
      new motionkit.axis.MotionAxisBlueprint(config.axes[i],[j.id],j.limits.lower+offsets[i],j.limits.upper+offsets[i],l.requireVelocity(),l.requireAcceleration(),offsets[i],[1.0],[-offsets[i]]);}];
    var bp=new MotionSystemBlueprint(robot.model,robot.blueprint,axes,session.fixedTimestep());
    var planning=new ToolpathMotionBinding(binding,bp,null,null,[config.axes[4]]);compiler=planning.compiler;
    var lowered=new ToolpathLowering(binding).lower([for(op in job.program.ops)switch op{
      case PositionJoint(id,position,span):var at=config.axes.indexOf(id);toolpathkit.path.ToolpathOp.PositionJoint(id,position-(at<0?0:offsets[at]),span);
      case _:op;
    }]);if(lowered.program==null)throw "Empty hobbing program";program=lowered.program;
    motion=newMotion();stock=new HobbingStock(config,center);observerId=simulation.addStepObserver(this);
  }
  function newMotion():ManipulatorMotion return new ManipulatorMotion(robot.robot,compiler,input,()->robot.runtime.pollEvents());
  function input(channel:String):Null<EventValue> {
    var s=robot.robot.snapshot(),b=s.velocities.get(indices[4]);
    if(channel=="hob.job_ready")return Digital(Math.abs(s.positions.get(indices[4])+offsets[4])<1e-5&&Math.abs(b)<0.01);
    if(channel=="hob.stopped")return Digital(Math.abs(b)<0.01);
    if(channel=="hob.at_speed")return Digital(Math.abs(b-job.rpm*2*Math.PI/60)<Math.max(0.01,job.rpm*2*Math.PI/60*0.001));
    return null;
  }
  public function feed():Void {
    if(completed||failure!=null||disposed)return;
    try{
      if(!started){startTime=session.simulationTime();motion.run(program);started=true;}
      motion.update(session.fixedTimestep());failure=motion.failure;
      if(motion.completed){completed=true;elapsed=session.simulationTime()-startTime;stock.finish(elapsed,syncError);}
    }catch(e:Dynamic){failure=Std.string(e);motion.abort();}
  }
  public function afterSimulationStep(timestamp:Int64):Void {
    if(!started||completed||failure!=null||disposed)return;
    try{
      var q=robot.robot.snapshot().positions;
      var axes=new HobbingAxes((q.get(indices[0])+offsets[0])*1000,(q.get(indices[1])+offsets[1])*1000,(q.get(indices[2])+offsets[2])*1000,q.get(indices[3])+offsets[3],q.get(indices[4])+offsets[4],q.get(indices[5])+offsets[5]);
      stock.follow(Int64.toFloat(timestamp)*1e-9,axes);
      // Compare measured phase only while the material feed is underway (A and C datum already positioned).
      if(axes.x<job.safeX-0.1&&axes.b>1){var phase=job.passes[0].tablePhase;
        syncError=Math.max(syncError,Math.abs(axes.c-axes.b*config.starts/config.teeth-phase));}
      elapsed=session.simulationTime()-startTime;
      if(stock.maxBytes>128*1024*1024||stock.stream.peakFrames>4096)throw "Hobbing allocation budget exceeded";
    }catch(e:Dynamic){failure=Std.string(e);motion.abort();}
  }
  public function present():Void{if(disposed)return;var geometry=stock.geometry();if(geometry!=null)project.scene.setRuntimeGeometry("project:"+config.stock,geometry);}
  public function beforeReset():Void{if(motion.running)motion.abort();}
  public function reset():Void{stock.dispose();stock=new HobbingStock(config,center);motion=newMotion();started=false;completed=false;failure=null;syncError=0;elapsed=0;}
  public function dispose():Void{if(disposed)return;beforeReset();disposed=true;simulation.removeStepObserver(observerId);stock.dispose();}
}
