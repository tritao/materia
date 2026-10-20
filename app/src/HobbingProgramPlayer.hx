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
  public var scenario(default,null):Null<String>;
  public var scenarioStage(default,null)="waiting";
  var scenarioAccess=false;
  var parkStarted=false;
  public final job:HobbingJob;
  public final handling:Null<HobbingHandling>;
  public var cutCompleted(default,null)=false;
  public var cutStarted(default,null)=false;
  public var headRetracted(default,null)=false;
  var handlingStart=-1.0;
  var handlingClock=0.0;
  var unloadStart=-1.0;
  var sleeveDisplay=-1.0;
  var sleeveInstalled=false;
  public var sleeveMeshUploads(default,null)=0;
  final sleeveCenter:Array<Float>;
  final withdrawal:Null<motionkit.program.MotionProgram>;
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
  final centerMetres:Array<Float>;
  final frontDirection:Array<Float>;
  var physicalPresentation=false;
  final indices:Array<Int>;
  final offsets:Array<Float>;
  final observerId:Int;
  final compiler:ProgramCompiler;
  final program:motionkit.program.MotionProgram;
  var motion:ManipulatorMotion;
  var started=false;
  var guardPermission=true;
  public var guardInterrupted(default,null)=false;
  var guardStopRequested=false;
  public function canStartGuardedCut():Bool return scenario!="idle"&&!started&&!completed&&failure==null&&!guardInterrupted;
  public function guardMotionActive():Bool return motion.running;
  public function setGuardPermission(allowed:Bool):Void {
    guardPermission=allowed;
    if(!allowed&&(started||parkStarted)&&motion.running&&!guardStopRequested) {
      guardStopRequested=true;guardInterrupted=true;
      try motion.stop() catch(error:Dynamic) {
        failure="Guard stop: "+Std.string(error);
        robot.robot.stop(robotkit.core.StopMode.Emergency);
      }
    }
  }

  final spindleAtSpeedChannel:String;
  var startTime=0.0;
  var disposed=false;
  final trace:Bool=Sys.getEnv("HOBBER_TRACE")=="1";
  var traceTime=-1.0;
  public function new(config:SceneArtifactHobbing,robot:AssemblyRobot,simulation:Simulation,session:SimSession,project:ProjectDocumentSession) {
    var configHandling=config.handling;
    handling=configHandling==null?null:new HobbingHandling(configHandling.frontApproachMm==null?0:cast configHandling.frontApproachMm,configHandling.liftMm==null?120:cast configHandling.liftMm);
    this.config=config;this.robot=robot;this.simulation=simulation;this.session=session;this.project=project;
    indices=[for(id in config.axes)[for(j in 0...robot.model.joints.length)if(robot.model.joints[j].id==id)j][0]];
    var definition=project.projectAssemblyDefinition;if(definition==null)throw "Hobbing needs assembly placement";
    var placement=new cadkit.modeling.AssemblyState(definition,project.projectAssemblyState);
    offsets=[for(i in 0...6)placement.joint(config.axes[i])*(i<3?project.assemblyUnitMetres():1.0)];
    var frame=placement.worldPose(config.stock);
    frontDirection=AssemblyRobot.rotate([frame.qx,frame.qy,frame.qz,frame.qw],[-Math.cos(offsets[5]),Math.sin(offsets[5]),0]);
    var component=definition.occurrences.filter(o->o.id==config.stock)[0].definition;
    var c=project.assemblyPreviewCenter(component);centerMetres=c==null?[0.0,0,0]:[for(value in c)value*project.assemblyUnitMetres()];
    var spec=new GearSpec(config.module,config.teeth,config.width,config.pressureAngle);
    var hob=new HobDesign(ConjugateRack.of(new ToothProfile(spec)),config.starts,config.outside,config.gashes);
    var settings=new HobbingSettings();settings.feedPerRevolution=config.feed;settings.cuttingSpeed=config.cuttingSpeed;
    if(config.tolerance>=0.001)settings.infeed=5; // coarse smoke: faster below-face air approach, mm/s
    settings.xAxis=config.axes[0];settings.yAxis=config.axes[1];settings.zAxis=config.axes[2];settings.tiltJoint=config.axes[3];settings.hobJoint=config.axes[4];settings.tableJoint=config.axes[5];
    job=new HobbingJob(spec,hob,config.length,settings);
    var spindleDrives=[for(drive in robot.blueprint.velocityDrives)if(drive.joint==indices[4])drive];
    if(spindleDrives.length!=1)throw "Hobbing needs one declared spindle velocity drive";
    var spindleDrive=spindleDrives[0],speedSuffix=".speed";
    if(!StringTools.endsWith(spindleDrive.speedChannel,speedSuffix))throw "Hobbing spindle needs named speed/direction channels";
    var spindleName=spindleDrive.speedChannel.substr(0,spindleDrive.speedChannel.length-speedSuffix.length);
    if(spindleDrive.directionChannel!=spindleName+".direction")throw "Hobbing spindle channels must share their declared name";
    var binding=new MachineBinding("hobber-axes",config.axes[0],config.axes[1],config.axes[2],0.02,new ToolpathChannels("",spindleName));
    spindleAtSpeedChannel=binding.channels.spindleAtSpeed;
    binding.spindleJointId=config.axes[4];binding.auxiliaryJoints=[config.axes[3],config.axes[5]];
    var delays:Map<String,Int> = [config.axes[4]=>0,config.axes[5]=>0]; // physics samples carry their real timestamp; no extra transport delay
    GearBinding.install(binding,robot.model,config.controllerHz,delays,[config.axes[5]]);
    var axes=[for(i in 0...3){var j=robot.model.joints[indices[i]],l=robot.model.coupledLimits(j.id,new robotkit.model.SteadyLoads());
      new motionkit.axis.MotionAxisBlueprint(config.axes[i],[j.id],j.limits.lower+offsets[i],j.limits.upper+offsets[i],l.requireVelocity(),l.requireAcceleration(),offsets[i],[1.0],[-offsets[i]]);}];
    var bp=new MotionSystemBlueprint(robot.model,robot.blueprint,axes,session.fixedTimestep());
    var held=[config.axes[4]];
    var guardConfig=project.machineMotion==null?null:project.machineMotion.enclosureGuard;
    if(guardConfig!=null) {
      // A process actuator may carry mechanically coupled door leaves or shafts.
      var owned=robot.blueprint.processJoints();
      for(index in 0...owned.length)if(owned[index]&&held.indexOf(robot.model.joints[index].id)<0)
        held.push(robot.model.joints[index].id);
    }
    var planning=new ToolpathMotionBinding(binding,bp,null,null,held);compiler=planning.compiler;
    var lowered=new ToolpathLowering(binding).lower([for(op in job.program.ops)switch op{
      case PositionJoint(id,position,span):var at=config.axes.indexOf(id);toolpathkit.path.ToolpathOp.PositionJoint(id,position-(at<0?0:offsets[at]),span);
      case _:op;
    }]);if(lowered.program==null)throw "Empty hobbing program";program=lowered.program;
    var hc=config.handling;
    var handlingSpan=toolpathkit.path.Provenance.cam(0,"loading clearance");
    withdrawal=hc==null?null:new ToolpathLowering(binding).lower([
      toolpathkit.path.ToolpathOp.SetSetup("hobber-axes",handlingSpan),
      toolpathkit.path.ToolpathOp.PositionJoint(config.axes[0],hc.retractX/1000-offsets[0],handlingSpan),
      toolpathkit.path.ToolpathOp.PositionJoint(config.axes[2],-offsets[2],handlingSpan),
      toolpathkit.path.ToolpathOp.PositionJoint(config.axes[1],-offsets[1],handlingSpan),
      toolpathkit.path.ToolpathOp.End(handlingSpan)]).program;
    if(hc!=null){
      var sleeveId=hc.sleeve;
      var sleeve=definition.occurrences.filter(o->o.id==sleeveId)[0];
      if(sleeve==null)throw "Handling sleeve is absent";
      var sc=project.assemblyPreviewCenter(sleeve.definition);sleeveCenter=[for(v in sc)v*project.assemblyUnitMetres()];
    }else sleeveCenter=[0.0,0,0];
    motion=newMotion();stock=new HobbingStock(config,centerMetres);observerId=simulation.addStepObserver(this);
    selectScenario(project.hobbingScenario==null?config.scenario:project.hobbingScenario);
  }
  public function selectScenario(id:Null<String>):Void {
    if(id!=null&&["idle","cutting","loading"].indexOf(id)<0)throw "Unknown hobbing scenario";
    scenario=id;scenarioStage=id=="idle"?"idle":"waiting";scenarioAccess=false;parkStarted=false;handlingClock=0;
    if(handling!=null){handling.inspect("sequence");if(id=="cutting"||id=="idle")handling.cutting();else handling.load(0);}
  }
  /** Runs on the guard's playback owner after a measured feedback image is published. */
  public function advanceScenario(guard:EnclosureGuardPlayer):Void {
    if(scenario==null||scenario=="idle"||completed)return;
    var s=guard.status();scenarioAccess=s.initialized&&s.open&&!s.locked&&s.axesStandstill&&s.spindleStandstill&&!s.fault;
    if(s.fault||guardInterrupted||failure!=null){scenarioStage="stopped";scenarioAccess=false;return;}
    if(!s.initialized)return;
    if(scenarioStage=="waiting") {
      if(s.phase=="Open"){guard.requestClose();scenarioStage="securing";}
      else if(s.canStart){guard.requestCut();scenarioStage=scenario=="loading"?"parking":"cutting";}
    }else if(scenarioStage=="securing"&&s.canStart){guard.requestCut();scenarioStage=scenario=="loading"?"parking":"cutting";}
    else if(scenarioStage=="opening-load"||scenarioStage=="opening-unload") {
      if(s.phase=="Secured"){guard.requestOpen();scenarioStage=scenarioStage=="opening-load"?"wait-load-door":"wait-unload-door";}
    }else if(scenarioAccess&&(scenarioStage=="wait-load-door"||scenarioStage=="wait-unload-door")) {
      scenarioStage=scenarioStage=="wait-load-door"?"loading":"unloading";handlingClock=0;
      if(handling!=null){if(scenarioStage=="loading")handling.load(0);else handling.unload(0);}
    }else if(scenarioStage=="closing-cut"&&s.phase=="Open"){guard.requestClose();scenarioStage="wait-cut-door";}
    else if(scenarioStage=="wait-cut-door"&&s.canStart){guard.requestCut();scenarioStage="cutting";headRetracted=false;}
  }
  public function scenarioLabel():String return switch(scenarioStage) {
    case "idle":"Idle - door controls available";
    case "waiting", "securing", "wait-cut-door":"Waiting for closed and locked feedback";
    case "parking":"Parking head for loading";
    case "opening-load", "wait-load-door":"Opening doors for loading";
    case "loading":"Loading and clamping cup";
    case "closing-cut":"Closing doors before cutting";
    case "cutting":cutCompleted?"Retracting head":"Cutting";
    case "opening-unload", "wait-unload-door":"Opening doors for unloading";
    case "unloading":"Releasing and unloading cup";
    case "completed":"Completed";
    case "stopped":"Stopped - reset before retrying";
    default:"Ready";
  };
  function newMotion():ManipulatorMotion return new ManipulatorMotion(robot.robot,compiler,input,()->robot.runtime.pollEvents());
  function input(channel:String):Null<EventValue> {
    var s=robot.robot.snapshot(),b=s.velocities.get(indices[4]);
    if(channel=="hob.job_ready")return Digital(Math.abs(s.positions.get(indices[4])+offsets[4])<1e-5&&Math.abs(b)<0.01);
    if(channel=="hob.stopped")return Digital(Math.abs(b)<0.01);
    if(channel==spindleAtSpeedChannel)return Digital(Math.abs(b-job.rpm*2*Math.PI/60)<Math.max(0.01,job.rpm*2*Math.PI/60*0.001));
    return null;
  }
  /** Display-only transfer time; paused playback never advances it. */
  public function advanceHandling(seconds:Float):Void {
    if(scenario!=null) {
      if(scenario=="loading"&&scenarioAccess&&(scenarioStage=="loading"||scenarioStage=="unloading"))handlingClock+=seconds;
    }else if(handling!=null&&!completed&&handling.inspection=="sequence"&&(!cutStarted||headRetracted))handlingClock+=seconds;
  }
  public function feed():Void {
    if(disposed)return;
    if(scenario=="loading"&&!guardInterrupted&&failure==null&&handling!=null&&(scenarioStage=="loading"||scenarioStage=="unloading")) {
      if(!scenarioAccess)return;
      if(scenarioStage=="loading") {if(handling.load(handlingClock))scenarioStage="closing-cut";}
      else if(handling.unload(handlingClock)){completed=true;scenarioStage="completed";}
      return;
    }
    if(!guardPermission||guardInterrupted) {
      if(guardStopRequested&&motion.running)motion.update(session.fixedTimestep());
      if(motion.failure!=null)failure=motion.failure;
      return;
    }
    if(completed||failure!=null||scenario=="idle")return;
    if(scenario!=null&&scenarioStage!="cutting"&&scenarioStage!="parking")return;
    try{
      if(scenarioStage=="parking") {
        if(!parkStarted){var park=withdrawal;if(park==null)throw "Loading needs a park program";motion.run(park);parkStarted=true;}
        motion.update(session.fixedTimestep());failure=motion.failure;
        if(motion.completed){headRetracted=true;scenarioStage="opening-load";}
        return;
      }
      var now=session.simulationTime();
      if(handling!=null&&handling.inspection!="sequence")return;
      if(scenario==null&&handling!=null&&!cutStarted){
        if(handlingStart<0)handlingStart=handlingClock;
        if(!handling.load(handlingClock-handlingStart))return;
      }
      if(!started){startTime=now;motion.run(program);started=true;cutStarted=true;}
      if(!cutCompleted){
        if(handling!=null)handling.cutting();
        motion.update(session.fixedTimestep());failure=motion.failure;
        if(motion.completed){
          cutCompleted=true;elapsed=now-startTime;stock.finish(elapsed,syncError);
          if(handling==null||scenario=="cutting"){completed=true;scenarioStage="completed";return;}
          var retract=withdrawal;if(retract==null)throw "Missing head withdrawal program";
          motion.run(retract);handling.retracting();
        }
      }else if(handling!=null&&!headRetracted){
        handling.retracting();motion.update(session.fixedTimestep());failure=motion.failure;
        if(motion.completed){
          var x=(robot.robot.snapshot().positions.get(indices[0])+offsets[0])*1000;
          var hc=config.handling;if(hc==null)throw "Handling configuration absent";
          var q=robot.robot.snapshot().positions;
          if(Math.abs(x-hc.retractX)>0.01||Math.abs((q.get(indices[2])+offsets[2])*1000)>0.01||Math.abs((q.get(indices[1])+offsets[1])*1000)>0.01)throw "Head has not reached loading park X/Y/Z";
          headRetracted=true;unloadStart=handlingClock;if(scenario=="loading")scenarioStage="opening-unload";
        }
      }else if(handling!=null)completed=handling.unload(handlingClock-unloadStart);
    }catch(e:Dynamic){failure=Std.string(e);motion.abort();}
  }
  public function afterSimulationStep(timestamp:Int64):Void {
    if(!started||cutCompleted||completed||failure!=null||disposed||guardInterrupted&&!motion.running)return;
    try{
      var q=robot.robot.snapshot().positions;
      var axes=new HobbingAxes((q.get(indices[0])+offsets[0])*1000,(q.get(indices[1])+offsets[1])*1000,(q.get(indices[2])+offsets[2])*1000,q.get(indices[3])+offsets[3],q.get(indices[4])+offsets[4],q.get(indices[5])+offsets[5]);
      stock.follow(Int64.toFloat(timestamp)*1e-9,axes);
      if(trace&&session.simulationTime()-traceTime>=0.25){traceTime=session.simulationTime();
        var contactCount=-1;var pairs:Array<Dynamic> = [];
        try{var contacts=simulation.robotContacts(robot.runtime);contactCount=contacts.length;
          for(contact in contacts)if(contact.active)pairs.push({a:robot.model.links[contact.linkIndex].id,b:contact.otherLink<0?"environment":robot.model.links[contact.otherLink].id,distanceM:contact.distance});}catch(ignored:Dynamic){}
        Sys.println("HOBBER_TRACE "+haxe.Json.stringify({t:traceTime,x:axes.x,z:axes.z,b:axes.b,c:axes.c,
          cSet:robot.robot.snapshot().setpointPositions.get(indices[5])+offsets[5],contacts:contactCount,contactPairs:pairs,
          expected:axes.b*config.starts/config.teeth+job.passes[0].tablePhase,
          removed:stock.forces.last==null?0:stock.forces.last.removedVolume}));
      }
      // Compare measured phase only while the material feed is underway (A and C datum already positioned).
      if(stock.forces.last!=null&&stock.forces.last.removedVolume>0&&axes.b>1){var phase=job.passes[0].tablePhase;
        syncError=Math.max(syncError,Math.abs(axes.c-axes.b*config.starts/config.teeth-phase));}
      elapsed=session.simulationTime()-startTime;
      if(stock.maxBytes>128*1024*1024||stock.stream.peakFrames>4096)throw "Hobbing allocation budget exceeded";
    }catch(e:Dynamic){failure=Std.string(e);motion.abort();}
  }
  /** Inspection is paused and view-only; never resets or rewrites the cut stock. */
  public function inspectCup(mode:String):Void {
    if(handling==null)throw "This scene has no cup handling demonstration";
    if(mode!="sequence"&&cutStarted&&!completed)throw "Finish or reset the handling sequence before inspecting another state";
    if(mode=="finished"&&!cutCompleted)throw "No finished cut is available";
    handling.inspect(mode);present();
  }
  /** Use the same authored/live frame selection as the viewport. */
  public function presentStockFrame(physical:Bool):Void {physicalPresentation=physical;present();}
  public function present():Void {
    if(disposed)return;
    var lift=handling==null?0:handling.displayedLift(),blank=handling!=null&&(handling.inspection=="blank"||handling.inspection=="clamped");
    var front=handling==null?0:handling.displayedFront();
    var object=project.scene.object("project:"+config.stock);
    if(object==null||object.rotation==null)throw "Cup presentation frame is absent";
    var q=object.rotation,c=robot.robot.snapshot().positions.get(indices[5])+offsets[5];
    // Sensors are safe to read on the UI owner while physics runs in the background.
    // The cup rotates only about C; paused inspection uses the authored scene frame.
    var local=physicalPresentation?[-front*Math.cos(c),front*Math.sin(c),0.0]:
      AssemblyRobot.rotate([-q[0],-q[1],-q[2],q[3]],[for(v in frontDirection)v*front]);
    var stockId="project:"+config.stock,geometry=stock.geometry(blank);
    if(geometry!=null)project.scene.setRuntimeGeometryParts(stockId,1,[0=>geometry]);
    project.scene.setRuntimeGeometryPartTransform(stockId,0,nativekit.scene.Transform.identity().translated(local[0]/1000,local[1]/1000,lift/1000));
    if(handling!=null){
      var clamp=handling.displayedClamp();
      if(clamp!=sleeveDisplay){
        var h=config.handling;if(h==null)throw "Handling configuration absent";
        if(!sleeveInstalled){
          var outer=h.outside/2,inner=outer-h.wall;
          var sleeve=CutterGeometry.sectionGeometry([{r:inner,z:0.0},{r:outer,z:0.0},{r:outer,z:h.length},{r:inner,z:h.length},{r:inner,z:0.0}],
            (x,y,z)->[x/1000-sleeveCenter[0],y/1000-sleeveCenter[1],z/1000-sleeveCenter[2]],(x,y,z)->[x,y,z]);
          project.scene.setRuntimeGeometryParts("project:"+h.sleeve,1,[0=>sleeve]);sleeveInstalled=true;sleeveMeshUploads++;
        }
        // Illustrative outer-radius fit take-up, affine radial scaling about the bore axis.
        // This is display-only; hydraulic strain/contact and wall thickness are not qualified.
        var scale=1+clamp*h.radialTravel/(h.outside/2);
        project.scene.setRuntimeGeometryPartTransform("project:"+h.sleeve,0,nativekit.scene.Transform.identity().set(0,scale).set(5,scale)
          .translated((scale-1)*sleeveCenter[0],(scale-1)*sleeveCenter[1],0));
        sleeveDisplay=clamp;
      }
    }
  }
  public function beforeReset():Void{if(motion.running)motion.abort();}
  public function reset():Void{stock.dispose();stock=new HobbingStock(config,centerMetres);motion=newMotion();started=false;cutStarted=false;cutCompleted=false;headRetracted=false;completed=false;failure=null;syncError=0;elapsed=0;
    guardInterrupted=false;guardStopRequested=false;
    handlingStart=-1;handlingClock=0;unloadStart=-1;sleeveDisplay=-1;
    if(handling!=null){handling.inspect("sequence");handling.load(0);}selectScenario(scenario);}
  public function dispose():Void{if(disposed)return;beforeReset();disposed=true;simulation.removeStepObserver(observerId);stock.dispose();}
}
