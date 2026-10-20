package app;

import materia.project.SceneArtifact.SceneArtifactEnclosureGuard;
import processkit.guard.EnclosureGuard;
import processkit.guard.EnclosureGuard.GuardInput;
import processkit.guard.EnclosureGuard.GuardOutput;
import processkit.guard.EnclosureGuard.DoorCommand;
import processkit.guard.EnclosureGuard.GuardFault;
import nativekit.sim.SimSession;
import robotkit.core.RobotSnapshot;
import sys.thread.Mutex;
import robotkit.runtime.Simulation;
import motionkit.trajectory.Trajectory;

/** Live simulation PLC: measured door switches/velocities, separate lock actuator, and independent door-actuator ownership. */
class EnclosureGuardPlayer implements SessionMember {
  public final config:SceneArtifactEnclosureGuard;
  final robot:AssemblyRobot;
  final session:SimSession;
  final simulation:Simulation;
  final robotIndex:Int;
  final doorSpeed:Float;
  final doorAcceleration:Float;
  final cutting:Null<HobbingProgramPlayer>;
  final doorIndex:Int;
  final offset:Float;
  final spindleIndices:Array<Int>;
  final axisIndices:Array<Int>;
  var logic:EnclosureGuard;
  var input = new GuardInput();
  var output:Null<GuardOutput>;
  var doorTrajectory:Null<Trajectory>;
  var trajectoryTime=0.0;
  var commanded=0.0;
  var commandedVelocity=0.0;
  var driveInitialized=false;
  var feedbackSampled=false;
  var initialized=false;
  var playbackRunning=false;
  var pendingTarget:Null<Float>;
  var doorCommand:DoorCommand = Hold;
  var locked = false;
  var lockCommand = true;
  var lockElapsed = 0.0;
  var boundaryClear = true;
  var faultReset = false;
  var axesStillSeconds=0.0;
  var spindleStillSeconds=0.0;
  final viewMutex = new Mutex();
  var view:EnclosureGuardStatus;

  public function new(config:SceneArtifactEnclosureGuard, robot:AssemblyRobot, session:SimSession,
      project:ProjectDocumentSession, simulation:Simulation, ?cutting:HobbingProgramPlayer) {
    this.config=config;this.robot=robot;this.session=session;this.cutting=cutting;this.simulation=simulation;
    if(robot.parts.length==0)throw "Enclosure actuator has no simulation placement";
    robotIndex=robot.parts[0].robotIndex;
    for(value in [config.stroke,config.setupSpeedLimit,config.timeoutSeconds,config.lockTravelSeconds,
        config.linearStandstillTolerance,config.rotaryStandstillTolerance,config.standstillConfirmationSeconds])
      if(!Math.isFinite(value)||value<=0)throw "Enclosure guard needs finite positive travel, timing and feedback policy";
    if(config.standstillConfirmationSeconds>=config.timeoutSeconds)throw "Standstill confirmation must fit inside the guard timeout";
    doorIndex=joint(config.doorJoint);
    if(robot.model.joints[doorIndex].type!=robotkit.model.JointType.Prismatic)throw "Guard door must be a sliding joint";
    if(config.spindleJoints.length==0||config.axisJoints.length==0)throw "Enclosure guard needs spindle and axis observations";
    spindleIndices=[for(id in config.spindleJoints)joint(id)];axisIndices=[for(id in config.axisJoints)joint(id)];
    if(spindleIndices.indexOf(doorIndex)>=0||axisIndices.indexOf(doorIndex)>=0)throw "Door cannot be its own hazardous-motion observation";
    var seen=new Map<Int,Bool>();
    for(index in spindleIndices.concat(axisIndices)) {if(seen.exists(index))throw "Guard observations must name distinct joints";seen.set(index,true);}
    for(id in [config.closedSensor,config.openSensor]) {
      var sensors=[for(s in robot.blueprint.sensors)if(s.id==id&&s.kind=="joint_switch"&&s.joint==doorIndex)s];
      if(sensors.length!=1)throw 'Enclosure guard needs measured door switch "$id"';
    }
    var definition=project.projectAssemblyDefinition;if(definition==null)throw "Enclosure guard needs assembly placement";
    offset=new cadkit.modeling.AssemblyState(definition,project.projectAssemblyState).joint(config.doorJoint)*project.assemblyUnitMetres();
    var limits=robot.model.joints[doorIndex].limits;
    if(-offset<limits.lower-1e-8||config.stroke-offset>limits.upper+1e-8)throw "Guard door stroke exceeds mechanical travel";
    doorSpeed=limits.requireVelocity();doorAcceleration=limits.requireAcceleration();
    if(robot.blueprint.processModelJoints.indexOf(doorIndex)<0)throw "Enclosure actuator must own its process joint";
    logic=new EnclosureGuard(config.setupSpeedLimit,config.timeoutSeconds);
    if(cutting!=null)cutting.setGuardPermission(false);
    view=new EnclosureGuardStatus(makeObservation("Waiting for door feedback",offset,0),false);
  }
  function joint(id:String):Int {
    for(j in 0...robot.model.joints.length)if(robot.model.joints[j].id==id)return j;
    throw 'Enclosure guard joint "$id" is missing';
  }
  function switchValue(snapshot:RobotSnapshot,id:String):Null<Bool> {
    for(n in 0...snapshot.sensors.length) {
      var s=snapshot.sensors.get(n);
      if(s.sensorId==id) return s.values.length==1&&Math.isFinite(s.values.get(0))?s.values.get(0)>=0.5:null;
    }
    return null;
  }
  function stationary(snapshot:RobotSnapshot,indices:Array<Int>):Bool {
    for(j in indices) {
      var tolerance=robot.model.joints[j].type==robotkit.model.JointType.Prismatic?config.linearStandstillTolerance:config.rotaryStandstillTolerance;
      var velocity=snapshot.velocities.get(j);
      if(!Math.isFinite(velocity)||Math.abs(velocity)>tolerance)return false;
    }
    return true;
  }
  public function requestOpen():Void {input.openRequest=true;input.closeRequest=false;input.cutRequest=false;}
  public function requestClose():Void {input.closeRequest=true;input.openRequest=false;input.cutRequest=false;}
  public function requestCut():Void {
    input.openRequest=false;input.closeRequest=false;
    if(cutting!=null&&cutting.canStartGuardedCut())input.cutRequest=true;
  }
  public function resetGuard():Void {input.openRequest=false;input.closeRequest=false;input.cutRequest=false;faultReset=true;}
  public function setBoundaryClear(clear:Bool):Void boundaryClear=clear;

  public function feed():Void {
    var s=robot.robot.snapshot();
    if(session.simulationTime()<=0)return; // Native switches publish after the first physics tick.
    var closed=switchValue(s,config.closedSensor),open=switchValue(s,config.openSensor);
    feedbackSampled=closed!=null&&open!=null&&Math.isFinite(s.positions.get(doorIndex))&&Math.isFinite(s.velocities.get(doorIndex));
    input.closed=closed==true;input.open=open==true;
    axesStillSeconds=stationary(s,axisIndices)?Math.min(config.standstillConfirmationSeconds,axesStillSeconds+session.fixedTimestep()):0;
    spindleStillSeconds=stationary(s,spindleIndices)?Math.min(config.standstillConfirmationSeconds,spindleStillSeconds+session.fixedTimestep()):0;
    input.axesStandstill=axesStillSeconds>=config.standstillConfirmationSeconds;
    input.spindleStandstill=spindleStillSeconds>=config.standstillConfirmationSeconds;
    input.axisSpeed=0;
    for(j in axisIndices)if(robot.model.joints[j].type==robotkit.model.JointType.Prismatic)
      input.axisSpeed=Math.max(input.axisSpeed,Math.abs(s.velocities.get(j)));
    if(closed==null||open==null||!Math.isFinite(s.positions.get(doorIndex))||!Math.isFinite(s.velocities.get(doorIndex))) {input.closed=true;input.open=true;}
    for(j in spindleIndices.concat(axisIndices))if(!Math.isFinite(s.velocities.get(j))||!Math.isFinite(s.positions.get(j))) {input.closed=true;input.open=true;}
    // A requested controlled access stop reports STOPPING through its held state.
    // Emergency-stop/fault states still trip the safety chain immediately.
    input.safetyReady=s.faultCode==0&&(s.safety==RobotKitRuntimeConstants.RK_SAFETY_READY||
      cutting!=null&&cutting.guardInterrupted&&s.safety==RobotKitRuntimeConstants.RK_SAFETY_STOPPING);
    input.boundaryClear=boundaryClear;input.cutActive=cutting!=null&&cutting.guardMotionActive();
    // The lock has its own simulated actuator and delayed feedback. A command is not confirmation.
    var lockMayMove=lockCommand?input.closed&&Math.abs(s.velocities.get(doorIndex))<=config.linearStandstillTolerance:
      input.axesStandstill&&input.spindleStandstill;
    if(lockCommand!=locked&&lockMayMove) {
      lockElapsed+=session.fixedTimestep();
      if(lockElapsed>=config.lockTravelSeconds) {locked=lockCommand;lockElapsed=0;}
    } else lockElapsed=0;
    input.locked=locked;input.reset=faultReset;faultReset=false;
    var previousFault=logic.fault;
    var nextOutput=logic.update(input,session.fixedTimestep());output=nextOutput;lockCommand=nextOutput.lock;
    if(feedbackSampled&&input.axesStandstill&&input.spindleStandstill&&nextOutput.fault==None)initialized=true;
    if(nextOutput.fault!=None&&nextOutput.fault!=previousFault&&Sys.getEnv("ENCLOSURE_GUARD_TRACE")=="1")
      Sys.println("ENCLOSURE_GUARD "+haxe.Json.stringify({phase:Std.string(logic.phase),fault:Std.string(nextOutput.fault),time:session.simulationTime(),
        safety:s.safety,faultCode:s.faultCode,door:s.positions.get(doorIndex)+offset,doorVelocity:s.velocities.get(doorIndex),
        observations:[for(j in spindleIndices.concat(axisIndices)){joint:robot.model.joints[j].id,q:s.positions.get(j),v:s.velocities.get(j)}]}));
    if(cutting!=null)cutting.setGuardPermission(nextOutput.allowCut);
    if(!driveInitialized) {commanded=s.positions.get(doorIndex);driveInitialized=true;}
    if(nextOutput.door!=doorCommand) {
      doorCommand=nextOutput.door;
      if(doorCommand!=Hold)pendingTarget=(doorCommand==OpenDoor?config.stroke:0.0)-offset;
    }
    if(nextOutput.fault!=None||!input.safetyReady) {
      disposeDoorTrajectory();pendingTarget=null;commanded=s.positions.get(doorIndex);commandedVelocity=0;
    }
    var trajectory=doorTrajectory;
    // End switches may arrive during deceleration. Finish the bounded profile before
    // starting a newly requested direction; lock engagement also requires measured door standstill.
    if(trajectory==null&&pendingTarget!=null&&Math.abs(s.velocities.get(doorIndex))<=config.linearStandstillTolerance) {
      var target:Float=cast pendingTarget;pendingTarget=null;trajectoryTime=0;
      trajectory=Trajectory.generateStateToState([s.positions.get(doorIndex)],[s.velocities.get(doorIndex)],[0.0],[target],
        [doorSpeed*0.95],[doorAcceleration*0.95],[doorAcceleration*20]);doorTrajectory=trajectory;
    }
    if(trajectory!=null) {
      trajectoryTime=Math.min(trajectory.durationSeconds(),trajectoryTime+session.fixedTimestep());
      var sample=trajectory.evaluate(trajectoryTime);commanded=sample.positions[0];commandedVelocity=sample.velocities[0];
      if(trajectoryTime>=trajectory.durationSeconds())disposeDoorTrajectory();
    }
    // Physics feedback and interpolated endpoints can lie a few ulps past a hard stop.
    // Keep the commanded target inside the authored mechanical interval.
    var limits=robot.model.joints[doorIndex].limits;
    commanded=Math.max(limits.lower,Math.min(limits.upper,commanded));
    simulation.driveProcessJoint(robotIndex,doorIndex,commanded,commandedVelocity);
    publishObservation(s);
    if(cutting!=null)cutting.advanceScenario(this);
  }
  function disposeDoorTrajectory():Void {
    var trajectory=doorTrajectory;doorTrajectory=null;if(trajectory!=null)trajectory.dispose();
  }

  function makeObservation(message:String,position:Float,velocity:Float):app.EnclosureGuardStatus.EnclosureGuardObservation {
    var fault=output!=null&&output.fault!=None;
    return {phase:Std.string(logic.phase),message:message,position:position,closed:input.closed,open:input.open,
      locked:locked,spindleStandstill:input.spindleStandstill,axesStandstill:input.axesStandstill,
      canStart:!fault&&logic.phase==Secured&&cutting!=null&&cutting.canStartGuardedCut(),
      allowCut:output!=null&&output.allowCut,fault:fault,interrupted:cutting!=null&&cutting.guardInterrupted,
      feedbackSampled:feedbackSampled,initialized:initialized,doorMoving:Math.abs(velocity)>config.linearStandstillTolerance};
  }
  function publishObservation(snapshot:RobotSnapshot):Void {
    var message=output!=null&&output.fault!=None?switch output.fault {
      case SafetyChain: "Guard stopped: check the E-stop and access boundary.";
      case InvalidFeedback: "Guard stopped: door feedback is missing or inconsistent.";
      case GuardLost: "Guard stopped: closed or locked confirmation was lost.";
      case StandstillLost: "Guard stopped: motion was detected during access.";
      case Timeout: "Guard stopped: motion or lock confirmation timed out.";
      case _: "Guard stopped: release conflicting requests and reset the guard.";
    }:switch logic.phase {
      case Secured:
        cutting!=null&&cutting.scenario=="idle"?"Idle. Door controls available; no cutting program runs.":
        cutting!=null&&cutting.failure!=null?"Cut stopped. Reset the simulation before retrying.":
        cutting!=null&&cutting.guardInterrupted?"Door secured. Reset the simulation before restarting the interrupted cut.":
        cutting!=null&&cutting.completed?"Cut completed. Door closed and locked.":
        cutting!=null&&cutting.cutStarted?"Cut running. Door closed and locked.":"Door closed and locked. Ready for Start cut.";
      case Stopping: input.locked?"Stopping the machine. Door stays locked until spindle and axes stop.":"Waiting for spindle and axis standstill. Door motion and cutting are disabled.";
      case Unlocking: "Waiting for unlocked confirmation.";
      case Opening: "Opening door. Cutting is disabled.";
      case Open: "Door open. Cutting is disabled.";
      case Closing: "Closing door. Cutting is disabled.";
      case Locking: "Waiting for locked confirmation.";
      case _: "Waiting for door feedback.";
    };
    var next=makeObservation(message,snapshot.positions.get(doorIndex)+offset,snapshot.velocities.get(doorIndex));
    viewMutex.acquire();view=new EnclosureGuardStatus(next,playbackRunning);viewMutex.release();
  }
  /** The simulation owner already published its image; UI presentation never reads mutable inputs. */
  public function present():Void {}
  /** Called by the transport owner before playback starts or after its worker has stopped. */
  public function setRunning(value:Bool):Void {
    viewMutex.acquire();playbackRunning=value;view=view.withRunning(value);viewMutex.release();
  }
  public function status():EnclosureGuardStatus {viewMutex.acquire();var result=view;viewMutex.release();return result;}
  public function beforeReset():Void {disposeDoorTrajectory();if(cutting!=null)cutting.setGuardPermission(false);}
  public function reset():Void {
    logic=new EnclosureGuard(config.setupSpeedLimit,config.timeoutSeconds);input=new GuardInput();output=null;feedbackSampled=false;initialized=false;
    disposeDoorTrajectory();pendingTarget=null;driveInitialized=false;commanded=0;commandedVelocity=0;doorCommand=Hold;locked=false;lockCommand=true;lockElapsed=0;boundaryClear=true;faultReset=false;axesStillSeconds=0;spindleStillSeconds=0;
    if(cutting!=null)cutting.setGuardPermission(false);publishObservation(robot.robot.snapshot());
  }
}
