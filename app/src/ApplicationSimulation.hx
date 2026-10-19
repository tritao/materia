package app;

import haxe.Int64;
import materia.assembly.AssemblyFrames;
import humankit.sim.HumanWorker;
import humankit.sim.HumanWorkerSignals;
import robotkit.runtime.RobotRuntime;
import robotkit.runtime.RobotRuntimeBlueprint;
import robotkit.runtime.RobotRuntimeCompiler;
import robotkit.runtime.SimulatedPresenceInputAdapter;
import robotkit.runtime.SimulatedRobotZoneAdapter;
import robotkit.runtime.Simulation;
import robotkit.runtime.SimulationSpace;
import robotkit.runtime.SimulationPresentationSnapshot;
import robotkit.core.Robot;
import robotkit.world.RobotWorld;
import robotkit.world.WorldSnapshot;
import robotkit.simulation.SimulatedRobot;
import robotkit.core.SensorFrame;
import nativekit.sim.MotionType;
import nativekit.sim.SimObject;
import nativekit.sim.SimPose;
import nativekit.sim.SimSession;
import nativekit.sim.SimShape;
import nativekit.sim.SimContactFilter;
import nativekit.sim.SimContactSurface;
import app.CncProgramPlayer.CncJob;
import motionkit.event.EventValue;
import sys.thread.Mutex;
#if !wasm
import sys.thread.Thread;
import sys.thread.Lock;
#end

typedef SimulationRobotVisual={
  var id:String;
  var position:Array<Float>;
  var rotation:Array<Float>;
  var sensors:Array<SensorFrame>;
  var links:Array<SimulationPoseVisual>;
}
typedef SimulationPoseVisual={var id:String;var position:Array<Float>;var rotation:Array<Float>;}

/** Geometry-centred part frame, prepared once for the active simulation configuration. */
private typedef AssemblyVisual = {
  var id:String;
  var robotIndex:Int;
  var linkIndex:Int;
  var offset:materia.assembly.AssemblyRecord.AssemblyFrame;
}

/**
 * Owns the one shared editable-scene simulation attached to the application
 * world: a SimKit session in which the robots, the environment props, and any
 * people take part.
 */
class ApplicationSimulation {
  public static inline var DETERMINISTIC:Int=0;
  public static inline var MUJOCO:Int=1;
  public final world:RobotWorld;
  /** Supply selection belongs to the cell, while its authored mission stays unchanged. */
  public var virtualWelder:Bool = false;
  public var virtualNutrunner:Bool = false;
  var virtualDevicePrepared:Bool = false;
  public var welderSupplyFactory:Null<processkit.simulation.SimulatedWelder -> processkit.simulation.SimulationWelderSupply> = null;
  public var appliedRevision(default, null):Int = 0;
  public var appliedDocumentRevision(default, null):Int = -1;
  public var appliedEnvironmentRevision(default, null):Int = -1;
  public var backend(default,null):Int;
  public var appliedBackend(default,null):Int=-1;
  public var timestep(default,null):Float=0.01;
  public var appliedTimestep(default,null):Float=-1.0;
  public var error(default, null):Null<String> = null;
  public var collisionWarnings(default, null):Array<String> = [];
  var space:Null<SimulationSpace> = null;
  var simulation:Null<Simulation> = null;
  var simulatedIds:Array<String> = [];
  var ownedRobots:Array<Robot> = [];
  var detachedPreparedRobots:Null<Array<Robot>>;
  var preparationOnly = false;
  var simulatedLinks:Array<Array<String>> = [];
  var simulatedObjects:Array<{id:String,object:SimObject}> = [];
  var motions:Null<RobotMotionPlayer> = null;
  /** The simulated tools on the session's robots. */
  var tools:Null<SimulatedTools> = null;
  /** The battery model of a CAD mobile power system, when present. */
  var power:Null<SimulatedMobilePower> = null;
  var cnc:Null<CncProgramPlayer> = null;
  var hobbing:Null<HobbingProgramPlayer> = null;
  /** Robot zone monitors own native collision worlds; they go with their simulation. */
  var zoneMonitors:Array<SimulatedRobotZoneAdapter> = [];
  var print:Null<PrintProgramPlayer> = null;
  var beltVisuals:Array<BeltVisuals> = [];
  var machineProgram:Null<MachineProgramPlayer> = null;
  var digitalProgram:Null<MachineDigitalProgramPlayer> = null;
  var pressBrake:Null<PressBrakeControllerPlayer> = null;
  /** The project's wheeled assembly's drive, when it has one. */
  var mobile:Null<robotkit.mobile.MobileBase> = null;
  /** One program per robot controller, run side by side and coordinated only through the cell's wired signals. */
  var missions:Array<MissionPlayer> = [];
  var cellSignals:Null<CellSignals> = null;
  /** The weld metal the mission's welds lay, when it welds. */
  var beads:Null<WeldBeads> = null;
  var workforce:Null<HumanWorkforce> = null;
  /** Everything that follows the session's lifecycle, in the order it is fed. */
  var members:Array<SessionMember> = [];
  var assemblyParts:Array<AssemblyVisual> = [];
  var running:Bool = false;
  /** Desktop playback owns its ticks on a worker; headless callers keep explicit stepping. */
  public var backgroundPacing:Bool = false;
  final playbackMutex = new Mutex();
  var playbackActive = false;
  var retirementFrame:Null<ApplicationPresentationSnapshot>;
  var playbackStop = false;
  var playbackCommands:Array<Void->Void> = [];
  var playbackFailure:Null<String>;
  var playbackFrame:Null<ApplicationPresentationSnapshot>;
  var presentedPlaybackRevision = -1;
  #if !wasm
  var playbackDone:Null<Lock>;
  #end
  /** Wall-clock stamp of the previous pump, or negative when pacing restarts. */
  var pumpStamp:Float = -1.0;
  /** Most ticks one pump runs; a slower owner loses simulation time instead of bursting. */
  static inline var MAX_TICKS_PER_PUMP:Int = 5;
  var presentAssemblyPhysics:Bool = false;
  var presentationEpoch:Int = 0;
  final participants:Array<SessionParticipant> = [];
  var participantRevision:Int = 0;
  var appliedParticipantRevision:Int = -1;

  public function new(world:RobotWorld,?backend:Int=DETERMINISTIC) {
    this.world=world;this.backend=DETERMINISTIC;setBackend(backend);
  }

  public function setBackend(value:Int):Bool {
    if(value!=DETERMINISTIC&&value!=MUJOCO)throw "Unsupported simulation backend";
    if(backend==value)return false;backend=value;return true;
  }
  public function backendName():String return backend==MUJOCO?"MuJoCo":"Deterministic";
  public function userBackendName():String return backend==MUJOCO?"MuJoCo":"Test backend";
  public function setTimestep(value:Float):Bool {
    if(!Math.isFinite(value)||value<=0)throw "Simulation timestep must be finite and positive";
    if(timestep==value)return false;timestep=value;return true;
  }

  public function pending(configuration:SensorConfiguration, scene:EditorScene):Bool
    return appliedDocumentRevision != configuration.revision() ||
      appliedEnvironmentRevision != scene.environmentRevision||appliedBackend!=backend||appliedTimestep!=timestep||
      appliedParticipantRevision != participantRevision;

  /** Adds a participant to every session built from the next rebuild on. */
  public function addParticipant(participant:SessionParticipant):Void {
    if (participants.indexOf(participant) >= 0) return;
    participants.push(participant);
    participantRevision++;
    refreshMembers();
  }

  /**
   * Removes a participant from the next rebuild on. It leaves the live session
   * now; a body it placed there stays, unmoving, until that rebuild.
   */
  public function removeParticipant(participant:SessionParticipant):Void {
    if (!participants.remove(participant)) return;
    participant.leave();
    participantRevision++;
    refreshMembers();
  }

  /** Rebuilds the list every tick walks, so the tick itself allocates nothing. */
  function refreshMembers():Void {
    members = [];
    if (cellSignals != null) members.push(cellSignals);
    if (motions != null) members.push(motions);
    if (tools != null) members.push(tools);
    if (power != null) members.push(power);
    if (cnc != null) members.push(cnc);
    if (hobbing != null) members.push(hobbing);
    if (print != null) members.push(print);
    if (machineProgram != null) members.push(machineProgram);
    if (digitalProgram != null) members.push(digitalProgram);
    if (pressBrake != null) members.push(pressBrake);
    for (program in missions) members.push(program);
    if (beads != null) members.push(beads);
    if (workforce != null) members.push(workforce);
    for (participant in participants) members.push(participant);
  }

  /** Builds the complete candidate before changing any live world adapter. */
  public function rebuild(configuration:SensorConfiguration, scene:EditorScene,
      ?session:ProjectDocumentSession, publishVisuals:Bool = true, ?control:ProjectLoadControl):Bool {
    var candidateSpace:Null<SimulationSpace> = null;
    var environmentTopologyOpen = false;
    var candidate:Null<Simulation> = null;
    var candidateRobots:Array<SimulatedRobot> = [];
    var candidateLinks:Array<Array<String>> = [];
    var candidateRobotModels:Array<robotkit.model.RobotModel> = [];
    var candidateRuntimes:Array<RobotRuntime> = [];
    var candidateBlueprints:Array<RobotRuntimeBlueprint> = [];
    var candidateZones:Array<SimulatedRobotZoneAdapter> = [];
    var candidateObjects:Array<{id:String,object:SimObject}> = [];
    var candidateWorkforce:Null<HumanWorkforce> = null;
    var candidateAssemblyParts:Array<AssemblyVisual> = [];
    var candidateWarnings:Array<String> = [];
    var candidateCnc:Null<CncProgramPlayer> = null;
    var candidateHobbing:Null<HobbingProgramPlayer> = null;
    var candidatePrint:Null<PrintProgramPlayer> = null;
    var pendingCnc:Null<{job:CncJob, robot:AssemblyRobot,
      input:Null<String->Null<EventValue>>, interfaceManaged:Bool}> = null;
    var candidateBelts:Array<BeltVisuals> = [];
    var candidateProgram:Null<MachineProgramPlayer> = null;
    var candidateDigital:Null<MachineDigitalProgramPlayer> = null;
    var candidatePress:Null<PressBrakeControllerPlayer> = null;
    var candidateMobile:Null<robotkit.mobile.MobileBase> = null;
    var candidatePower:Null<SimulatedMobilePower> = null;
    var candidateMissions:Array<MissionPlayer> = [];
    var candidateSignals:Null<CellSignals> = null;
    var candidateBeads:Null<WeldBeads> = null;
    var candidateAssembly:Null<AssemblyRobot> = null;
    var assemblyIndex = -1;
		var candidateByController = new Map<String, AssemblyRobot>();
		var indexByController = new Map<String, Int>();
    var heldBoxes:Array<MissionPlayer.FloorObstacle> = [];
    try {
      if (control != null) { control.throwIfCancelled(); control.phase("Building controllers"); }
      var models = configuration.robotModels();
      var hasWorkers = false;
      for (record in scene.records()) if (record.type == "human-worker") { hasWorkers = true; break; }
      var assembly = session == null ? null : session.projectAssemblyDefinition;
      var restContactGroup = haxe.Int64.ofInt(1); // This session contains one generated assembly.
      if (models.length == 0 && assembly == null && participants.length == 0 &&
          !hasWorkers) throw "Nothing to simulate";
      for (configured in models) {
        var id=configured.id;
        var existing = world.robot(id);
        if (existing != null && simulatedIds.indexOf(id) < 0)
          throw 'Robot "$id" is remote and read-only';
      }
      var hobbingConfig=session==null||session.machineMotion==null?null:session.machineMotion.hobbing;
      if(hobbingConfig!=null)timestep=1/hobbingConfig.controllerHz;
      var createdSpace = SimulationSpace.create(backend, timestep);
      candidateSpace = createdSpace;
      candidate = Simulation.inSpace(createdSpace);
      if (session != null && session.signals.length > 0) {
        candidateSignals = new CellSignals(session.signals,
          () -> createdSpace.session.simulationTime(),
          (controller, channel) -> {
            var source = candidateByController.get(controller);
            if (source == null) throw 'Signal source controller "$controller" has no runtime';
            return switch source.runtime.channelValue(channel) {
              case Digital(on): on;
              case _: throw 'Signal "$channel" is not a digital output';
            };
          },
          (controller, channel, on, sequence, stamp) -> {
            var target = candidateByController.get(controller);
            if (target == null) throw 'Signal target controller "$controller" has no runtime';
            target.runtime.publishSensorFrame(channel, [on ? 1.0 : 0.0], sequence, stamp, "cell.controller.tick");
          },
          (controller, channel, on) -> {
            var source = candidateByController.get(controller);
            if (source == null) throw 'Signal source controller "$controller" has no runtime';
            source.runtime.writeDigitalOutput(channel, on);
          });
      }
      for (index in 0...models.length) {
        if (control != null) control.throwIfCancelled();
        var editable=models[index];
        var blueprint = RobotRuntimeCompiler.compile(editable.model, editable.profile, appliedRevision + 1);
        var runtime = candidate.addRobotAtPose(blueprint, editable.position, editable.rotation);
        var id = editable.id;
        candidateRobots.push(new SimulatedRobot(id, runtime, editable.model.name,
          [for (link in editable.model.links) link.id], [for (joint in editable.model.joints) joint.id]));
        candidateLinks.push([for (link in editable.model.links) link.id]);
        candidateRobotModels.push(editable.model);
        candidateRuntimes.push(runtime);
        candidateBlueprints.push(blueprint);
      }
      var candidateMotions = session == null ? [] : session.robotMotions;
      if (assembly != null && session != null) {
        var controls:Array<Null<materia.project.SceneArtifact.SceneArtifactCellController>> =
          session.controllers.length == 0 ? [null] : [for (control in session.controllers) control];
        for (control in controls) {
          var id = AssemblyRobot.idFor(assembly,
            control == null || session.controllers.length == 1 ? null : control.id);
          if (world.robot(id) != null && simulatedIds.indexOf(id) < 0)
            throw 'Robot "$id" is remote and read-only';
          var robotIndex = candidateRobots.length;
			var job = session.cncJob;
			var ownsJob = job != null && (control == null || job.controllerId == control.id);
			var interfaceManaged = ownsJob && control != null && control.kind == "cnc" &&
				session.controllers.length > 1;
          var pressConfig = session.machineMotion == null ? null : session.machineMotion.pressBrake;
          var ownsPress = pressConfig != null && (control == null || control.id == pressConfig.controllerId);
          if (ownsPress && ownsJob) throw "One controller cannot own both a press and a CNC job";
          var processChannels = ownsPress ? processkit.pressbrake.PressBrakeChannels.declarations()
            : ownsJob ? CncProgramPlayer.processChannels(job, interfaceManaged) : null;
          var built = AssemblyRobot.add(candidate, scene, session, assembly, backend == MUJOCO,
            appliedRevision + 1, robotIndex, processChannels, virtualWelder, control, restContactGroup, virtualNutrunner);
          candidateRobots.push(built.robot);
          candidateLinks.push([for (link in built.model.links) link.id]);
          candidateRobotModels.push(built.model);
          if (ownsPress) {
            if (candidatePress != null || pressConfig == null) throw "Press controller execution ownership is ambiguous";
            candidatePress = new PressBrakeControllerPlayer(pressConfig, built, createdSpace.session);
          }
          candidateRuntimes.push(built.runtime);
          candidateBlueprints.push(built.blueprint);
          for (part in built.parts) candidateAssemblyParts.push(prepareAssemblyVisual(part));
          for (warning in built.warnings) candidateWarnings.push(warning);
          if (control != null) {
            candidateByController.set(control.id, built);
            indexByController.set(control.id, robotIndex);
          }
          if (built.mobile != null) candidateMobile = built.mobile;
          if (candidateAssembly == null || control == null ||
              (session.missions != null && session.missions[0].controllerId == control.id)) {
            candidateAssembly = built;
            assemblyIndex = robotIndex;
          }
		  if (job != null && ownsJob) {
		    if (pendingCnc != null) throw "A project simulation can run only one generated CNC job";
		    pendingCnc = {job: job, robot: built,
		      input: candidateSignals == null || control == null ? null : candidateSignals.inputFor(control.id),
		      interfaceManaged: interfaceManaged};
		  }
          // A bad print program fails the rebuild here, before anything live changes.
          var printJob = session.printJob;
          if (printJob != null && built.heaters != null) {
            if (ownsJob) throw "A controller cannot run a CNC job and a print job together";
            if (candidatePrint != null) throw "A project simulation can run only one print job";
            candidatePrint = new PrintProgramPlayer(printJob, built, candidate, session, createdSpace.session);
          }
          var machine = session.machineMotion;
          if (machine != null) {
            var jointIds = [for (joint in built.model.joints) joint.id];
            var ownedBelts = [for (belt in machine.belts) if (jointIds.indexOf(belt.driverJoint) >= 0) belt];
            if (ownedBelts.length > 0) candidateBelts.push(new BeltVisuals(session, built, candidate, ownedBelts));
            if (machine.program != null && machine.program.axes.length > 0 &&
                jointIds.indexOf(machine.program.axes[0]) >= 0) {
              if (ownsJob || ownsPress) throw "A controller cannot share CNC or press execution with an axis program";
              candidateProgram = new MachineProgramPlayer(machine.program, built, createdSpace.session,
                machine.virtualDevice == true, candidateSignals == null || control == null ? null : candidateSignals.inputFor(control.id));
            }
          }
          if (machine != null && machine.digitalProgram != null) {
            if (candidateDigital != null || session.controllers.length > 1 || job != null ||
                machine.program != null || session.missions != null || built.mobile != null)
              throw "Machine digital program needs exclusive assembly execution ownership";
            candidateDigital = new MachineDigitalProgramPlayer(machine.digitalProgram, built);
          }
        }
        if (session.machineMotion != null && session.machineMotion.program != null && candidateProgram == null)
          throw "Machine axis program has no controller owning its axes";
        if (session.machineMotion != null && session.machineMotion.pressBrake != null && candidatePress == null)
          throw "Press program has no controller owning its axes";
      }
      var resolvedMotions = RobotMotionPlayer.resolve(candidateMotions,
        [for (robot in candidateRobots) robot.id()], candidateRobotModels);
      if (candidateDigital != null && candidateAssembly != null)
        for (track in resolvedMotions) if (track.robotId == candidateAssembly.robot.id())
          throw "Machine digital program cannot share its assembly with a robot motion track";
      if (candidatePress != null)
        for (track in resolvedMotions) if (track.robotId == candidatePress.robotId())
          throw "Press control cannot share its controller with a robot motion track";
      var environmentRecords = scene.records();
      if (control != null) { control.throwIfCancelled(); control.phase("Building environment"); }
      environmentRecords.sort(function(a, b) return Reflect.compare(a.id, b.id));
      var hasPneumaticActuator = candidateAssembly != null &&
        hasPneumaticCylinder(candidateAssembly.blueprint);
      if (!hasPneumaticActuator && session != null) for (control in session.controllers) {
        var built = candidateByController.get(control.id);
        if (built != null && hasPneumaticCylinder(built.blueprint)) hasPneumaticActuator = true;
      }
      var pneumaticWorkpieceSurface = hasPneumaticActuator
        ? SimulationContactSurfaces.environmentPart() : null;
      // A machining stock has ordinary layer collisions disabled so it can enter the cutter. Keep
      // it as a real free body that still rests on fixtures and contacts its vise; cutter contact is
      // excluded explicitly for the CNC link below.
      var unlayeredWorkpieceSurface = hasPneumaticActuator
        ? SimulationContactSurfaces.environmentPart(SimContactFilter.PairsAndEnvironment)
        : new SimContactSurface(null, 0, 0, 0, SimContactFilter.PairsAndEnvironment);
      var assemblyOwned = new Map<String, Bool>();
      for (part in candidateAssemblyParts) assemblyOwned.set(part.id, true);
      createdSpace.session.beginTopologyUpdate();
      environmentTopologyOpen = true;
      for (object in environmentRecords) if ((object.collisionEnabled || object.dynamicBody) && !assemblyOwned.exists(object.id)) {
        if (control != null) control.throwIfCancelled();
        var centerX=object.x,centerY=object.y,centerZ=object.z;
        var halfX=object.width/2.0,halfY=object.height/2.0,halfZ=object.depth/2.0;
        var shape = SimShape.box(halfX, halfY, halfZ);
        var inertial:Null<nativekit.sim.SimInertialProperties> = null;
        if(scene.isCadPart(object.id)){
          var bounds=scene.cadSession(object.id).collisionBounds;
          if(bounds==null)throw "CAD collision bounds are unavailable for: "+object.id;
          var offset = rotateOffset(bounds.center.x, bounds.center.y, bounds.center.z, object.rotation);
          centerX+=offset[0];centerY+=offset[1];centerZ+=offset[2];
          halfX=bounds.halfExtents.x;halfY=bounds.halfExtents.y;halfZ=bounds.halfExtents.z;
          var hull:Null<Array<Float>> = null;
          var definition = session == null ? null : session.projectAssemblyDefinition;
          var physical = session == null ? null : session.projectPhysical;
          if (session != null && definition != null && physical != null) for (occurrence in definition.occurrences)
            if (object.id == "project:" + occurrence.id) for (part in physical.parts)
              if (part.id == occurrence.definition) {
                var vertices = part.collisionHull;
                if (vertices == null) throw "Environment part has no physical collision hull: " + object.id;
                var centre = session.assemblyPreviewCenter(occurrence.definition);
                if (centre == null) throw "Environment part has no preview centre: " + object.id;
                hull = [for (i in 0...vertices.length) (vertices[i]-centre[i%3])*physical.metresPerUnit];
              }
          if (hull == null) {
            var derived = scene.cadSession(object.id).collisionHull();
            hull = derived.vertices;
            if (derived.warning != null) candidateWarnings.push(object.id + ": " + derived.warning);
          }
          shape = SimShape.convex(hull);
        }
        // Generated previews are already centred and have no editable CAD
        // session. Their authored pieces are still component facts.
        var authored = projectEnvironmentShape(session,object.id);
        if (authored != null) shape = authored;
        // A free project part turns with its own CAD inertia, not a solver's nominal one.
        if (object.dynamicBody) inertial = projectInertial(session, object.id, object.mass);
        var rotation=object.rotation==null?[0.0,0.0,0.0,1.0]:object.rotation;
        var surface = object.collisionEnabled
          ? (object.dynamicBody ? pneumaticWorkpieceSurface : null)
          : unlayeredWorkpieceSurface;
        var created=createdSpace.session.createObject(
          object.dynamicBody?MotionType.Dynamic:MotionType.Static,shape,
          new SimPose(centerX,centerY,centerZ,rotation[0],rotation[1],rotation[2],rotation[3]),
          object.dynamicBody?object.mass:0.0,
          !object.dynamicBody && assembly != null && [for (part in assembly.occurrences) if (object.id == "project:" + part.id) part].length == 1 ? restContactGroup : null,
          surface, inertial);
        candidateObjects.push({id:object.id,object:created});
        if (!object.dynamicBody) heldBoxes.push({id: object.id, x: centerX, y: centerY, z: centerZ,
          halfX: halfX, halfY: halfY, halfZ: halfZ,
          yaw: Math.atan2(2 * (rotation[3] * rotation[2] + rotation[0] * rotation[1]),
            1 - 2 * (rotation[1] * rotation[1] + rotation[2] * rotation[2]))});
      }
      environmentTopologyOpen = false;
      createdSpace.session.endTopologyUpdate();
      // The robot's tools act on the free objects after each step, as their channels say.
      var candidateTools = new SimulatedTools([for (entry in candidateObjects) {id: entry.id, object: entry.object}], candidate);
      if (control != null) { control.throwIfCancelled(); control.phase("Preparing programs and mission"); }
      var cncToBuild = pendingCnc;
      if (cncToBuild != null) {
        var cncParts = new SimulationAssemblyParts(candidate, [cncToBuild.robot],
          candidateTools.objectsByScene(), session);
        candidateCnc = new CncProgramPlayer(cncToBuild.job, cncToBuild.robot, candidate,
          session, createdSpace.session, cncParts, cncToBuild.input, null, cncToBuild.interfaceManaged);
        if (cncToBuild.interfaceManaged) {
          var cutterOccurrence = cncToBuild.job.toolPart == null
            ? cncToBuild.job.spindle : cncToBuild.job.toolPart;
          var cutter = cncToBuild.robot.part("project:" + cutterOccurrence);
          for (stock in cncToBuild.job.stocks)
            cncParts.get("project:" + stock).excludeContactWith(cutter.robotIndex, cutter.linkIndex);
        }
      }
      if(hobbingConfig!=null){
        if(candidateAssembly==null||session==null)throw "Hobbing needs its simulated assembly";
        candidateHobbing=new HobbingProgramPlayer(hobbingConfig,candidateAssembly,candidate,createdSpace.session,session);
      }
      var freeObjects = [for (entry in candidateObjects) entry.object];
      for (index in 0...candidateBlueprints.length) {
        var hasPresenceInput = false;
        for (sensor in candidateBlueprints[index].externalSensorLayout())
          if (sensor.kind == "presence_input") hasPresenceInput = true;
        if (hasPresenceInput) candidate.addStepObserver(new SimulatedPresenceInputAdapter(candidate,
          candidateRuntimes[index], candidateBlueprints[index], index, freeObjects));
        var hasZone = false;
        for (sensor in candidateBlueprints[index].externalSensorLayout()) if (sensor.kind == "zone_input") hasZone = true;
        // A machine's robot zone watches every other robot's simulated links and the parts they carry.
        if (hasZone) {
          var zone = new SimulatedRobotZoneAdapter(candidate, candidateRuntimes[index],
            candidateBlueprints[index], index, [for (other in 0...candidateBlueprints.length) if (other != index)
              {robotIndex: other, shapes: candidateBlueprints[other].linkCollisionShapes}], freeObjects,
            robotkit.collision.CollisionClearance.MARGIN, () -> new collisionkit.native.NativeCollisionWorld());
          candidateZones.push(zone);
          candidate.addStepObserver(zone);
        }
      }
      var driverByChannel = new Map<String, processkit.simulation.SimulatedScrewdriver>();
      var welderByChannel = new Map<String, processkit.simulation.SimulatedWelder>();
      if (session != null && candidateAssembly != null) for (tool in session.robotTools) {
        var toolRobot = candidateAssembly;
        var toolIndex = assemblyIndex;
        if (session.controllers.length > 0) for (control in session.controllers) {
          var bound = candidateByController.get(control.id);
          if (bound != null && bound.toolFrames.exists(tool.contact.occurrence)) {
            toolRobot = bound;
            var index = indexByController.get(control.id);
            if (index == null) throw 'Controller "${control.id}" has no simulated robot';
            toolIndex = index;
          }
        }
        if (tool.kind == "screwdriver") {
          var driver = candidateTools.screwdriverFor(candidate,toolRobot,toolIndex,tool,session);
          if (virtualNutrunner) driver.supply = new processkit.simulation.VirtualNutrunnerSupply(candidate,toolRobot.runtime,
            toolIndex,tool.sensor,toolRobot.blueprint,driver.channels,@:privateAccess driver.feedJoint,@:privateAccess driver.spindleJoint);
          candidate.addStepObserver(candidateTools.addScrewdriver(driver));
          driverByChannel.set(tool.channel, driver);
          continue;
        }
        if (tool.kind == "torch") {
          var welder = candidateTools.welderFor(candidate, toolRobot, toolIndex, tool, session);
          if (virtualWelder) welder.supply = new processkit.simulation.VirtualWelderSupply(candidate,
            toolRobot.runtime, toolIndex, tool.sensor);
          else if (welderSupplyFactory != null) welder.supply = welderSupplyFactory(welder);
          candidate.addStepObserver(candidateTools.addWelder(welder));
          welderByChannel.set(tool.channel, welder);
          continue;
        }
        if (tool.kind == "gripper") {
          if (tool.sensor == null) throw 'Gripper tool "${tool.channel}" has no jaw switch';
          candidate.addStepObserver(candidateTools.addGripper(new robotkit.runtime.SimulatedPneumaticGripper(
            toolRobot.runtime, toolRobot.blueprint, tool.channel, tool.sensor)));
          continue;
        }
        var carrier = toolRobot.part("project:" + tool.contact.occurrence);
        candidate.addStepObserver(candidateTools.add(new robotkit.runtime.SimulatedSuctionTool(candidate,
          toolRobot.runtime, toolIndex, carrier.linkIndex, tool.channel, freeObjects, tool.sensor)));
      }
      var programs = session == null || session.missions == null ? [] : session.missions;
      var cellTools = session == null ? [] : session.robotTools;
      for (work in programs) {
        // Each program runs on its own controller's robot with that controller's tools, feeders and cell I/O.
        var programRobot = work.controllerId == null ? candidateAssembly : candidateByController.get(work.controllerId);
        var programIndex = work.controllerId == null ? assemblyIndex : indexByController.get(work.controllerId);
        if (programRobot == null || programIndex == null) throw 'A mission needs the robot of controller "${work.controllerId}"';
        var robotIndex:Int = programIndex;
        var ownTools = [for (tool in cellTools) if (tool.controller == work.controllerId) tool];
        var drivers:Array<processkit.simulation.SimulatedScrewdriver> = [];
        var welders:Array<processkit.simulation.SimulatedWelder> = [];
        for (tool in ownTools) {
          var driver = driverByChannel.get(tool.channel), welder = welderByChannel.get(tool.channel);
          if (tool.kind == "screwdriver") {
            if (driver == null) throw 'Screwdriver "${tool.channel}" has no simulated driver';
            drivers.push(driver);
          }
          if (tool.kind == "torch") {
            if (welder == null) throw 'Torch "${tool.channel}" has no simulated welder';
            welders.push(welder);
          }
        }
        var feeders:Array<SimulatedScrewFeeder> = [], deliveries:Array<SimulatedBlowFeed> = [];
        if (work.screwFeeders != null) {
          if (drivers.length != 1) throw "Screw feeding needs the program's one mounted screwdriver";
          var feederParts = new SimulationAssemblyParts(candidate,[programRobot],candidateTools.objectsByScene(),session);
          for (definition in work.screwFeeders) {
            var feeder = new SimulatedScrewFeeder(definition,candidate,programRobot.runtime,feederParts,drivers[0]);
            candidateTools.screwFeeders.push(feeder); feeders.push(feeder); candidate.addStepObserver(feeder);
          }
        }
        if (work.blowFeeds != null) {
          if (drivers.length != 1) throw "Blow feed needs the program's one mounted screwdriver";
          var parts = new SimulationAssemblyParts(candidate,[programRobot],candidateTools.objectsByScene(),session);
          for (definition in work.blowFeeds) {
            var delivery = new SimulatedBlowFeed(definition,programRobot.runtime,parts,drivers[0]);
            candidateTools.blowFeeds.push(delivery); deliveries.push(delivery); candidate.addStepObserver(delivery);
          }
        }
        var pairedScrews:Map<String,Bool> = [];
        for (step in work.steps) if (step.kind == "screw") {
          var job = materia.project.SceneScrewInvocation.job(step.screw,work.steps);
          if (job == null || drivers.length != 1)
            throw "A screw mission needs its named screw and one declared screwdriver";
          var identities = [job.screw];
          var recovery = job.recovery;
          if (recovery != null) identities.push(recovery.replacement.screw);
          var holder = drivers[0].holding;
          // Only declared bit/screw couplings exclude mutual contact; fixture contacts remain physical.
          for (identity in identities) {
            if (pairedScrews.exists(identity)) continue;
            var screws = [for (object in candidateTools.objectsByScene()) if (object.id == "project:"+identity) object.object];
            if (screws.length != 1) throw 'Fastening "${job.fastening}" has no unique independent screw body "$identity"';
            candidate.excludeContactPair(holder.robotIndex,holder.linkIndex,screws[0]);
            pairedScrews.set(identity,true);
          }
        }
        var welds = [for (step in work.steps) if (step.kind == "weld") step].length > 0;
        var mobileProfile = session == null ? null : session.mobileBase;
        if (welds && mobileProfile != null && mobileProfile.energy != null) {
          if (welders.length != 1) throw "A powered mobile welding mission needs one simulated welder";
          candidatePower = new SimulatedMobilePower(mobileProfile.energy, mobileProfile, programRobot,
            candidate, robotIndex, timestep, welders[0]);
        }
        var player = new MissionPlayer(work, programRobot, candidate, robotIndex, timestep, heldBoxes,
          candidateTools.objectsByScene(), session, [for (id in candidateByController.keys()) candidateByController.get(id)],
          candidateSignals == null || work.controllerId == null ? null : candidateSignals.inputFor(work.controllerId),
          candidateSignals == null || work.controllerId == null ? null : candidateSignals.outputFor(work.controllerId), null,
          virtualWelder || virtualNutrunner ? MissionPlayer.ARM_ACCELERATION * robotkit.runtime.VirtualServoOptions.ACCELERATION_SCALE
            : MissionPlayer.ARM_ACCELERATION, candidatePower, drivers, feeders, deliveries);
        candidateMissions.push(player);
        // A program that welds lays the weld metal as its welder deposits it.
        if (welds) {
          var torch = [for (tool in ownTools) if (tool.kind == "torch") tool][0].torch;
          if (welders.length != 1 || torch == null) throw "A mission that welds needs the robot's one simulated welder";
          if (candidateBeads != null) throw "Only one program in a cell may weld";
          candidateBeads = new WeldBeads(player, welders[0], candidate, scene,
            timestep, torch.wireDiameterMm, torch.depositionEfficiency);
        }
      }

      var objectsById:Map<String, SimObject> = new Map();
      for (entry in candidateObjects) objectsById.set(entry.id, entry.object);
      candidateWorkforce = HumanWorkforce.build(createdSpace.session, scene, environmentRecords,
        objectsById, [for (robot in candidateRobots) robot.id()], candidateRobotModels, candidate);
      for (warning in candidateWorkforce.warnings) candidateWarnings.push(warning);

      var previousSimulation = simulation;
      if (control != null) control.throwIfCancelled();
      var previousSpace = space;
      // Finish the old mission's planning before retiring its native worlds.
      for (program in missions) program.beforeReset();
      var previousWorkforce = workforce;
      var previousIds = simulatedIds.copy();
      var previousRobots:Array<Robot> = [];
      for (id in previousIds) {
        var detached = world.detach(id);
        if (detached != null) previousRobots.push(detached);
      }
      var attached:Array<String> = [];
      try {
        for (robot in candidateRobots) { world.attach(robot); attached.push(robot.id()); }
      } catch (failure:Dynamic) {
        for (id in attached) world.detach(id);
        for (robot in previousRobots) world.attach(robot);
        throw failure;
      }
      // Nothing below can fail: participants move to the new session, which
      // then starts if the old one was running.
      for (participant in participants) participant.join(createdSpace.session);
      pumpStamp = -1.0;
      simulation = candidate;
      virtualDevicePrepared = false;
      space = candidateSpace;
      simulatedIds = [for (robot in candidateRobots) robot.id()];
      ownedRobots = [for (robot in candidateRobots) (robot:Robot)];
      simulatedLinks = candidateLinks;
      simulatedObjects = candidateObjects;
      motions = resolvedMotions.length == 0 ? null :
        new RobotMotionPlayer(world, createdSpace.session, resolvedMotions);
      tools = candidateTools;
      power = candidatePower;
      workforce = candidateWorkforce;
      if (cnc != null) cnc.dispose();
      if (hobbing != null) hobbing.dispose();
      if (print != null) print.dispose();
      for (visual in beltVisuals) visual.reset();
      beltVisuals = candidateBelts;
      if (machineProgram != null) machineProgram.beforeReset();
      machineProgram = candidateProgram;
      if (digitalProgram != null) digitalProgram.beforeReset();
      digitalProgram = candidateDigital;
      if (pressBrake != null) pressBrake.beforeReset();
      pressBrake = candidatePress;
      cnc = candidateCnc;
      hobbing = candidateHobbing;
      var previousZones = zoneMonitors;
      zoneMonitors = candidateZones;
      print = candidatePrint;
      mobile = candidateMobile;
      missions = candidateMissions;
      cellSignals = candidateSignals;
      if (beads != null) beads.dispose();
      beads = candidateBeads;
      refreshMembers();
      assemblyParts = candidateAssemblyParts;
      appliedRevision++;
      appliedDocumentRevision = configuration.revision();
      appliedEnvironmentRevision = scene.environmentRevision;
      appliedBackend=backend;
      appliedTimestep=timestep;
      appliedParticipantRevision = participantRevision;
      error = null;
      collisionWarnings = candidateWarnings;
      presentAssemblyPhysics = running;
      presentationEpoch++;
      if (publishVisuals) scene.setWorkerVisualsVisible(false);
      if (previousWorkforce != null) previousWorkforce.dispose();
      releaseSpace(previousSpace, previousSimulation);
      for (zone in previousZones) zone.dispose();
      for (robot in previousRobots) robot.close();
      return true;
    } catch (failure:Dynamic) {
      error = Std.string(failure);
      if (environmentTopologyOpen && candidateSpace != null)
        try candidateSpace.session.endTopologyUpdate() catch (_:Dynamic) {}
      if (candidateHobbing != null) candidateHobbing.dispose();
      if (candidateWorkforce != null) candidateWorkforce.dispose();
      if (candidateSpace != null && candidateSpace != space)
        releaseSpace(candidateSpace, candidate);
      for (zone in candidateZones) zone.dispose();
      return false;
    }
  }

  /** Custom participants and character visuals retain their existing owner-thread lifecycle. */
  public function supportsBackgroundBuild(scene:EditorScene):Bool {
    if (participants.length > 0) return false;
    for (record in scene.records()) if (record.type == "human-worker") return false;
    return true;
  }

  /** Prepare in a separate world, preserving the live simulation until installation. */
  public function preparationTarget():ApplicationSimulation {
    var target = new ApplicationSimulation(new RobotWorld(), backend);
    target.setTimestep(timestep);
    target.appliedRevision = appliedRevision;
    target.preparationOnly = true;
    target.virtualWelder = virtualWelder;
    target.virtualNutrunner = virtualNutrunner;
    target.welderSupplyFactory = welderSupplyFactory;
    return target;
  }

  /** Release logical-world ownership on the builder before publishing its result. */
  public function detachPreparedRobots():Void {
    detachedPreparedRobots = [];
    for (id in simulatedIds) {
      var robot = world.detach(id);
      if (robot == null) throw 'Prepared robot "$id" is unavailable';
      detachedPreparedRobots.push(robot);
    }
    world.close();
  }

  /** Called on the UI thread after preparation has finished; transport keeps this object's identity. */
  public function installPrepared(prepared:ApplicationSimulation, scene:EditorScene):Void {
    for (id in prepared.simulatedIds)
      if (world.robot(id) != null && simulatedIds.indexOf(id) < 0)
        throw 'Robot "$id" is remote and read-only';
    var readySpace = prepared.space;
    if (readySpace == null) throw "Prepared simulation has no native world";
    readySpace.session.stop(); // Return the prepared native world to this thread.
    var robots = prepared.detachedPreparedRobots;
    if (robots == null || robots.length != prepared.simulatedIds.length)
      throw "Prepared robots were not released by their builder";
    clear();
    for (robot in robots) world.attach(robot);
    space = prepared.space;
    simulation = prepared.simulation;
    simulatedIds = prepared.simulatedIds;
    ownedRobots = prepared.ownedRobots;
    simulatedLinks = prepared.simulatedLinks;
    simulatedObjects = prepared.simulatedObjects;
    motions = prepared.motions == null ? null : prepared.motions.inWorld(world);
    tools = prepared.tools;
    power = prepared.power;
    cnc = prepared.cnc;
    hobbing = prepared.hobbing;
    zoneMonitors = prepared.zoneMonitors;
    print = prepared.print;
    beltVisuals = prepared.beltVisuals;
    machineProgram = prepared.machineProgram;
    digitalProgram = prepared.digitalProgram;
    pressBrake = prepared.pressBrake;
    mobile = prepared.mobile;
    missions = prepared.missions;
    cellSignals = prepared.cellSignals;
    beads = prepared.beads;
    workforce = prepared.workforce;
    assemblyParts = prepared.assemblyParts;
    appliedRevision = prepared.appliedRevision;
    appliedDocumentRevision = prepared.appliedDocumentRevision;
    appliedEnvironmentRevision = prepared.appliedEnvironmentRevision;
    appliedBackend = prepared.appliedBackend;
    timestep = prepared.timestep;
    appliedTimestep = prepared.appliedTimestep;
    appliedParticipantRevision = participantRevision;
    collisionWarnings = prepared.collisionWarnings;
    virtualDevicePrepared = false;
    presentAssemblyPhysics = false;
    pumpStamp = -1.0;
    error = null;
    presentationEpoch++;
    refreshMembers();
    scene.setWorkerVisualsVisible(false);
    prepared.space = null;
    prepared.simulation = null;
    prepared.simulatedIds = [];
    prepared.detachedPreparedRobots = null;
  }

  /** Stops a space, removes its robots, then releases the space itself. */
  static function releaseSpace(released:Null<SimulationSpace>, robots:Null<Simulation>):Void {
    if (released != null) released.session.stop();
    if (robots != null) robots.dispose();
    if (released != null) released.dispose();
  }

  /**
   * A CAD part's mass properties in its free body's frame: the part frame, translated to the preview centre the body's
   * hull is built around. The tensor is centroidal, from the resolved kg m^2 inertia or the unit-density one, scaled
   * to the body's mass.
   */
  static function projectInertial(session:Null<ProjectDocumentSession>, id:String,
      mass:Float):Null<nativekit.sim.SimInertialProperties> {
    var definition = session == null ? null : session.projectAssemblyDefinition;
    var physical = session == null ? null : session.projectPhysical;
    if (session == null || definition == null || physical == null) return null;
    for (occurrence in definition.occurrences) if (id == "project:" + occurrence.id)
      for (part in physical.parts) if (part.id == occurrence.definition) {
        var centre = session.assemblyPreviewCenter(occurrence.definition);
        if (centre == null) throw "Dynamic project part has no preview centre: " + id;
        return cadInertial(part, centre, physical.metresPerUnit, mass);
      }
    return null;
  }

  static function cadInertial(part:cadbridge.AssemblySimulationBridge.AssemblyPhysicalPart, centre:Array<Float>,
      metresPerUnit:Float, mass:Float):nativekit.sim.SimInertialProperties {
    if (!Math.isFinite(mass) || mass <= 0) throw 'Dynamic CAD part "${part.id}" needs a positive mass';
    var center = [for (axis in 0...3) (part.centerOfMass[axis] - centre[axis]) * metresPerUnit];
    var resolved = part.physicalInertiaKgM2, resolvedMass = part.physicalMassKg;
    var tensor:Array<Float>;
    if (resolved != null && resolvedMass != null) tensor = [for (value in resolved) value * mass / resolvedMass];
    else {
      var baseMass = part.volume * part.density * Math.pow(metresPerUnit, 3);
      var factor = part.density * Math.pow(metresPerUnit, 5) * mass / baseMass;
      tensor = [for (value in part.inertia) value * factor];
    }
    return new nativekit.sim.SimInertialProperties(center, tensor);
  }

  public static function rotateOffset(x:Float, y:Float, z:Float, rotation:Null<Array<Float>>):Array<Float> {
    if (rotation == null) return [x, y, z];
    var qx = rotation[0], qy = rotation[1], qz = rotation[2], qw = rotation[3];
    var tx = 2 * (qy * z - qz * y), ty = 2 * (qz * x - qx * z), tz = 2 * (qx * y - qy * x);
    return [x + qw * tx + qy * tz - qz * ty,
      y + qw * ty + qz * tx - qx * tz,
      z + qw * tz + qx * ty - qy * tx];
  }

  static function prepareAssemblyVisual(part:AssemblyPart):AssemblyVisual {
    return {id:part.id, robotIndex:part.robotIndex, linkIndex:part.linkIndex,
      offset:AssemblyFrames.compose(part.offset,
        AssemblyFrames.translation(part.center[0], part.center[1], part.center[2]))};
  }

  /** Advances the simulation one tick. Read what it produced with `snapshot()`. */
  public function step(?timestampNs:Int64):Void {
    var active = space;
    if (active == null) throw "Apply the pending simulation configuration first";
    if (running) throw "Stop realtime simulation before deterministic stepping";
    if(hobbing!=null)hobbing.advanceHandling(active.session.fixedTimestep());
    feedMembers();
    presentMembers();
    active.session.step(timestampNs == null ? Int64.ofInt(0) : timestampNs);
    presentAssemblyPhysics = true;
  }

  /** Every robot's latest observation. Built on request: stepping does not need it. */
  public function snapshot():WorldSnapshot return world.snapshot();
  public function start():Void {
    var active = space;
    if (active == null) throw "Apply the pending simulation configuration first";
    if (!running) { pumpStamp = -1.0; presentationEpoch++; }
    running = true; presentAssemblyPhysics = true;
    #if !wasm
    if (backgroundPacing && !playbackActive && motions == null && participants.length == 0 &&
        (workforce == null || workforce.ids().length == 0)) startPlayback();
    #end
  }
  public function stop():Void { stopPlayback(); var active = space; if (active != null) active.session.stop(); running = false; pumpStamp = -1.0;
    if (tools != null) {tools.safeWelders(); tools.safeFeeders();}
    if (presentAssemblyPhysics) presentationEpoch++;
    presentAssemblyPhysics = false;
    if(hobbing!=null)hobbing.presentStockFrame(false); }
  public function reset():Bool {
    var active = space;
    if (active == null) return false;
    stopPlayback();
    active.session.stop();
    // Members stop what they run while the robots' runtimes still answer; the session's reset starts them over.
    for (member in members) member.beforeReset();
    active.session.reset(); virtualDevicePrepared = false; running = false; presentAssemblyPhysics = false;
    pumpStamp = -1.0;
    // The session's own reset has restored its objects and actors; what lives beside it follows.
    for (member in members) member.reset();
    for (visual in beltVisuals) visual.reset();
    presentMembers();
    presentationEpoch++; return true;
  }
  public function isRunning():Bool return running;

  /** Operator changes execute between ticks on their playback owner. */
  public function control(action:Void->Void):Void {
    if (!playbackActive) { action(); return; }
    playbackMutex.acquire();
    playbackCommands.push(action);
    playbackMutex.release();
  }

  /** Latch an emergency stop on every robot in this shared simulated cell. */
  public function emergencyStop():Bool {
    if (simulation == null || simulatedIds.length == 0) return false;
    for (id in simulatedIds) {
      emergencyStopRobot(id);
    }
    return true;
  }

  /** Emergency stop one simulated robot by its world ID. */
  public function emergencyStopRobot(id:robotkit.core.RobotId):Bool {
    var robot = simulation == null || simulatedIds.indexOf(id) < 0 ? null : world.robot(id);
    if (robot == null) return false;
    robot.stop(robotkit.core.StopMode.Emergency);
    return true;
  }

  function resetRobotSafety(id:robotkit.core.RobotId):Bool {
    var robot = simulation == null || simulatedIds.indexOf(id) < 0 ? null : world.robot(id);
    if (robot == null) return false;
    var safety = robot.snapshot().safety;
    if (safety == RobotKitRuntimeConstants.RK_SAFETY_FAULT ||
        safety == RobotKitRuntimeConstants.RK_SAFETY_EMERGENCY_STOP)
      robot.resetSafety();
    return true;
  }

  /** Reset only the failed mission robot, then ask its cell controller to retry the failed step. */
  public function retryMission():Bool {
    var failed = [for (program in missions) if (program.failure != null) program];
    if (failed.length != 1 || simulation == null) return false;
    var resume = playbackActive;
    if (resume) { stopPlayback(); var active = space; if (active != null) active.session.stop(); }
    if (!resetRobotSafety(failed[0].robotId())) return false;
    var retried = failed[0].retryFailedStep();
    if (resume) start();
    return retried;
  }

  /** Scene ids of the objects the tool links are holding right now. */
  public function heldObjectIds():Array<String> return tools == null ? [] : tools.heldIds();

  /** Why the project's CNC program stopped, or null while it runs or when there is none. */
  public function cncFailure():Null<String> return cnc == null ? null : cnc.failure;
  /** Why the project's print stopped, or null while it runs or when there is none. */
  public function printFailure():Null<String> return print == null ? null : print.failure;

  /** The project's CNC program player, or null when the project has no CNC job. */
  public function hobbingPlayer():Null<HobbingProgramPlayer> return hobbing;
  public function cncPlayer():Null<CncProgramPlayer> return cnc;
  /** The project's print job player, or null when the project has none. */
  public function printPlayer():Null<PrintProgramPlayer> return print;
  /** The player of the project's G-code job, whatever the machine: CNC or print. */
  public function toolpathPlayer():Null<ToolpathPlayer> return cnc != null ? cnc : print;
  public function programPlayer():Null<MachineProgramPlayer> return machineProgram;
  static function hasPneumaticCylinder(blueprint:robotkit.runtime.RobotRuntimeBlueprint):Bool {
    for (drive in blueprint.cylinderDrives)
      if (drive.control == robotkit.runtime.CylinderControl.PressureEffort) return true;
    return false;
  }

  public function digitalProgramPlayer():Null<MachineDigitalProgramPlayer> return digitalProgram;
  public function pressBrakePlayer():Null<PressBrakeControllerPlayer> return pressBrake;
  public function beltDisplayUpdates():Int {
    var count = 0;
    for (visual in beltVisuals) count += visual.updates;
    return count;
  }
  /** The drive of the project's wheeled assembly, for twist commands; null when it has none. */
  public function mobileBase():Null<robotkit.mobile.MobileBase> return mobile;
  /** The cell's one program, or null when its project ships none; a cell with several programs names one. */
  public function missionPlayer(?controllerId:String):Null<MissionPlayer> {
    if (controllerId != null) {
      for (program in missions) if (program.mission.controllerId == controllerId) return program;
      return null;
    }
    if (missions.length > 1) throw "The cell runs several programs; name the controller whose program is wanted";
    return missions.length == 0 ? null : missions[0];
  }
  /** Every robot controller's program, in authored order. */
  public function missionPlayers():Array<MissionPlayer> return missions.copy();
  /** The weld metal the mission has laid, or null when it does not weld. */
  public function weldBeads():Null<WeldBeads> return beads;
  /** The simulated welder on the assembly's torch, or null when it has none. */
  public function welder():Null<processkit.simulation.SimulatedWelder> return tools == null || tools.welders.length == 0 ? null : tools.welders[0];

  /** The stock the project's CNC program is cutting, or null when it cuts none. */
  public function machiningStock():Null<MachiningStock> return cnc == null ? null : cnc.stock;
  public function humanWorker(id:String):Null<HumanWorker> return workforce == null ? null : workforce.worker(id);
  public function humanSignals(id:String):Null<HumanWorkerSignals>
    return workforce == null ? null : workforce.signals(id);
  public function humanWarnings(id:String):Array<String>
    return workforce == null ? [] : workforce.warningsFor(id);
  public function humanWorkerIds():Array<String> return workforce == null ? [] : workforce.ids();

  /**
   * Runs the realtime simulation from the owner's loop: the ticks that the
   * wall time since the last pump has made due, each fed to the session's
   * members before it runs. Members are fed on the simulation clock, one call
   * per tick, so a slow frame cannot starve them of motion.
   */
  public function pump():Void {
    if (retirementFrame != null) return;
    if (playbackActive) {
      playbackMutex.acquire();
      var failure = playbackFailure, frame = playbackFrame;
      playbackMutex.release();
      if (failure != null) { stop(); error = failure; return; }
      if (frame != null && frame.revision != presentedPlaybackRevision) {
        // Logical-world events and subscribed streams remain on their UI owner.
        world.pump();
        for (robot in ownedRobots) robot.streams();
        world.pump();
        presentMembers();
        presentedPlaybackRevision = frame.revision;
      }
      return;
    }
    try pumpOwner(true) catch (failure:Dynamic) {
      stop();
      error = "Physics playback stopped: " + Std.string(failure);
    }
  }

  function pumpOwner(present:Bool):Void {
    var active = space;
    if (active == null || !running) return;
    var now = Sys.time();
    var elapsed = pumpStamp < 0.0 ? 0.0 : Math.max(0.0, now - pumpStamp);
    pumpStamp = now;
    if(hobbing!=null)hobbing.advanceHandling(elapsed);
    var due = active.session.dueTicks(haxe.Int64.fromFloat(elapsed * 1.0e9), MAX_TICKS_PER_PUMP);
    runOwnerTicks(due, present);
  }

  /**
   * Runs `count` realtime ticks and presents once, as a frame of the running simulation does. Separate from `pump`
   * so the cost of a frame can be measured without waiting on the wall clock.
   */
  public function runTicks(count:Int):Void {
    if (playbackActive) throw "Background playback owns the simulation ticks";
    runOwnerTicks(count, true, true);
  }

  function runOwnerTicks(count:Int, present:Bool,deterministic:Bool=false):Void {
    var active = space;
    if (active == null || !running) return;
    for (_ in 0...count) {
      if(deterministic&&hobbing!=null)hobbing.advanceHandling(active.session.fixedTimestep());
      feedMembers();
      active.session.stepPaced();
    }
    if (present && count > 0) presentMembers();
  }

  #if !wasm
  function startPlayback():Void {
    var active = space;
    if (active == null) throw "Apply the pending simulation configuration first";
    playbackFrame = captureOwnerSnapshot();
    playbackStop = false;
    playbackFailure = null;
    playbackDone = new Lock();
    playbackActive = true;
    var done = playbackDone;
    Thread.create(function() {
      try {
        active.session.stop(); // Transfer the prepared world to the playback owner.
        var previous = -1;
        while (true) {
          playbackMutex.acquire();
          var stop = playbackStop;
          var commands = playbackCommands;
          playbackCommands = [];
          playbackMutex.release();
          for (command in commands) command();
          if (stop) break;
          pumpOwner(false);
          var revision = haxe.Int64.toInt(active.session.stepIndex());
          if (revision != previous) {
            var frame = captureOwnerSnapshot();
            playbackMutex.acquire();
            playbackFrame = frame;
            playbackMutex.release();
            previous = revision;
          }
          Sys.sleep(0.001);
        }
      } catch (failure:Dynamic) {
        playbackMutex.acquire();
        playbackFailure = Std.string(failure);
        playbackMutex.release();
      }
      done.release();
    });
  }
  #end

  function stopPlayback():Void {
    #if !wasm
    if (!playbackActive) return;
    playbackMutex.acquire();
    playbackStop = true;
    playbackMutex.release();
    playbackDone.wait();
    playbackDone = null;
    playbackActive = false;
    playbackFrame = null;
    presentedPlaybackRevision = -1;
    #end
  }

  function feedMembers():Void {
    if (virtualWelder && !virtualDevicePrepared && simulation != null && tools != null && tools.welders.length > 0)
      virtualDevicePrepared = simulation.virtualDeviceReady(tools.welders[0].robotIndex);
    if (virtualNutrunner && !virtualDevicePrepared && simulation != null && tools != null && tools.screwdrivers.length > 0)
      virtualDevicePrepared = simulation.virtualDeviceReady(@:privateAccess tools.screwdrivers[0].robotIndex);
    for (member in members) {
      if ((virtualWelder || virtualNutrunner) && !virtualDevicePrepared && Std.isOfType(member, MissionPlayer)) continue;
      // The editor's realtime run plans a program's first plan beside the simulation; a deterministic step waits for it.
      if (Std.isOfType(member, ToolpathPlayer)) {
        var toolpath:ToolpathPlayer = cast member;
        toolpath.planAhead = running;
      }
      member.feed();
    }
  }

  /** Publishes every member's current state to the scene, once per presented frame. */
  function presentMembers():Void {
    for (member in members) if(member==hobbing)hobbing.presentStockFrame(presentAssemblyPhysics);else member.present();
  }

  /** True in both running and paused simulation modes. */
  public function isActive():Bool return simulation!=null;
  /** Where physics has a robot's link, not where the robot's device reports its joints (a machine knows only its motors). */
  public function physicalLinkPosition(robotIndex:Int, linkIndex:Int):Array<Float> {
    if (simulation == null) throw "no simulation is running";
    return simulation.linkPose(robotIndex, linkIndex).position.copy();
  }
  /** The live session, for people and other participants that join it. */
  public function activeSession():Null<SimSession> { var active = space; return active == null ? null : active.session; }
  public function environmentObject(id:String):Null<SimObject> {
    for (item in simulatedObjects) if (item.id == id) return item.object;
    return null;
  }
  public function simulatedRobotIds():Array<String> return simulatedIds.copy();
  /** Captures physics poses once, then combines them with the matching sensor streams. */
  public function capturePresentationSnapshot():ApplicationPresentationSnapshot {
    if (retirementFrame != null) return retirementFrame;
    if (playbackActive) {
      playbackMutex.acquire();
      var frame = playbackFrame;
      playbackMutex.release();
      if (frame != null) return frame;
    }
    return captureOwnerSnapshot();
  }

  function captureOwnerSnapshot():ApplicationPresentationSnapshot {
    var active = space, robotsInSession = simulation;
    var frame = active == null ? null : active.session.capture();
    var physics = frame == null || robotsInSession == null ? null : robotsInSession.presentFrame(frame);
    try {
      for (visual in beltVisuals) visual.present(physics);
    } catch (error:Dynamic) {
      if (physics != null) physics.dispose();
      if (frame != null) frame.dispose();
      throw error;
    }
    var robotSensorFrames:Array<Null<Array<SensorFrame>>> = [];
    for (_ in 0...simulatedIds.length) robotSensorFrames.push(null);
    try {
      // Poll every attached robot as before, but avoid constructing a WorldSnapshot that this
      // presentation never reads. Streams already observe the runtime snapshot and return the
      // immutable frames needed by the viewport.
      if (!playbackActive) world.pump();
      var observedRobots = playbackActive ? ownedRobots : world.robots();
      for (robot in observedRobots) {
        var id = robot.id();
        var frames = robot.streams().latestFrames();
        var index = simulatedIds.indexOf(id);
        if (index >= 0) robotSensorFrames[index] = frames;
      }
      if (!playbackActive) world.pump();
    } catch (error:Dynamic) {
      if (physics != null) physics.dispose();
      if (frame != null) frame.dispose();
      throw error;
    }
    var robots:Array<SimulationRobotVisual> = [];
    for (index in 0...simulatedIds.length) {
      var id = simulatedIds[index];
      var sensorFrames = robotSensorFrames[index];
      robots.push({id:id,position:[0.0,0.0,0.0],rotation:[0.0,0.0,0.0,1.0],
        sensors:sensorFrames == null ? [] : sensorFrames,links:[for (linkId in simulatedLinks[index])
          {id:linkId,position:[0.0,0.0,0.0],rotation:[0.0,0.0,0.0,1.0]}]});
    }
    if (physics != null) for (pose in physics.poses) switch pose.kind {
      case SimulationPresentationSnapshot.ROBOT_BASE:
        if (pose.robotIndex < robots.length) {
          robots[pose.robotIndex].position = pose.position;
          robots[pose.robotIndex].rotation = pose.rotation;
        }
      case SimulationPresentationSnapshot.ROBOT_LINK:
        if (pose.robotIndex < robots.length && pose.linkIndex < robots[pose.robotIndex].links.length) {
          var link = robots[pose.robotIndex].links[pose.linkIndex];
          link.position = pose.position;
          link.rotation = pose.rotation;
        }
      default:
    }
    var orderedEnvironment:Array<SimulationPoseVisual> = [];
    if (frame != null) {
      for (object in simulatedObjects) {
        var pose = frame.objectPose(object.object);
        orderedEnvironment.push({id:object.id,position:[pose.x,pose.y,pose.z],
          rotation:[pose.qx,pose.qy,pose.qz,pose.qw]});
      }
      frame.dispose();
    }
    if (presentAssemblyPhysics) for (part in assemblyParts) {
      // The fixed geometry-centred frame is already expressed on the body's link.
      var link = robots[part.robotIndex].links[part.linkIndex];
      var pose = AssemblyRobot.composeComponents(link.position, link.rotation, part.offset);
      orderedEnvironment.push({id:part.id, position:pose.position, rotation:pose.rotation});
    }
    return new ApplicationPresentationSnapshot(physics, robots, orderedEnvironment,
      presentationEpoch);
  }

  /** Compatibility helper; frame consumers should share capturePresentationSnapshot(). */
  public function visualState():Array<SimulationRobotVisual> {
    var snapshot = capturePresentationSnapshot();
    var result = snapshot.robots;
    return result;
  }
  public function environmentVisualState():Array<SimulationPoseVisual> {
    var snapshot = capturePresentationSnapshot();
    var result = snapshot.environment;
    return result;
  }
  /** Authored CAD pieces use the same source as robot links, centred on the scene body. */
  static function projectEnvironmentShape(project:Null<ProjectDocumentSession>, id:String):Null<SimShape> {
    if (project == null) return null;
    var definition = project.projectAssemblyDefinition;
    if (definition == null) return null;
    var occurrences = [for (item in definition.occurrences) if (id == "project:"+item.id) item];
    if (occurrences.length == 0) return null;
    if (occurrences.length != 1) throw "Environment body has ambiguous CAD identity: "+id;
    var occurrence = occurrences[0];
    var components = [for (item in definition.definitions) if (item.id == occurrence.definition) item];
    if (components.length != 1) throw "Environment body has no unique CAD component: "+id;
    var component = components[0];
    if (component.collisionHulls == null) return null;
    var physical = project.projectPhysical, center = project.assemblyPreviewCenter(occurrence.definition);
    if (physical == null || center == null) throw "Environment body has no CAD collision frame: "+id;
    var source = cadbridge.AssemblySimulationBridge.collisionPieces(occurrence.id,component,null);
    var pieces = [for (vertices in source)
      [for (i in 0...vertices.length) (vertices[i]-center[i%3])*physical.metresPerUnit]];
    return pieces.length == 1 ? SimShape.convex(pieces[0]) : SimShape.compoundConvex(pieces);
  }

  /** Freeze presentation before the shutdown worker takes ownership. */
  public function beginStop():Void {
    retirementFrame = capturePresentationSnapshot();
  }

  /** Joins playback and physics without touching UI-owned scene or RobotWorld state. */
  public function quiesceForStop():Void {
    stopPlayback();
    var active = space;
    if (active != null) active.session.stop();
  }

  /** A failed retirement must not leave presentation frozen or native ownership on its worker. */
  public function recoverFailedStop():Void {
    retirementFrame = null;
    stop();
  }

  /** Scene/world changes stay on their owner; blocking releases can be deferred to a worker. */
  public function clear():Void clearWithReleases(null);

  public function clearWithReleases(releases:Null<Array<Void->Void>>):Void {
    function release(action:Void->Void):Void {
      if (releases == null) action(); else releases.push(action);
    }
    stop();
    for (program in missions) program.beforeReset();
    for (participant in participants) participant.leave();
    var retired = workforce;
    if (retired != null) {
      retired.dispose();
      retired.scene.setWorkerVisualsVisible(true);
    }
    workforce = null; motions = null; tools = null; power = null;
    if (cnc != null) cnc.dispose(releases);
    if (hobbing != null) { var retiredHobbing=hobbing;release(()->retiredHobbing.dispose());hobbing=null; }
    if (print != null) print.dispose(releases);
    if (beads != null) beads.dispose();
    for (visual in beltVisuals) visual.reset();
    beltVisuals = [];
    if (machineProgram != null) machineProgram.beforeReset();
    machineProgram = null;
    if (digitalProgram != null) digitalProgram.beforeReset();
    digitalProgram = null;
    if (pressBrake != null) pressBrake.beforeReset();
    pressBrake = null;
    cnc = null; print = null; mobile = null; missions = []; beads = null; cellSignals = null; refreshMembers();
    // A mission's programs are planned on worker threads that finish the plan they are on after they are cancelled; the
    // world they plan for goes now, so wait for them (see ProgramPlanner).
    if (!preparationOnly) release(function() motionkit.robot.ProgramPlanner.shutdown());
    if (detachedPreparedRobots != null) {
      for (robot in detachedPreparedRobots) release(function() robot.close());
      detachedPreparedRobots = null;
    } else for (id in simulatedIds) { var robot=world.detach(id); if(robot!=null)release(function() robot.close()); }
    ownedRobots = [];
    simulatedIds.resize(0);
    simulatedLinks.resize(0); simulatedObjects.resize(0);
    assemblyParts.resize(0);
    collisionWarnings = [];
    presentAssemblyPhysics = false;
    var retiredSpace = space, retiredSimulation = simulation;
    release(function() releaseSpace(retiredSpace, retiredSimulation));
    simulation = null; space = null;
    retirementFrame = null;
    for (zone in zoneMonitors) zone.dispose();
    zoneMonitors = [];
    appliedDocumentRevision=-1;appliedEnvironmentRevision=-1;appliedBackend=-1;appliedTimestep=-1;
    appliedParticipantRevision=-1;
  }
  public function dispose():Void clear();
}
