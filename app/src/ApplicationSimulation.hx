package app;

import haxe.Int64;
import humankit.sim.HumanWorker;
import humankit.sim.HumanWorkerSignals;
import robotkit.runtime.RobotRuntimeCompiler;
import robotkit.runtime.Simulation;
import robotkit.runtime.SimulationSpace;
import robotkit.runtime.SimulationPresentationSnapshot;
import robotkit.core.Robot;
import robotkit.world.RobotWorld;
import robotkit.simulation.SimulatedRobot;
import robotkit.world.WorldSnapshot;
import robotkit.core.SensorFrame;
import nativekit.sim.MotionType;
import nativekit.sim.SimObject;
import nativekit.sim.SimPose;
import nativekit.sim.SimSession;
import nativekit.sim.SimShape;

typedef SimulationRobotVisual={
  var id:String;
  var position:Array<Float>;
  var rotation:Array<Float>;
  var sensors:Array<SensorFrame>;
  var links:Array<SimulationPoseVisual>;
}
typedef SimulationPoseVisual={var id:String;var position:Array<Float>;var rotation:Array<Float>;}

/**
 * Owns the one shared editable-scene simulation attached to the application
 * world: a SimKit session in which the robots, the environment props, and any
 * people take part.
 */
class ApplicationSimulation {
  public static inline var DETERMINISTIC:Int=0;
  public static inline var MUJOCO:Int=1;
  public final world:RobotWorld;
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
  var simulatedLinks:Array<Array<String>> = [];
  var simulatedObjects:Array<{id:String,object:SimObject}> = [];
  var motions:Null<RobotMotionPlayer> = null;
  /** The simulated tools on the session's robots. */
  var tools:Null<SimulatedTools> = null;
  var cnc:Null<CncProgramPlayer> = null;
  /** The project's wheeled assembly's drive, when it has one. */
  var mobile:Null<robotkit.mobile.MobileBase> = null;
  /** The work the assembly robot does on its own, when its project ships a mission. */
  var mission:Null<MissionPlayer> = null;
  /** The weld metal the mission's welds lay, when it welds. */
  var beads:Null<WeldBeads> = null;
  var workforce:Null<HumanWorkforce> = null;
  /** Everything that follows the session's lifecycle, in the order it is fed. */
  var members:Array<SessionMember> = [];
  var assemblyParts:Array<AssemblyPart> = [];
  var running:Bool = false;
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
    if (motions != null) members.push(motions);
    if (tools != null) members.push(tools);
    if (cnc != null) members.push(cnc);
    if (mission != null) members.push(mission);
    if (beads != null) members.push(beads);
    if (workforce != null) members.push(workforce);
    for (participant in participants) members.push(participant);
  }

  /** Builds the complete candidate before changing any live world adapter. */
  public function rebuild(configuration:SensorConfiguration, scene:EditorScene,
      ?session:ProjectDocumentSession):Bool {
    var candidateSpace:Null<SimulationSpace> = null;
    var candidate:Null<Simulation> = null;
    var candidateRobots:Array<SimulatedRobot> = [];
    var candidateLinks:Array<Array<String>> = [];
    var candidateRobotModels:Array<robotkit.model.RobotModel> = [];
    var candidateObjects:Array<{id:String,object:SimObject}> = [];
    var candidateWorkforce:Null<HumanWorkforce> = null;
    var candidateAssemblyParts:Array<AssemblyPart> = [];
    var candidateWarnings:Array<String> = [];
    var candidateCnc:Null<CncProgramPlayer> = null;
    var candidateMobile:Null<robotkit.mobile.MobileBase> = null;
    var candidateMission:Null<MissionPlayer> = null;
    var candidateBeads:Null<WeldBeads> = null;
    var candidateAssembly:Null<AssemblyRobot> = null;
    var assemblyIndex = -1;
    var heldBoxes:Array<MissionPlayer.FloorObstacle> = [];
    try {
      var models = configuration.robotModels();
      var hasWorkers = false;
      for (record in scene.records()) if (record.type == "human-worker") { hasWorkers = true; break; }
      var assembly = session == null ? null : session.projectAssemblyDefinition;
      if (models.length == 0 && assembly == null && participants.length == 0 &&
          !hasWorkers) throw "Nothing to simulate";
      for (configured in models) {
        var id=configured.id;
        var existing = world.robot(id);
        if (existing != null && simulatedIds.indexOf(id) < 0)
          throw 'Robot "$id" is remote and read-only';
      }
      var createdSpace = SimulationSpace.create(backend, timestep);
      candidateSpace = createdSpace;
      candidate = Simulation.inSpace(createdSpace);
      for (index in 0...models.length) {
        var editable=models[index];
        var blueprint = RobotRuntimeCompiler.compile(editable.model, editable.profile, appliedRevision + 1);
        var runtime = candidate.addRobotAtPose(blueprint, editable.position, editable.rotation);
        var id = editable.id;
        candidateRobots.push(new SimulatedRobot(id, runtime, editable.model.name,
          [for (link in editable.model.links) link.id], [for (joint in editable.model.joints) joint.id]));
        candidateLinks.push([for (link in editable.model.links) link.id]);
        candidateRobotModels.push(editable.model);
      }
      var candidateMotions = session == null ? [] : session.robotMotions;
      if (assembly != null && session != null) {
        var id = AssemblyRobot.idFor(assembly);
        if (world.robot(id) != null && simulatedIds.indexOf(id) < 0)
          throw 'Robot "$id" is remote and read-only';
        var robotIndex = candidateRobots.length;
        assemblyIndex = robotIndex;
        var built = AssemblyRobot.add(candidate, scene, session, assembly, backend == MUJOCO,
          appliedRevision + 1, robotIndex, session.cncJob == null ? null : CncProgramPlayer.processChannels());
        candidateRobots.push(built.robot);
        candidateLinks.push([for (link in built.model.links) link.id]);
        candidateRobotModels.push(built.model);
        for (part in built.parts) candidateAssemblyParts.push(part);
        for (warning in built.warnings) candidateWarnings.push(warning);
        candidateMobile = built.mobile;
        candidateAssembly = built;
        // A bad program fails the rebuild here, before anything live changes.
        var job = session.cncJob;
        if (job != null)
          candidateCnc = new CncProgramPlayer(job, built, candidate, session, createdSpace.session);
      }
      var resolvedMotions = RobotMotionPlayer.resolve(candidateMotions,
        [for (robot in candidateRobots) robot.id()], candidateRobotModels);
      var environmentRecords = scene.records();
      environmentRecords.sort(function(a, b) return Reflect.compare(a.id, b.id));
      var assemblyOwned = new Map<String, Bool>();
      if (assembly != null) {
        var free = AssemblyRobot.unownedOccurrences(scene, session, assembly);
        for (occurrence in assembly.occurrences)
          if (!free.exists(occurrence.id)) assemblyOwned.set("project:" + occurrence.id, true);
      }
      for (object in environmentRecords) if (object.collisionEnabled && !assemblyOwned.exists(object.id)) {
        var centerX=object.x,centerY=object.y,centerZ=object.z;
        var halfX=object.width/2.0,halfY=object.height/2.0,halfZ=object.depth/2.0;
        if(scene.isCadPart(object.id)){
          var bounds=scene.cadSession(object.id).collisionBounds;
          if(bounds==null)throw "CAD collision bounds are unavailable for: "+object.id;
          var offset = rotateOffset(bounds.center.x, bounds.center.y, bounds.center.z, object.rotation);
          centerX+=offset[0];centerY+=offset[1];centerZ+=offset[2];
          halfX=bounds.halfExtents.x;halfY=bounds.halfExtents.y;halfZ=bounds.halfExtents.z;
        }
        var rotation=object.rotation==null?[0.0,0.0,0.0,1.0]:object.rotation;
        var created=createdSpace.session.createObject(
          object.dynamicBody?MotionType.Dynamic:MotionType.Static,SimShape.box(halfX,halfY,halfZ),
          new SimPose(centerX,centerY,centerZ,rotation[0],rotation[1],rotation[2],rotation[3]),
          object.dynamicBody?object.mass:0.0);
        candidateObjects.push({id:object.id,object:created});
        if (!object.dynamicBody) heldBoxes.push({id: object.id, x: centerX, y: centerY, z: centerZ,
          halfX: halfX, halfY: halfY, halfZ: halfZ,
          yaw: Math.atan2(2 * (rotation[3] * rotation[2] + rotation[0] * rotation[1]),
            1 - 2 * (rotation[1] * rotation[1] + rotation[2] * rotation[2]))});
      }
      // The robot's tools act on the free objects after each step, as their channels say.
      var candidateTools = new SimulatedTools([for (entry in candidateObjects) {id: entry.id, object: entry.object}]);
      var freeObjects = [for (entry in candidateObjects) entry.object];
      if (session != null && candidateAssembly != null) for (tool in session.robotTools) {
        if (tool.kind == "torch") {
          candidate.addStepObserver(candidateTools.addWelder(SimulatedTools.welderFor(candidate, candidateAssembly,
            assemblyIndex, tool, session)));
          continue;
        }
        var carrier = candidateAssembly.part("project:" + tool.contact.occurrence);
        candidate.addStepObserver(candidateTools.add(new robotkit.runtime.SimulatedSuctionTool(candidate,
          candidateAssembly.runtime, assemblyIndex, carrier.linkIndex, tool.channel, freeObjects, tool.sensor)));
      }
      var work = session == null ? null : session.mission;
      if (work != null) {
        if (candidateAssembly == null) throw "A mission needs the project's assembly";
        candidateMission = new MissionPlayer(work, candidateAssembly, candidate, assemblyIndex, timestep, heldBoxes,
          candidateTools.objectsByScene(), session);
        // A mission that welds lays the weld metal as the welder deposits it.
        var welds = [for (step in work.steps) if (step.kind == "weld") step].length > 0;
        if (welds) {
          var active:ProjectDocumentSession = cast session;
          var torch = [for (tool in active.robotTools) if (tool.kind == "torch") tool][0].torch;
          if (candidateTools.welders.length != 1 || torch == null) throw "A mission that welds needs the robot's one simulated welder";
          candidateBeads = new WeldBeads(candidateMission, candidateTools.welders[0], candidate, candidateAssembly.parts, scene,
            timestep, torch.wireDiameterMm, torch.depositionEfficiency);
        }
      }

      var objectsById:Map<String, SimObject> = new Map();
      for (entry in candidateObjects) objectsById.set(entry.id, entry.object);
      candidateWorkforce = HumanWorkforce.build(createdSpace.session, scene, environmentRecords,
        objectsById, [for (robot in candidateRobots) robot.id()], candidateRobotModels, candidate);
      for (warning in candidateWorkforce.warnings) candidateWarnings.push(warning);

      var previousSimulation = simulation;
      var previousSpace = space;
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
      space = candidateSpace;
      simulatedIds = [for (robot in candidateRobots) robot.id()];
      simulatedLinks = candidateLinks;
      simulatedObjects = candidateObjects;
      motions = resolvedMotions.length == 0 ? null :
        new RobotMotionPlayer(world, createdSpace.session, resolvedMotions);
      tools = candidateTools;
      workforce = candidateWorkforce;
      if (cnc != null) cnc.dispose();
      cnc = candidateCnc;
      mobile = candidateMobile;
      mission = candidateMission;
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
      scene.setWorkerVisualsVisible(false);
      if (previousWorkforce != null) previousWorkforce.dispose();
      releaseSpace(previousSpace, previousSimulation);
      for (robot in previousRobots) robot.close();
      return true;
    } catch (failure:Dynamic) {
      error = Std.string(failure);
      if (candidateWorkforce != null) candidateWorkforce.dispose();
      if (candidateSpace != null && candidateSpace != space)
        releaseSpace(candidateSpace, candidate);
      return false;
    }
  }

  /** Stops a space, removes its robots, then releases the space itself. */
  static function releaseSpace(released:Null<SimulationSpace>, robots:Null<Simulation>):Void {
    if (released != null) released.session.stop();
    if (robots != null) robots.dispose();
    if (released != null) released.dispose();
  }

  public static function rotateOffset(x:Float, y:Float, z:Float, rotation:Null<Array<Float>>):Array<Float> {
    if (rotation == null) return [x, y, z];
    var qx = rotation[0], qy = rotation[1], qz = rotation[2], qw = rotation[3];
    var tx = 2 * (qy * z - qz * y), ty = 2 * (qz * x - qx * z), tz = 2 * (qx * y - qy * x);
    return [x + qw * tx + qy * tz - qz * ty,
      y + qw * ty + qz * tx - qx * tz,
      z + qw * tz + qx * ty - qy * tx];
  }

  /** Advances the simulation one tick. Read what it produced with `snapshot()`. */
  public function step(?timestampNs:Int64):Void {
    var active = space;
    if (active == null) throw "Apply the pending simulation configuration first";
    if (running) throw "Stop realtime simulation before deterministic stepping";
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
  }
  public function stop():Void { var active = space; if (active != null) active.session.stop(); running = false; pumpStamp = -1.0;
    if (presentAssemblyPhysics) presentationEpoch++;
    presentAssemblyPhysics = false; }
  public function reset():Bool {
    var active = space;
    if (active == null) return false;
    active.session.stop();
    // Members stop what they run while the robots' runtimes still answer; the session's reset starts them over.
    for (member in members) member.beforeReset();
    active.session.reset(); running = false; presentAssemblyPhysics = false;
    pumpStamp = -1.0;
    // The session's own reset has restored its objects and actors; what lives beside it follows.
    for (member in members) member.reset();
    presentMembers();
    presentationEpoch++; return true;
  }
  public function isRunning():Bool return running;
  /** Scene ids of the objects the tool links are holding right now. */
  public function heldObjectIds():Array<String> return tools == null ? [] : tools.heldIds();

  /** Why the project's CNC program stopped, or null while it runs or when there is none. */
  public function cncFailure():Null<String> return cnc == null ? null : cnc.failure;

  /** The project's CNC program player, or null when the project has no CNC job. */
  public function cncPlayer():Null<CncProgramPlayer> return cnc;
  /** The drive of the project's wheeled assembly, for twist commands; null when it has none. */
  public function mobileBase():Null<robotkit.mobile.MobileBase> return mobile;
  /** The mission the assembly robot is running, or null when its project ships none. */
  public function missionPlayer():Null<MissionPlayer> return mission;
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
    var active = space;
    if (active == null || !running) return;
    var now = Sys.time();
    var elapsed = pumpStamp < 0.0 ? 0.0 : Math.max(0.0, now - pumpStamp);
    pumpStamp = now;
    var due = active.session.dueTicks(haxe.Int64.fromFloat(elapsed * 1.0e9), MAX_TICKS_PER_PUMP);
    runTicks(due);
  }

  /**
   * Runs `count` realtime ticks and presents once, as a frame of the running simulation does. Separate from `pump`
   * so the cost of a frame can be measured without waiting on the wall clock.
   */
  public function runTicks(count:Int):Void {
    var active = space;
    if (active == null || !running) return;
    for (_ in 0...count) {
      feedMembers();
      active.session.stepPaced();
    }
    if (count > 0) presentMembers();
  }

  function feedMembers():Void {
    for (member in members) member.feed();
  }

  /** Publishes every member's current state to the scene, once per presented frame. */
  function presentMembers():Void {
    for (member in members) member.present();
  }

  /** True in both running and paused simulation modes. */
  public function isActive():Bool return simulation!=null;
  /** The live session, for people and other participants that join it. */
  public function activeSession():Null<SimSession> { var active = space; return active == null ? null : active.session; }
  public function environmentObject(id:String):Null<SimObject> {
    for (item in simulatedObjects) if (item.id == id) return item.object;
    return null;
  }
  public function simulatedRobotIds():Array<String> return simulatedIds.copy();
  /** Captures physics poses once, then combines the matching frame's world publications. */
  public function capturePresentationSnapshot():ApplicationPresentationSnapshot {
    var active = space, robotsInSession = simulation;
    var frame = active == null ? null : active.session.capture();
    var physics = frame == null || robotsInSession == null ? null : robotsInSession.presentFrame(frame);
    var publication:WorldSnapshot;
    try publication = world.snapshot() catch (error:Dynamic) {
      if (physics != null) physics.dispose();
      if (frame != null) frame.dispose();
      throw error;
    }
    var robots:Array<SimulationRobotVisual> = [];
    for (index in 0...simulatedIds.length) {
      var id = simulatedIds[index];
      var robot = publication.robot(id);
      robots.push({id:id,position:[0.0,0.0,0.0],rotation:[0.0,0.0,0.0,1.0],
        sensors:world.robot(id)==null?[]:world.robot(id).streams().latestFrames(),links:[for (linkId in simulatedLinks[index])
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
      // The part's frame on its body's link, then its geometry's centre in that frame.
      var link = robots[part.robotIndex].links[part.linkIndex];
      var pose = AssemblyRobot.compose({position: link.position, rotation: link.rotation}, part.offset);
      var offset = rotateOffset(part.center[0], part.center[1], part.center[2], pose.rotation);
      orderedEnvironment.push({id:part.id,
        position:[pose.position[0] + offset[0], pose.position[1] + offset[1],
          pose.position[2] + offset[2]], rotation:pose.rotation});
    }
    return new ApplicationPresentationSnapshot(publication, physics, robots, orderedEnvironment,
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
  public function clear():Void {
    stop();
    for (participant in participants) participant.leave();
    var retired = workforce;
    if (retired != null) {
      retired.dispose();
      retired.scene.setWorkerVisualsVisible(true);
    }
    workforce = null; motions = null; tools = null;
    if (cnc != null) cnc.dispose();
    if (beads != null) beads.dispose();
    cnc = null; mobile = null; mission = null; beads = null; refreshMembers();
    // A mission's programs are planned on worker threads that finish the plan they are on after they are cancelled; the
    // world they plan for goes now, so wait for them (see ProgramPlanner).
    motionkit.robot.ProgramPlanner.shutdown();
    for (id in simulatedIds) { var robot=world.detach(id); if(robot!=null)robot.close(); }
    simulatedIds.resize(0);
    simulatedLinks.resize(0); simulatedObjects.resize(0);
    assemblyParts.resize(0);
    collisionWarnings = [];
    presentAssemblyPhysics = false;
    releaseSpace(space, simulation); simulation = null; space = null;
    appliedDocumentRevision=-1;appliedEnvironmentRevision=-1;appliedBackend=-1;appliedTimestep=-1;
    appliedParticipantRevision=-1;
  }
  public function dispose():Void clear();
}
