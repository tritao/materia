package app;

import haxe.Int64;
import humankit.sim.HumanWorker;
import humankit.sim.HumanWorkerSignals;
import robotkit.runtime.RobotRuntimeCompiler;
import robotkit.runtime.Simulation;
import robotkit.runtime.SimulationClosure;
import robotkit.runtime.SimulationSpace;
import RobotKitRuntime;
import robotkit.runtime.SimulationPresentationSnapshot;
import robotkit.world.Robot;
import robotkit.world.RobotWorld;
import robotkit.world.SimulatedRobot;
import robotkit.world.WorldSnapshot;
import robotkit.world.SensorFrame;
import robotkit.model.CollisionApproximation;
import materia.project.MaterialLibrary;
import cadbridge.AssemblySimulationBridge;
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
  var workforce:Null<HumanWorkforce> = null;
  /** Everything that follows the session's lifecycle, in the order it is fed. */
  var members:Array<SessionMember> = [];
  var assemblyParts:Array<{id:String,robotIndex:Int,linkIndex:Int,center:Array<Float>}> = [];
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
    var candidateAssemblyParts:Array<{id:String,robotIndex:Int,linkIndex:Int,center:Array<Float>}> = [];
    var candidateWarnings:Array<String> = [];
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
      candidate = Simulation.inSession(createdSpace.session);
      for (index in 0...models.length) {
        var editable=models[index];
        var blueprint = RobotRuntimeCompiler.compile(editable.model, appliedRevision + 1);
        var runtime = candidate.addRobotAtPose(blueprint, editable.position, editable.rotation);
        var id = editable.id;
        candidateRobots.push(new SimulatedRobot(id, runtime, editable.model.name,
          [for (link in editable.model.links) link.id], [for (joint in editable.model.joints) joint.id]));
        candidateLinks.push([for (link in editable.model.links) link.id]);
        candidateRobotModels.push(editable.model);
      }
      var candidateMotions = session == null ? [] : session.robotMotions;
      if (assembly != null) {
        var physical = session == null ? null : session.projectPhysical;
        if (physical == null) throw "Assembly physical properties are unavailable";
        var converted = AssemblySimulationBridge.toRobotModel(assembly, physical,
          session == null ? null : session.projectAssemblyState);
        // Link collision geometry is installed with generated-part hulls in the
        // collision phase; the runtime's generic 10 cm robot box is not a part shape.
        converted.model.collisionApproximation = CollisionApproximation.None;
        var sceneParts = new Map<String, SceneObjectData>();
        for (record in scene.records()) sceneParts.set(record.id, record);
        var collisionHulls:Array<Null<Array<Float>>> = [for (_ in converted.model.links) null];
        var physicalParts = new Map<String, cadbridge.AssemblySimulationBridge.AssemblyPhysicalPart>();
        for (part in physical.parts) physicalParts.set(part.id, part);
        for (part in physical.parts) if (part.collisionWarning != null)
          candidateWarnings.push(part.id + ": " + part.collisionWarning);
        for (occurrence in assembly.occurrences) {
          var record = sceneParts.get("project:" + occurrence.id);
          var part = physicalParts.get(occurrence.definition);
          if (record == null || part == null)
            throw 'Assembly occurrence "${occurrence.id}" is missing its generated part';
          var link:Null<robotkit.model.Link> = null;
          for (item in converted.model.links) if (item.id == occurrence.id) { link = item; break; }
          if (link == null) throw 'Assembly occurrence "${occurrence.id}" has no simulated link';
          var baseMass = link.mass;
          var chosenMass = record.mass;
          if (record.materialId != null && record.materialId != part.materialId &&
              Math.abs(record.mass - baseMass) <= 1e-9 * Math.max(1.0, baseMass)) {
            var density:Null<Float> = null;
            var customMaterials = session == null ? [] : session.customMaterials;
            for (material in customMaterials)
              if (material.id == record.materialId) { density = material.physical.density; break; }
            if (density == null) density = MaterialLibrary.require(record.materialId).physical.density;
            chosenMass = part.volume * density * Math.pow(physical.metresPerUnit, 3);
          }
          if (!Math.isFinite(chosenMass) || chosenMass <= 0)
            throw 'Assembly occurrence "${occurrence.id}" has an invalid mass';
          link.inertiaTensor = [for (value in link.inertiaTensor) value * chosenMass / baseMass];
          link.mass = chosenMass;
          if (record.collisionEnabled) {
            if (part.collisionHull == null || part.collisionHull.length < 12)
              throw 'Assembly part "${occurrence.definition}" has no convex collision hull';
            var linkIndex = converted.model.links.indexOf(link);
            collisionHulls[linkIndex] = [for (value in part.collisionHull)
              value * physical.metresPerUnit];
          }
        }
        if (converted.closureIds.length > 0 && backend != MUJOCO)
          throw "Assembly closures require MuJoCo equality constraints: " +
            converted.closureIds.join(", ");
        var closures:Array<SimulationClosure> = [];
        for (closure in converted.closures) {
          var type = switch (closure.type) {
            case materia.assembly.AssemblyDefinition.AssemblyJointType.Fixed:
              RobotKitRuntimeConstants.RK_RUNTIME_JOINT_FIXED;
            case materia.assembly.AssemblyDefinition.AssemblyJointType.Revolute,
                 materia.assembly.AssemblyDefinition.AssemblyJointType.Continuous:
              RobotKitRuntimeConstants.RK_RUNTIME_JOINT_REVOLUTE;
            case materia.assembly.AssemblyDefinition.AssemblyJointType.Prismatic:
              RobotKitRuntimeConstants.RK_RUNTIME_JOINT_PRISMATIC;
            default: throw 'Unknown assembly closure "${closure.id}" type';
          }
          var parent = -1, child = -1;
          for (index in 0...converted.model.links.length) {
            if (converted.model.links[index].id == closure.parent) parent = index;
            if (converted.model.links[index].id == closure.child) child = index;
          }
          if (parent < 0 || child < 0)
            throw 'Assembly closure "${closure.id}" references an unknown link';
          closures.push(new SimulationClosure(parent, child, type,
            closure.anchorParent, closure.axisParent));
        }
        var id = "assembly:" + assembly.id;
        if (world.robot(id) != null && simulatedIds.indexOf(id) < 0)
          throw 'Robot "$id" is remote and read-only';
        var blueprint = RobotRuntimeCompiler.compile(converted.model, appliedRevision + 1);
        var runtime = candidate.addRobotAtPose(blueprint, [0.0, 0.0, 0.0],
          [0.0, 0.0, 0.0, 1.0], null, null, collisionHulls, closures);
        candidateRobots.push(new SimulatedRobot(id, runtime, converted.model.name,
          [for (link in converted.model.links) link.id],
          [for (joint in converted.model.joints) joint.id]));
        candidateLinks.push([for (link in converted.model.links) link.id]);
        candidateRobotModels.push(converted.model);
        var robotIndex = candidateRobots.length - 1;
        for (occurrence in assembly.occurrences) {
          var center = session == null ? null : session.assemblyPreviewCenter(occurrence.definition);
          if (center == null) throw 'Assembly part "${occurrence.definition}" has no preview center';
          var linkIndex = -1;
          for (index in 0...converted.model.links.length)
            if (converted.model.links[index].id == occurrence.id) { linkIndex = index; break; }
          if (linkIndex < 0) throw 'Assembly occurrence "${occurrence.id}" has no simulated link';
          candidateAssemblyParts.push({id: "project:" + occurrence.id, robotIndex: robotIndex,
            linkIndex: linkIndex, center: [for (coordinate in center) coordinate *
              physical.metresPerUnit]});
        }
      }
      var resolvedMotions = RobotMotionPlayer.resolve(candidateMotions,
        [for (robot in candidateRobots) robot.id()], candidateRobotModels);
      var environmentRecords = scene.records();
      environmentRecords.sort(function(a, b) return Reflect.compare(a.id, b.id));
      var assemblyOwned = new Map<String, Bool>();
      if (assembly != null) for (occurrence in assembly.occurrences)
        assemblyOwned.set("project:" + occurrence.id, true);
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
      workforce = candidateWorkforce;
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

  static function rotateOffset(x:Float, y:Float, z:Float, rotation:Null<Array<Float>>):Array<Float> {
    if (rotation == null) return [x, y, z];
    var qx = rotation[0], qy = rotation[1], qz = rotation[2], qw = rotation[3];
    var tx = 2 * (qy * z - qz * y), ty = 2 * (qz * x - qx * z), tz = 2 * (qx * y - qy * x);
    return [x + qw * tx + qy * tz - qz * ty,
      y + qw * ty + qz * tx - qx * tz,
      z + qw * tz + qx * ty - qy * tx];
  }

  public function step(?timestampNs:Int64):WorldSnapshot {
    var active = space;
    if (active == null) throw "Apply the pending simulation configuration first";
    if (running) throw "Stop realtime simulation before deterministic stepping";
    feedMembers();
    presentMembers();
    active.session.step(timestampNs == null ? Int64.ofInt(0) : timestampNs);
    presentAssemblyPhysics = true;
    return world.snapshot();
  }
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
    active.session.stop(); active.session.reset(); running = false; presentAssemblyPhysics = false;
    pumpStamp = -1.0;
    // The session's own reset has restored its objects and actors; what lives beside it follows.
    for (member in members) member.reset();
    presentMembers();
    presentationEpoch++; return true;
  }
  public function isRunning():Bool return running;
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
    for (_ in 0...due) {
      feedMembers();
      active.session.stepPaced();
    }
    if (due > 0) presentMembers();
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
        sensors:robot==null?[]:robot.sensors.toArray(),links:[for (linkId in simulatedLinks[index])
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
      var link = robots[part.robotIndex].links[part.linkIndex];
      var offset = rotateOffset(part.center[0], part.center[1], part.center[2], link.rotation);
      orderedEnvironment.push({id:part.id,
        position:[link.position[0] + offset[0], link.position[1] + offset[1],
          link.position[2] + offset[2]], rotation:link.rotation});
    }
    return new ApplicationPresentationSnapshot(publication, physics, robots, orderedEnvironment,
      presentationEpoch);
  }

  public function visualRevision():Int {
    var snapshot = capturePresentationSnapshot();
    var result = snapshot.revision;
    return result;
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
    workforce = null; motions = null; refreshMembers();
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
