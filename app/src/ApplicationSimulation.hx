package app;

import haxe.Int64;
import robotkit.runtime.RobotRuntimeCompiler;
import robotkit.runtime.Simulation;
import robotkit.runtime.SimulationPresentationSnapshot;
import robotkit.world.Robot;
import robotkit.world.RobotWorld;
import robotkit.world.SimulatedRobot;
import robotkit.world.WorldSnapshot;
import robotkit.world.SensorFrame;
import robotkit.model.CollisionApproximation;
import materia.project.MaterialLibrary;
import cadbridge.AssemblySimulationBridge;

typedef SimulationRobotVisual={
  var id:String;
  var position:Array<Float>;
  var rotation:Array<Float>;
  var sensors:Array<SensorFrame>;
  var links:Array<SimulationPoseVisual>;
}
typedef SimulationPoseVisual={var id:String;var position:Array<Float>;var rotation:Array<Float>;}

/** Owns the one shared editable-scene simulation attached to the application world. */
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
  var simulation:Null<Simulation> = null;
  var simulatedIds:Array<String> = [];
  var simulatedLinks:Array<Array<String>> = [];
  var simulatedObjects:Array<{id:String,handle:Int}> = [];
  var assemblyParts:Array<{id:String,robotIndex:Int,linkIndex:Int,center:Array<Float>}> = [];
  var running:Bool = false;
  var presentAssemblyPhysics:Bool = false;
  var presentationEpoch:Int = 0;

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
      appliedEnvironmentRevision != scene.environmentRevision||appliedBackend!=backend||appliedTimestep!=timestep;

  /** Builds the complete candidate before changing any live world adapter. */
  public function rebuild(configuration:SensorConfiguration, scene:EditorScene,
      ?session:ProjectDocumentSession):Bool {
    var candidate:Null<Simulation> = null;
    var candidateRobots:Array<SimulatedRobot> = [];
    var candidateLinks:Array<Array<String>> = [];
    var candidateObjects:Array<{id:String,handle:Int}> = [];
    var candidateAssemblyParts:Array<{id:String,robotIndex:Int,linkIndex:Int,center:Array<Float>}> = [];
    try {
      var models = configuration.robotModels();
      var assembly = session == null ? null : session.projectAssemblyDefinition;
      if (models.length == 0 && assembly == null) throw "Nothing to simulate";
      for (configured in models) {
        var id=configured.id;
        var existing = world.robot(id);
        if (existing != null && simulatedIds.indexOf(id) < 0)
          throw 'Robot "$id" is remote and read-only';
      }
      candidate = new Simulation(timestep,1,backend);
      for (index in 0...models.length) {
        var editable=models[index];
        var blueprint = RobotRuntimeCompiler.compile(editable.model, appliedRevision + 1);
        var runtime = candidate.addRobotAtPose(blueprint, editable.position, editable.rotation);
        var id = editable.id;
        candidateRobots.push(new SimulatedRobot(id, runtime, editable.model.name,
          [for (link in editable.model.links) link.id], [for (joint in editable.model.joints) joint.id]));
        candidateLinks.push([for (link in editable.model.links) link.id]);
      }
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
        var collisionBoxes:Array<Null<Array<Float>>> = [for (_ in converted.model.links) null];
        var physicalParts = new Map<String, cadbridge.AssemblySimulationBridge.AssemblyPhysicalPart>();
        for (part in physical.parts) physicalParts.set(part.id, part);
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
            var center = session.assemblyPreviewCenter(occurrence.definition);
            if (center == null) throw 'Assembly part "${occurrence.definition}" has no preview center';
            var scale = physical.metresPerUnit;
            var linkIndex = converted.model.links.indexOf(link);
            collisionBoxes[linkIndex] = [record.width / 2 + Math.abs(center[0] * scale),
              record.height / 2 + Math.abs(center[1] * scale),
              record.depth / 2 + Math.abs(center[2] * scale)];
          }
        }
        if (converted.closureIds.length > 0)
          throw "Assembly closures are not supported by this simulation backend: " +
            converted.closureIds.join(", ");
        var id = "assembly:" + assembly.id;
        if (world.robot(id) != null && simulatedIds.indexOf(id) < 0)
          throw 'Robot "$id" is remote and read-only';
        var blueprint = RobotRuntimeCompiler.compile(converted.model, appliedRevision + 1);
        var runtime = candidate.addRobotAtPose(blueprint, [0.0, 0.0, 0.0],
          [0.0, 0.0, 0.0, 1.0], collisionBoxes);
        var assemblyRobotIndex = candidateRobots.length;
        for (coupling in converted.couplings) {
          var source = blueprint.identity.jointIndex(coupling.source);
          var target = blueprint.identity.jointIndex(coupling.target);
          if (source < 0 || target < 0)
            throw 'Assembly coupling "${coupling.id}" references a missing tree joint';
          var sourceKind = [for (joint in assembly.joints) if (joint.id == coupling.source) joint.type][0];
          var targetKind = [for (joint in assembly.joints) if (joint.id == coupling.target) joint.type][0];
          var sourceScale = sourceKind == materia.kinematics.AssemblyDefinition.AssemblyJointType.Prismatic
            ? physical.metresPerUnit : 1.0;
          var targetScale = targetKind == materia.kinematics.AssemblyDefinition.AssemblyJointType.Prismatic
            ? physical.metresPerUnit : 1.0;
          var ratio = coupling.ratio * targetScale / sourceScale;
          // The bridge places both joints at their saved coordinates as zero.
          // The authored absolute offset therefore cancels in runtime deltas.
          candidate.setJointCoupling(assemblyRobotIndex, source, target, ratio, 0.0);
        }
        candidateRobots.push(new SimulatedRobot(id, runtime, converted.model.name,
          [for (link in converted.model.links) link.id],
          [for (joint in converted.model.joints) joint.id]));
        candidateLinks.push([for (link in converted.model.links) link.id]);
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
        var handle=candidate.spawnBox([centerX,centerY,centerZ],
          [halfX,halfY,halfZ],object.dynamicBody,object.mass,object.rotation);
        candidateObjects.push({id:object.id,handle:handle});
      }
      if (running) candidate.start();

      var previousSimulation = simulation;
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
      simulation = candidate;
      simulatedIds = [for (robot in candidateRobots) robot.id()];
      simulatedLinks = candidateLinks;
      simulatedObjects = candidateObjects;
      assemblyParts = candidateAssemblyParts;
      appliedRevision++;
      appliedDocumentRevision = configuration.revision();
      appliedEnvironmentRevision = scene.environmentRevision;
      appliedBackend=backend;
      appliedTimestep=timestep;
      error = null;
      presentAssemblyPhysics = running;
      presentationEpoch++;
      if (previousSimulation != null) previousSimulation.dispose();
      for (robot in previousRobots) robot.close();
      return true;
    } catch (failure:Dynamic) {
      error = Std.string(failure);
      if (candidate != null && candidate != simulation) candidate.dispose();
      return false;
    }
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
    if (simulation == null) throw "Apply the pending simulation configuration first";
    if (running) throw "Stop realtime simulation before deterministic stepping";
    simulation.step(timestampNs == null ? Int64.ofInt(0) : timestampNs);
    presentAssemblyPhysics = true;
    return world.snapshot();
  }
  public function start():Void {
    if (simulation == null) throw "Apply the pending simulation configuration first";
    if (!running) { simulation.start(); presentationEpoch++; }
    running = true; presentAssemblyPhysics = true;
  }
  public function stop():Void { if (simulation != null) simulation.stop(); running = false;
    if (presentAssemblyPhysics) presentationEpoch++;
    presentAssemblyPhysics = false; }
  public function reset():Bool {
    if (simulation == null) return false;
    simulation.reset(); running = false; presentAssemblyPhysics = false;
    presentationEpoch++; return true;
  }
  public function isRunning():Bool return running;
  /** True in both running and paused simulation modes. */
  public function isActive():Bool return simulation!=null;
  public function simulatedRobotIds():Array<String> return simulatedIds.copy();
  /** Captures physics poses once, then combines the matching frame's world publications. */
  public function capturePresentationSnapshot():ApplicationPresentationSnapshot {
    var physics = simulation == null ? null : simulation.capturePresentation();
    var publication:WorldSnapshot;
    try publication = world.snapshot() catch (error:Dynamic) {
      if (physics != null) physics.dispose();
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
    var environment = new Map<String, SimulationPoseVisual>();
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
      case SimulationPresentationSnapshot.ENVIRONMENT:
        for (object in simulatedObjects) if (object.handle == pose.objectId) {
          environment.set(object.id, {id:object.id,position:pose.position,rotation:pose.rotation});
          break;
        }
      default:
    }
    var orderedEnvironment:Array<SimulationPoseVisual> = [];
    for (object in simulatedObjects) {
      var pose = environment.get(object.id);
      if (pose != null) orderedEnvironment.push(pose);
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
    for (id in simulatedIds) { var robot=world.detach(id); if(robot!=null)robot.close(); }
    simulatedIds.resize(0);
    simulatedLinks.resize(0); simulatedObjects.resize(0);
    assemblyParts.resize(0);
    presentAssemblyPhysics = false;
    if (simulation != null) simulation.dispose(); simulation = null;
    appliedDocumentRevision=-1;appliedEnvironmentRevision=-1;appliedBackend=-1;appliedTimestep=-1;
  }
  public function dispose():Void clear();
}
