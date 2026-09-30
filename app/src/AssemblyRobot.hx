package app;

import RobotKitRuntime;
import cadbridge.AssemblySimulationBridge;
import materia.assembly.AssemblyDefinition;
import materia.project.MaterialLibrary;
import robotkit.model.CollisionApproximation;
import robotkit.model.RobotModel;
import robotkit.runtime.RobotRuntime;
import robotkit.runtime.RobotRuntimeBlueprint;
import robotkit.runtime.RobotRuntimeCompiler;
import robotkit.runtime.Simulation;
import robotkit.runtime.SimulationClosure;
import robotkit.world.ProcessChannelDeclaration;
import robotkit.world.SimulatedRobot;

/** Where one generated part sits on its link, for drawing the part where its link is. */
typedef AssemblyPart = {id:String, robotIndex:Int, linkIndex:Int, center:Array<Float>};

/**
 * The project's assembly as one simulated robot: a link per part, with each
 * part's mass, collision hull and closures taken from the project's physical
 * data and the scene's part objects.
 */
class AssemblyRobot {
  public final robot:SimulatedRobot;
  public final runtime:RobotRuntime;
  public final model:RobotModel;
  /** The runtime blueprint the robot was compiled to, for motion planners that need its limits. */
  public final blueprint:RobotRuntimeBlueprint;
  public final parts:Array<AssemblyPart>;
  /** Parts whose collision shape could not be made exact, each tagged with its part. */
  public final warnings:Array<String>;

  function new(robot:SimulatedRobot, runtime:RobotRuntime, model:RobotModel, blueprint:RobotRuntimeBlueprint,
      parts:Array<AssemblyPart>, warnings:Array<String>) {
    this.robot = robot;
    this.runtime = runtime;
    this.model = model;
    this.blueprint = blueprint;
    this.parts = parts;
    this.warnings = warnings;
  }

  /** The id the assembly's robot takes in the world. */
  public static function idFor(assembly:AssemblyDefinition):String return "assembly:" + assembly.id;

  /**
   * The occurrences that are not bolted to the assembly: parts flagged dynamic (a workpiece)
   * simulate as free objects, and the assembly robot has no link for them.
   */
  public static function freeOccurrences(scene:EditorScene, assembly:AssemblyDefinition):Map<String, Bool> {
    var sceneParts = new Map<String, SceneObjectData>();
    for (record in scene.records()) sceneParts.set(record.id, record);
    var free = new Map<String, Bool>();
    for (occurrence in assembly.occurrences) {
      var record = sceneParts.get("project:" + occurrence.id);
      if (record != null && record.dynamicBody) free.set(occurrence.id, true);
    }
    return free;
  }

  /**
   * Adds the assembly to `candidate` as a robot compiled at `revision`;
   * `robotIndex` is the index that robot will have among the candidate's
   * robots. Throws when the project lacks the data to simulate a part, or when
   * closures are needed and `supportsClosures` is false.
   */
  public static function add(candidate:Simulation, scene:EditorScene, session:ProjectDocumentSession,
      assembly:AssemblyDefinition, supportsClosures:Bool, revision:Int, robotIndex:Int,
      ?channels:Array<ProcessChannelDeclaration>):AssemblyRobot {
    var warnings:Array<String> = [];
    var parts:Array<AssemblyPart> = [];
    var physical = session.projectPhysical;
    if (physical == null) throw "Assembly physical properties are unavailable";
    var free = freeOccurrences(scene, assembly);
    var converted = AssemblySimulationBridge.toRobotModel(assembly, physical,
      session.projectAssemblyState, [for (id in free.keys()) id]);
    // Link collision geometry is installed with generated-part hulls in the
    // collision phase; the runtime's generic 10 cm robot box is not a part shape.
    converted.model.collisionApproximation = CollisionApproximation.None;
    var sceneParts = new Map<String, SceneObjectData>();
    for (record in scene.records()) sceneParts.set(record.id, record);
    var collisionHulls:Array<Null<Array<Float>>> = [for (_ in converted.model.links) null];
    var physicalParts = new Map<String, cadbridge.AssemblySimulationBridge.AssemblyPhysicalPart>();
    for (part in physical.parts) physicalParts.set(part.id, part);
    for (part in physical.parts) if (part.collisionWarning != null)
      warnings.push(part.id + ": " + part.collisionWarning);
    for (occurrence in assembly.occurrences) {
      if (free.exists(occurrence.id)) continue;
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
        for (material in session.customMaterials)
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
    if (converted.closureIds.length > 0 && !supportsClosures)
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
    var blueprint = RobotRuntimeCompiler.compile(converted.model, revision);
    // Process channels (a machine's spindle and coolant) must be declared before the robot is added.
    if (channels != null) for (channel in channels) blueprint.channels.push(channel);
    var runtime = candidate.addRobotAtPose(blueprint, [0.0, 0.0, 0.0],
      [0.0, 0.0, 0.0, 1.0], null, null, collisionHulls, closures);
    var robot = new SimulatedRobot(idFor(assembly), runtime, converted.model.name,
      [for (link in converted.model.links) link.id],
      [for (joint in converted.model.joints) joint.id]);
    for (occurrence in assembly.occurrences) {
      if (free.exists(occurrence.id)) continue;
      var center = session.assemblyPreviewCenter(occurrence.definition);
      if (center == null) throw 'Assembly part "${occurrence.definition}" has no preview center';
      var linkIndex = -1;
      for (index in 0...converted.model.links.length)
        if (converted.model.links[index].id == occurrence.id) { linkIndex = index; break; }
      if (linkIndex < 0) throw 'Assembly occurrence "${occurrence.id}" has no simulated link';
      parts.push({id: "project:" + occurrence.id, robotIndex: robotIndex,
        linkIndex: linkIndex, center: [for (coordinate in center) coordinate *
          physical.metresPerUnit]});
    }
    return new AssemblyRobot(robot, runtime, converted.model, blueprint, parts, warnings);
  }
}
