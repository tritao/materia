package app;

import RobotKitRuntime;
import cadbridge.AssemblySimulationBridge;
import materia.assembly.AssemblyDefinition;
import materia.assembly.AssemblyDefinitionFlattener;
import materia.assembly.AssemblyFrames;
import materia.assembly.AssemblyRecord.AssemblyFrame;
import materia.project.SceneArtifact.SceneArtifactPlace;
import materia.project.MaterialLibrary;
import robotkit.mobile.MobileBase;
import robotkit.model.CollisionApproximation;
import robotkit.model.RobotModel;
import robotkit.runtime.RobotRuntime;
import robotkit.runtime.RobotRuntimeBlueprint;
import robotkit.runtime.RobotRuntimeCompiler;
import robotkit.runtime.Simulation;
import robotkit.runtime.SimulationClosure;
import robotkit.runtime.Simulation.SimulationLinkHull;
import robotkit.execution.ProcessChannelDeclaration;
import robotkit.simulation.SimulatedRobot;

/**
 * A simulated part: it rides link `linkIndex` of robot `robotIndex` at `offset` (metres, in the
 * link's frame; several parts of one rigid body share a link), and its scene geometry is centred at
 * `center` in its own frame.
 */
typedef AssemblyPart = {id:String, robotIndex:Int, linkIndex:Int, offset:materia.assembly.AssemblyRecord.AssemblyFrame,
  center:Array<Float>};

/**
 * The project's assembly as one simulated robot: a link per rigid body, with
 * its parts' mass, collision hulls and closures taken from the project's
 * physical data and the scene's part objects.
 */
class AssemblyRobot {
  public static function missionFrameName(at:SceneArtifactPlace):String return "mission:" + at.occurrence + "/" + at.connector;
  public final robot:SimulatedRobot;
  public final runtime:RobotRuntime;
  public final model:RobotModel;
  /** The runtime blueprint the robot was compiled to, for motion planners that need its limits. */
  public final blueprint:RobotRuntimeBlueprint;
  public final parts:Array<AssemblyPart>;
  /** The collision hull of each part that has one, in the frame of the link that carries it (metres), by occurrence id. */
  public final hulls:Array<cadbridge.AssemblySimulationBridge.AssemblyLinkHull>;
  /** Parts whose collision shape could not be made exact, each tagged with its part. */
  public final warnings:Array<String>;
  /**
   * The drive of a wheeled assembly, which takes body twists; its wheels roll the chassis over the
   * floor in the simulation. Null for an assembly fixed to the world.
   */
  public final mobile:Null<MobileBase>;
  /** The robot's root link, its chassis when it drives: the body frame of its localization. */
  public final rootLink:String;
  /** The frame of each tool's contact on the robot's model, by the contact's occurrence. */
  public final toolFrames:Map<String, robotkit.model.Frame>;
  /** Connector frames used as endpoints of planned mission chains. */
  public final missionFrames:Map<String, robotkit.model.Frame>;

  function new(robot:SimulatedRobot, runtime:RobotRuntime, model:RobotModel, blueprint:RobotRuntimeBlueprint,
      parts:Array<AssemblyPart>, hulls:Array<cadbridge.AssemblySimulationBridge.AssemblyLinkHull>, warnings:Array<String>,
      mobile:Null<MobileBase>, toolFrames:Map<String, robotkit.model.Frame>, missionFrames:Map<String, robotkit.model.Frame>) {
    this.robot = robot;
    this.runtime = runtime;
    this.model = model;
    this.blueprint = blueprint;
    this.parts = parts;
    this.hulls = hulls;
    this.warnings = warnings;
    this.mobile = mobile;
    this.toolFrames = toolFrames;
    this.missionFrames = missionFrames;
    var children = [for (joint in model.joints) joint.child];
    var roots = [for (link in model.links) if (children.indexOf(link) < 0) link];
    if (roots.length != 1) throw "The assembly robot needs exactly one root link";
    rootLink = roots[0].id;
  }

  /** The simulated part with scene id `id` (`project:<occurrence>`). */
  public function part(id:String):AssemblyPart {
    for (entry in parts) if (entry.id == id) return entry;
    throw 'Part "$id" is not part of the simulated assembly';
  }

  /** The world pose of a part's own frame, from its link's pose and its offset on that link. */
  public static function partPose(simulation:Simulation, part:AssemblyPart):{position:Array<Float>, rotation:Array<Float>}
    return compose(simulation.linkPose(part.robotIndex, part.linkIndex), part.offset);

  /** `pose` followed by `offset`: positions in metres, rotations as xyzw quaternions. */
  public static function compose(pose:{position:Array<Float>, rotation:Array<Float>},
      offset:materia.assembly.AssemblyRecord.AssemblyFrame):{position:Array<Float>, rotation:Array<Float>}
    return composeComponents(pose.position, pose.rotation, offset);

  /** Compose directly from pose components without allocating a temporary pose wrapper. */
  public static function composeComponents(position:Array<Float>, rotation:Array<Float>,
      offset:materia.assembly.AssemblyRecord.AssemblyFrame):{position:Array<Float>, rotation:Array<Float>} {
    var q = rotation;
    var x = q[0], y = q[1], z = q[2], w = q[3];
    var tx = 2 * (y * offset.z - z * offset.y);
    var ty = 2 * (z * offset.x - x * offset.z);
    var tz = 2 * (x * offset.y - y * offset.x);
    return {position: [position[0] + (offset.x + w * tx + y * tz - z * ty),
        position[1] + (offset.y + w * ty + z * tx - x * tz),
        position[2] + (offset.z + w * tz + x * ty - y * tx)],
      rotation: [w * offset.qx + x * offset.qw + y * offset.qz - z * offset.qy,
        w * offset.qy - x * offset.qz + y * offset.qw + z * offset.qx,
        w * offset.qz + x * offset.qy - y * offset.qx + z * offset.qw,
        w * offset.qw - x * offset.qx - y * offset.qy - z * offset.qz]};
  }

  /** Compose a link pose and part offset into reusable frame storage. */
  public static function composeComponentsInto(position:Array<Float>, rotation:Array<Float>,
      offset:materia.assembly.AssemblyRecord.AssemblyFrame, target:AssemblyFrame):Void {
    var x = rotation[0], y = rotation[1], z = rotation[2], w = rotation[3];
    var tx = 2 * (y * offset.z - z * offset.y);
    var ty = 2 * (z * offset.x - x * offset.z);
    var tz = 2 * (x * offset.y - y * offset.x);
    target.x = position[0] + offset.x + w * tx + y * tz - z * ty;
    target.y = position[1] + offset.y + w * ty + z * tx - x * tz;
    target.z = position[2] + offset.z + w * tz + x * ty - y * tx;
    target.qx = w * offset.qx + x * offset.qw + y * offset.qz - z * offset.qy;
    target.qy = w * offset.qy - x * offset.qz + y * offset.qw + z * offset.qx;
    target.qz = w * offset.qz + x * offset.qy - y * offset.qx + z * offset.qw;
    target.qw = w * offset.qw - x * offset.qx - y * offset.qy - z * offset.qz;
  }

  /** `v` turned by the xyzw quaternion `q`. */
  public static function rotate(q:Array<Float>, v:Array<Float>):Array<Float> {
    var x = q[0], y = q[1], z = q[2], w = q[3];
    var tx = 2 * (y * v[2] - z * v[1]), ty = 2 * (z * v[0] - x * v[2]), tz = 2 * (x * v[1] - y * v[0]);
    return [v[0] + w * tx + y * tz - z * ty, v[1] + w * ty + z * tx - x * tz, v[2] + w * tz + x * ty - y * tx];
  }

  /**
   * The frame of the connector `at` names, in its occurrence's own frame and the assembly's CAD
   * units, whichever occurrence of a shared definition it is.
   */
  public static function connectorFrame(assembly:AssemblyDefinition, at:SceneArtifactPlace):AssemblyFrame {
    var flat = AssemblyDefinitionFlattener.flatten(assembly);
    var definitionId:Null<String> = null;
    for (item in flat.occurrences) if (item.id == at.occurrence) definitionId = item.definition;
    if (definitionId == null) throw 'The assembly has no occurrence "${at.occurrence}"';
    for (component in flat.definitions) if (component.id == definitionId)
      for (connector in component.connectors) if (connector.name == at.connector) return connector.frame;
    throw 'Occurrence "${at.occurrence}" has no connector "${at.connector}"';
  }

  /** The id the assembly's robot takes in the world. */
  public static function idFor(assembly:AssemblyDefinition):String return "assembly:" + assembly.id;

  /**
   * The occurrences that are not part of the assembly robot: parts flagged dynamic (a workpiece),
   * and a mobile robot's surroundings, which simulate as objects of their own, moving or held where
   * they stand; the assembly robot has no link for them.
   */
  public static function unownedOccurrences(scene:EditorScene, session:ProjectDocumentSession,
      assembly:AssemblyDefinition):Map<String, Bool> {
    var result = freeOccurrences(scene, assembly);
    var mobile = session.mobileBase;
    if (mobile != null && mobile.robot != null) {
      var prefix = mobile.robot + "/";
      for (occurrence in materia.assembly.AssemblyDefinitionFlattener.flatten(assembly).occurrences)
        if (!StringTools.startsWith(occurrence.id, prefix)) result.set(occurrence.id, true);
    }
    return result;
  }

  /** The occurrences flagged dynamic, which simulate as free objects. */
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
  /**
   * The link pairs the simulation leaves out, from the same collision description the planner and the
   * editor use (COLLISION.md CL3b): adjacent and rigidly joined links, and links whose hulls overlap in
   * the robot's starting configuration (its joints start at zero, the loaded state). SimKit's own rule
   * (bounding spheres overlapping at rest) would leave out more and check fewer real contacts.
   */
  static function simulationExcludes(model:RobotModel, hulls:Array<SimulationLinkHull>):Array<Int> {
    var description = new collisionkit.CollisionDescription();
    var options = new robotkit.collision.RobotCollision.RobotCollisionOptions("robot");
    options.hulls = [for (index in 0...hulls.length)
      new robotkit.collision.RobotCollision.RobotLinkHull(hulls[index].link, 'hull$index', hulls[index].vertices)];
    var robot = robotkit.collision.RobotCollision.describe(description, model, options);
    var world = new collisionkit.native.NativeCollisionWorld();
    try {
      var build = description.build(world);
      var excludes = robotkit.collision.RobotCollision.simulationExcludes(description, build, robot);
      world.dispose();
      return excludes;
    } catch (error:Dynamic) {
      world.dispose();
      throw error;
    }
  }

  public static function add(candidate:Simulation, scene:EditorScene, session:ProjectDocumentSession,
      assembly:AssemblyDefinition, supportsClosures:Bool, revision:Int, robotIndex:Int,
      ?channels:Array<ProcessChannelDeclaration>, virtualWelder:Bool = false):AssemblyRobot {
    var warnings:Array<String> = [];
    var parts:Array<AssemblyPart> = [];
    var physical = session.projectPhysical;
    if (physical == null) throw "Assembly physical properties are unavailable";
    var free = unownedOccurrences(scene, session, assembly);
    var sceneParts = new Map<String, SceneObjectData>();
    for (record in scene.records()) sceneParts.set(record.id, record);
    var physicalParts = new Map<String, cadbridge.AssemblySimulationBridge.AssemblyPhysicalPart>();
    for (part in physical.parts) physicalParts.set(part.id, part);
    for (part in physical.parts) if (part.collisionWarning != null)
      warnings.push(part.id + ": " + part.collisionWarning);
    // A part the user gave another material keeps its volume and takes that material's density.
    var masses = new Map<String, Float>();
    for (occurrence in assembly.occurrences) {
      if (free.exists(occurrence.id)) continue;
      var record = sceneParts.get("project:" + occurrence.id);
      var part = physicalParts.get(occurrence.definition);
      if (record == null || part == null)
        throw 'Assembly occurrence "${occurrence.id}" is missing its generated part';
      if (record.collisionEnabled && (part.collisionHull == null || part.collisionHull.length < 12))
        throw 'Assembly part "${occurrence.definition}" has no convex collision hull';
      var baseMass = part.volume * part.density * Math.pow(physical.metresPerUnit, 3);
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
      masses.set(occurrence.id, chosenMass);
    }
    var converted = AssemblySimulationBridge.toRobotModel(assembly, physical,
      session.projectAssemblyState, [for (id in freeOccurrences(scene, assembly).keys()) id], id -> masses.get(id), session.mobileBase);
    // Link collision geometry is installed with generated-part hulls in the
    // collision phase; the runtime's generic 10 cm robot box is not a part shape.
    converted.model.collisionApproximation = CollisionApproximation.None;
    // Each part collides through its own hull on its body's link, unless the user turned it off.
    // A wheeled base rolls by its drive plant, which stands for its wheels' contact with the floor: their
    // hulls would only chatter against the floor and kick the wheel joints the odometry reads.
    var rolling = new Map<Int, Bool>();
    if (converted.profile.mobileBase != null) switch (converted.profile.mobileBase.drive) {
      case Differential(left, right, _, _):
        for (joint in converted.model.joints)
          if (joint.id == left || joint.id == right) rolling.set(converted.model.links.indexOf(joint.child), true);
      case _:
    }
    var linkHulls:Array<SimulationLinkHull> = [];
    for (hull in converted.linkHulls) {
      var record = sceneParts.get("project:" + hull.part);
      if (record != null && record.collisionEnabled && !rolling.exists(hull.link))
        linkHulls.push({link: hull.link, vertices: hull.vertices});
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
        case materia.assembly.AssemblyDefinition.AssemblyJointType.Spherical:
          RobotKitRuntimeConstants.RK_RUNTIME_JOINT_SPHERICAL;
        case materia.assembly.AssemblyDefinition.AssemblyJointType.Cylindrical:
          RobotKitRuntimeConstants.RK_RUNTIME_JOINT_CYLINDRICAL;
        case materia.assembly.AssemblyDefinition.AssemblyJointType.Planar:
          RobotKitRuntimeConstants.RK_RUNTIME_JOINT_PLANAR;
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
    // Each tool reports on its sensor (a suction tool's vacuum, a torch's weld circuit), mounted at the tool's
    // contact on the link that carries it.
    for (tool in session.robotTools) {
      var sensorId = tool.sensor;
      if (sensorId == null) continue;
      var carrier = converted.partLinks.get(tool.contact.occurrence);
      if (carrier == null) throw 'Robot tool "${tool.contact.occurrence}" is not part of the robot';
      var kind = tool.kind == "torch" ? processkit.tool.WeldSensor.KIND : "tool_vacuum_kpa";
      var sensor = converted.model.addSensor(new robotkit.model.Sensor(sensorId, kind, 0.0, sensorId));
      var mount = converted.model.addFrame(new robotkit.model.Frame(sensorId + " mount", converted.model.links[carrier.link]));
      mount.position = [carrier.offset.x, carrier.offset.y, carrier.offset.z];
      sensor.frame = mount;
    }
    // A frame on a link at a connector of the part riding it, in the link's frame and metres.
    function mountFrame(name:String, at:SceneArtifactPlace):robotkit.model.Frame {
      var carrier = converted.partLinks.get(at.occurrence);
      if (carrier == null) throw '"$name" is not mounted on a part of the robot';
      var connector = connectorFrame(assembly, at);
      var scale = physical.metresPerUnit;
      var placed = AssemblyFrames.compose(carrier.offset,
        {x: connector.x * scale, y: connector.y * scale, z: connector.z * scale,
          qx: connector.qx, qy: connector.qy, qz: connector.qz, qw: connector.qw});
      var frame = converted.model.addFrame(new robotkit.model.Frame(name, converted.model.links[carrier.link]));
      frame.position = [placed.x, placed.y, placed.z];
      frame.rotation = [placed.qx, placed.qy, placed.qz, placed.qw];
      return frame;
    }
    // Each tool's contact is a frame of the robot, where its arm's programs put the tool.
    var toolFrames = new Map<String, robotkit.model.Frame>();
    for (tool in session.robotTools)
      toolFrames.set(tool.contact.occurrence, mountFrame(tool.contact.occurrence + " contact", tool.contact));
    // Each scanner scans from its mount connector.
    for (scanner in session.robotSensors) {
      var sensor = converted.model.addSensor(new robotkit.model.Sensor(scanner.id, scanner.kind, scanner.updateRate, scanner.id));
      sensor.rayCount = scanner.rayCount;
      sensor.maxRange = scanner.maxRange;
      sensor.frame = mountFrame(scanner.id + " mount", scanner.mount);
    }
    var missionFrames = new Map<String, robotkit.model.Frame>();
    if (session.mission != null) for (step in session.mission.steps) if (step.kind == "moveJoints") {
      var at:SceneArtifactPlace = cast step.at;
      var name = missionFrameName(at);
      if (!missionFrames.exists(name)) missionFrames.set(name, mountFrame(name, at));
    }
    var machine = session.machineMotion;
    var device:Null<robotkit.runtime.VirtualDeviceOptions> = null;
    if (virtualWelder) device = robotkit.runtime.VirtualServoOptions.fromModel(converted.model);
    else if (machine != null && machine.virtualDevice == true) {
      var binding = robotkit.device.DeviceBinding.bind(converted.model,
        robotkit.device.DeviceLayout.forActuators(converted.model), 40000);
      converted.model = binding.model;
      device = new robotkit.runtime.VirtualDeviceOptions();
      device.actuators = binding.virtualActuators();
    }
    if (machine != null && machine.program != null) converted.model.materializeLimits(true);
    if (virtualWelder) {
      // The device's braking bound is the same arm-program policy, rather than an invented motor acceleration.
      for (joint in converted.model.joints) {
        var acceleration = joint.limits.maxAcceleration;
        joint.limits.maxAcceleration = acceleration == null ? MissionPlayer.ARM_ACCELERATION
          : Math.min(acceleration, MissionPlayer.ARM_ACCELERATION);
      }
    }
    var blueprint = RobotRuntimeCompiler.compile(converted.model, converted.profile, revision);
    // Process channels (a machine's spindle and coolant, a tool's vacuum) must be declared before the
    // robot is added.
    if (channels != null) for (channel in channels) blueprint.addChannel(channel);
    // Each robot tool brings its own channels with the stop policy it needs (RobotKit's `ToolChannels`): a suction tool
    // keeps holding through a commanded stop, as when its base arrives somewhere carrying a part; a torch's arc and wire
    // go off on any stop. A declaration made above that disagrees with the tool is refused.
    for (tool in session.robotTools) {
      var welder = tool.torch;
      blueprint.addTool(welder == null ? new robotkit.tool.SuctionChannels(tool.channel)
        : new processkit.tool.WeldChannels(tool.channel, welder.wireSpeedChannel, welder.voltageChannel));
    }
    if (virtualWelder) {
      var torches = [for (tool in session.robotTools) if (tool.torch != null) tool];
      if (torches.length != 1 || device == null) throw "A virtual welding device needs exactly one torch";
      var torch = torches[0];
      var welding = torch.torch;
      if (welding == null) throw "Virtual welding profile is missing";
      processkit.simulation.VirtualWelderSupply.configure(device, blueprint,
        {arc:torch.channel, wireSpeed:welding.wireSpeedChannel, voltage:welding.voltageChannel}, welding.efficiency);
    }
    // A mobile robot stands at its origin on the floor; its root link is framed there.
    var origin = session.mobileBase == null ? null : session.mobileBase.origin;
    var position = origin == null ? [0.0, 0.0, 0.0] : [origin.x, origin.y, 0.0];
    var rotation = origin == null ? [0.0, 0.0, 0.0, 1.0] : [0.0, 0.0, Math.sin(origin.yaw / 2), Math.cos(origin.yaw / 2)];
    // Like a machine whose servos are on, its joints hold their designed pose until something commands
    // them, such as an arm while its base drives.
    var runtime = candidate.addRobotAtPose(blueprint, position, rotation, device, null, null, closures, null, null, null, null,
      linkHulls, true, simulationExcludes(converted.model, linkHulls));
    var authored:Null<Array<materia.project.SceneArtifact.SceneArtifactPowerUpOffset>> = session.cncJob != null ? session.cncJob.powerUpOffsets :
      session.mission == null ? null : session.mission.powerUpOffsets;
    var sideOffsets:Null<Array<materia.project.SceneArtifact.SceneArtifactPowerUpSideOffset>> = session.cncJob != null ? session.cncJob.powerUpSideOffsets :
      session.mission == null ? null : session.mission.powerUpSideOffsets;
    if ((authored != null && authored.length > 0) || (sideOffsets != null && sideOffsets.length > 0)) {
      var named = new Map<String, Float>();
      if (authored != null) for (entry in authored) {
        if (entry == null || named.exists(entry.joint)) throw "Duplicate or null scene power-up offset";
        named.set(entry.joint, entry.offset);
      }
      var base = robotkit.runtime.PowerUpOffsets.resolve(blueprint, named);
      if (sideOffsets != null && sideOffsets.length > 0) {
        var sides = new Map<String, Float>();
        for (entry in sideOffsets) {
          if (entry == null || sides.exists(entry.homeSwitch)) throw "Duplicate or null scene side offset";
          sides.set(entry.homeSwitch, entry.offset);
        }
        var placement = robotkit.runtime.PowerUpSideOffsets.resolve(blueprint, base, sides);
        candidate.setPowerUpSides(robotIndex, placement.offsets, placement.drives);
      } else candidate.setPowerUpOffsets(robotIndex, base);
    }
    var robot = new SimulatedRobot(idFor(assembly), runtime, converted.model.name,
      [for (link in converted.model.links) link.id],
      [for (joint in converted.model.joints) joint.id]);
    // A wheeled assembly's chassis rolls by the wheel rates the robot applies each tick, whoever commands them.
    var mobile:Null<MobileBase> = null;
    if (converted.profile.mobileBase != null) {
      mobile = MobileBase.fromBlueprint(robot, blueprint);
      var odometry = mobile.driveModel.createOdometry();
      if (odometry == null) throw "Only differential-drive assemblies can drive in the simulation";
      candidate.setDifferentialDrive(robotIndex, odometry.leftWheelJoint, odometry.rightWheelJoint,
        odometry.wheelRadius, odometry.trackWidth, odometry.leftDirection, odometry.rightDirection);
    }
    for (occurrence in assembly.occurrences) {
      if (free.exists(occurrence.id)) continue;
      var center = session.assemblyPreviewCenter(occurrence.definition);
      if (center == null) throw 'Assembly part "${occurrence.definition}" has no preview center';
      var placed = converted.partLinks.get(occurrence.id);
      if (placed == null) throw 'Assembly occurrence "${occurrence.id}" has no simulated link';
      parts.push({id: "project:" + occurrence.id, robotIndex: robotIndex, linkIndex: placed.link,
        offset: placed.offset, center: [for (coordinate in center) coordinate * physical.metresPerUnit]});
    }
    return new AssemblyRobot(robot, runtime, converted.model, blueprint, parts, converted.linkHulls, warnings, mobile,
      toolFrames, missionFrames);
  }
}
