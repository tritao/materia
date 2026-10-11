package app;

import RobotKitRuntime;
import cadbridge.AssemblySimulationBridge;
import materia.assembly.AssemblyDefinition;
import materia.assembly.AssemblyDefinitionFlattener;
import materia.assembly.AssemblyFrames;
import materia.assembly.AssemblyRecord.AssemblyFrame;
import materia.project.SceneArtifact.SceneArtifactPlace;
import materia.project.SceneArtifact.SceneArtifactCellController;
import materia.project.MaterialLibrary;
import robotkit.mobile.MobileBase;
import robotkit.model.CollisionApproximation;
import robotkit.model.CollisionShape.ContactSurface;
import robotkit.model.CollisionShape.ShapeContact;
import robotkit.model.RobotModel;
import robotkit.model.Transmission;
import robotkit.runtime.RobotRuntime;
import robotkit.runtime.RobotRuntimeBlueprint;
import robotkit.runtime.RobotRuntimeCompiler;
import robotkit.runtime.Simulation;
import robotkit.runtime.SimulationClosure;
import robotkit.runtime.RobotRuntimeBlueprint.RobotRuntimeLinkShape;
import robotkit.execution.ProcessChannelDeclaration;
import robotkit.execution.ProcessEventValue;
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
  public final shapes:Array<RobotRuntimeLinkShape>;
  /** Part pairs that meet as designed interfaces across moving joints. */
  public final jointInterfaces:Array<cadbridge.AssemblySimulationBridge.AssemblyJointInterface>;
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
  /** The heaters of a printer's virtual device, or null when the assembly has none. */
  public final heaters:Null<robotkit.runtime.VirtualHeaters>;

  function new(robot:SimulatedRobot, runtime:RobotRuntime, model:RobotModel, blueprint:RobotRuntimeBlueprint,
      parts:Array<AssemblyPart>, shapes:Array<RobotRuntimeLinkShape>,
      jointInterfaces:Array<cadbridge.AssemblySimulationBridge.AssemblyJointInterface>, warnings:Array<String>,
      mobile:Null<MobileBase>, toolFrames:Map<String, robotkit.model.Frame>, missionFrames:Map<String, robotkit.model.Frame>,
      heaters:Null<robotkit.runtime.VirtualHeaters>) {
    this.heaters = heaters;
    this.robot = robot;
    this.runtime = runtime;
    this.model = model;
    this.blueprint = blueprint;
    this.parts = parts;
    this.shapes = shapes;
    this.jointInterfaces = jointInterfaces;
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
  public static function idFor(assembly:AssemblyDefinition, ?controllerId:String):String
    return "assembly:" + assembly.id + (controllerId == null ? "" : ":" + controllerId);

  /**
   * The occurrences that are not part of the assembly robot: parts flagged dynamic (a workpiece),
   * and a mobile robot's surroundings, which simulate as objects of their own, moving or held where
   * they stand; the assembly robot has no link for them.
   */
  static function unownedOccurrences(session:ProjectDocumentSession, assembly:AssemblyDefinition,
      free:Map<String, Bool>):Map<String, Bool> {
    var result = new Map<String, Bool>();
    for (id in free.keys()) result.set(id, true);
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
    for (occurrence in AssemblyDefinitionFlattener.flatten(assembly).occurrences) {
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
  static function simulationExcludes(model:RobotModel, shapes:Array<RobotRuntimeLinkShape>):Array<Int> {
    var description = new collisionkit.CollisionDescription();
    var options = new robotkit.collision.RobotCollision.RobotCollisionOptions("robot");
    var saved = [for (link in model.links) link.collisionShapes];
    for (link in model.links) link.collisionShapes = [];
    for (source in shapes) model.links[source.link].collisionShapes.push(source.shape);
    var robot:robotkit.collision.RobotCollision.RobotCollisionBodies;
    try {
      robot = robotkit.collision.RobotCollision.describe(description, model, options);
    } catch (error:Dynamic) {
      for (i in 0...model.links.length) model.links[i].collisionShapes = saved[i];
      throw error;
    }
    for (i in 0...model.links.length) model.links[i].collisionShapes = saved[i];
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
      ?channels:Array<ProcessChannelDeclaration>, virtualWelder:Bool = false, ?control:SceneArtifactCellController,
      ?restContactGroup:haxe.Int64):AssemblyRobot {
    var warnings:Array<String> = [];
    var parts:Array<AssemblyPart> = [];
    var physical = session.projectPhysical;
    if (physical == null) throw "Assembly physical properties are unavailable";
    var free = freeOccurrences(scene, assembly);
    var unowned = unownedOccurrences(session, assembly, free);
    var sceneParts = new Map<String, SceneObjectData>();
    for (record in scene.records()) sceneParts.set(record.id, record);
    var physicalParts = new Map<String, cadbridge.AssemblySimulationBridge.AssemblyPhysicalPart>();
    for (part in physical.parts) physicalParts.set(part.id, part);
    for (part in physical.parts) if (part.collisionWarning != null)
      warnings.push(part.id + ": " + part.collisionWarning);
    // A part the user gave another material keeps its volume and takes that material's density.
    var masses = new Map<String, Float>();
    for (occurrence in assembly.occurrences) {
      if (unowned.exists(occurrence.id)) continue;
      var record = sceneParts.get("project:" + occurrence.id);
      var part = physicalParts.get(occurrence.definition);
      if (record == null || part == null)
        throw 'Assembly occurrence "${occurrence.id}" is missing its generated part';
      if (record.collisionEnabled && (part.collisionHull == null || part.collisionHull.length < 12))
        throw 'Assembly part "${occurrence.definition}" has no convex collision hull';
      var baseMass = part.volume * part.density * Math.pow(physical.metresPerUnit, 3);
      var chosenMass = record.mass;
      if (part.physicalMassDeclared != true && record.materialId != null && record.materialId != part.materialId &&
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
    var ownedJoints = control == null ? null : control.joints;
    if (ownedJoints != null) {
      var movable = [for (joint in AssemblyDefinitionFlattener.flatten(assembly).joints)
        if (Std.string(joint.role) == "tree" && Std.string(joint.type) != "fixed") joint.id];
      if (movable.length == ownedJoints.length) ownedJoints = null;
    }
    var converted = AssemblySimulationBridge.toRobotModel(assembly, physical,
      session.projectAssemblyState, [for (id in free.keys()) id], id -> masses.get(id),
      control == null ? session.mobileBase : null,
      ownedJoints, control == null || ownedJoints == null ? null : control.sensors);
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
    var contacts = new Map<String, materia.project.SceneContactSurface>();
    var motion = session.machineMotion;
    if (motion != null && motion.contacts != null)
      for (contact in motion.contacts) contacts.set(contact.occurrence, contact.surface);
    var simulationShapes:Array<RobotRuntimeLinkShape> = [];
    // CAD physical-part hulls retain their exact convex geometry. A pneumatic
    // joint's connected links need an explicit contact response: the simulator's
    // generic soft-contact default lets cylinder force push a workpiece through
    // a jaw before its position switch trips. Keep that response local instead
    // of stiffening every part in the machine.
    var pneumaticContactLinks = new Map<Int, Bool>();
    var hasPneumaticActuator = false;
    for (actuator in converted.model.actuators)
      if (actuator.drive != null && actuator.drive.pneumatic() != null) {
        hasPneumaticActuator = true;
        switch actuator.transmission {
          case SimpleTransmission(jointId, _, _):
            var pending = [jointId];
            var visited = new Map<String, Bool>();
            while (pending.length > 0) {
              var current:Null<String> = pending.pop();
              if (current == null) continue;
              if (visited.exists(current)) continue;
              visited.set(current, true);
              var joint = [for (candidate in converted.model.joints) if (candidate.id == current) candidate];
              if (joint.length != 1) throw 'Pneumatic drive references missing joint "$current"';
              pneumaticContactLinks.set(converted.model.links.indexOf(joint[0].parent), true);
              pneumaticContactLinks.set(converted.model.links.indexOf(joint[0].child), true);
              // Pneumatic force enters through the driven coordinate, but every
              // mechanically coupled jaw must stop against the workpiece with
              // the same contact response.
              for (coupling in converted.model.couplings)
                if (coupling.leader == current) pending.push(coupling.follower);
            }
          case _:
        }
    }
    var pneumaticContact = hasPneumaticActuator ? SimulationContactSurfaces.robotPart() : null;
    var fingerPadSurfaces = new Map<String, robotkit.model.CollisionShape.ContactSurface>();
    for (tool in session.robotTools) if (tool.kind == "gripper") {
      var gripper = tool.gripper;
      if (gripper == null) throw 'Gripper tool "${tool.contact.occurrence}" has no physical pad description';
      var surface = SimulationContactSurfaces.gripperFinger(gripper.fingerPadFrictionCoefficient);
      for (member in gripper.padMembers) fingerPadSurfaces.set(member, surface);
    }
    for (hull in cadbridge.AssemblySimulationBridge.shapes(converted.model)) {
      var record = sceneParts.get("project:" + hull.shape.name);
      if (record != null && record.collisionEnabled && !rolling.exists(hull.link)) {
        var simulationShape = hull.shape;
        var surface = contacts.get(hull.shape.name);
        if (surface != null) {
          simulationShape.surface = new ContactSurface(surface.friction.copy(), surface.frictionDimensions,
            surface.contactTimeConstant, surface.contactDampingRatio);
          simulationShape.contact = switch surface.filter {
            case Layers: ShapeContact.Layers;
            case PairsOnly: ShapeContact.PairsOnly;
            case PairsAndEnvironment: ShapeContact.PairsAndEnvironment;
          };
        } else {
          var padSurface = fingerPadSurfaces.get(hull.shape.name);
          if (padSurface != null) simulationShape.surface = padSurface;
          else if (pneumaticContact != null && pneumaticContactLinks.exists(hull.link))
            simulationShape.surface = pneumaticContact;
        }
        simulationShapes.push(hull);
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
      if (control != null && converted.partLinks.get(tool.contact.occurrence) == null) continue;
      var sensorId = tool.sensor;
      if (sensorId == null) continue;
      var carrier = converted.partLinks.get(tool.contact.occurrence);
      if (carrier == null) throw 'Robot tool "${tool.contact.occurrence}" is not part of the robot';
      if (tool.kind == "gripper") {
        var physical = [for (sensor in converted.model.sensors) if (sensor.id == sensorId) sensor];
        if (physical.length != 1 || physical[0].kind != "joint_switch")
          throw 'Gripper sensor "$sensorId" must be a joint switch in the robot assembly';
      } else {
        var kind = tool.kind == "screwdriver" ? robotkit.tool.ScrewdriverContract.SENSOR_KIND :
          tool.kind == "torch" ? processkit.tool.WeldSensor.KIND : "tool_vacuum_kpa";
        var sensor = converted.model.addSensor(new robotkit.model.Sensor(sensorId, kind, 0.0, sensorId));
        var mount = converted.model.addFrame(new robotkit.model.Frame(sensorId + " mount", converted.model.links[carrier.link]));
        mount.position = [carrier.offset.x, carrier.offset.y, carrier.offset.z];
        sensor.frame = mount;
      }
    }
    // A print job's heaters report on sensors of their own, mounted on the parts that carry the thermistors. In a cell
    // of several controllers, the job runs on the one whose robot drives the extruder.
    var printJob = session.printJob;
    if (printJob != null && control != null && [for (joint in converted.model.joints) if (joint.id == printJob.extruder) joint].length == 0)
      printJob = null;
    if (printJob != null) for (heater in printJob.heaters) {
      var carrier = converted.partLinks.get(heater.mount);
      if (carrier == null) throw 'Heater "${heater.name}" is mounted on "${heater.mount}", which is not part of the printer';
      var sensor = converted.model.addSensor(new robotkit.model.Sensor(heater.sensor, robotkit.tool.HeaterSensor.KIND, 0.0, heater.sensor));
      var mount = converted.model.addFrame(new robotkit.model.Frame(heater.sensor + " mount", converted.model.links[carrier.link]));
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
      if (control == null || converted.partLinks.get(tool.contact.occurrence) != null)
        toolFrames.set(tool.contact.occurrence, mountFrame(tool.contact.occurrence + " contact", tool.contact));
    // Each scanner scans from its mount connector.
    for (scanner in session.robotSensors) {
      if (control != null && converted.partLinks.get(scanner.mount.occurrence) == null) continue;
      var sensor = converted.model.addSensor(new robotkit.model.Sensor(scanner.id, scanner.kind, scanner.updateRate, scanner.id));
      sensor.rayCount = scanner.rayCount;
      sensor.maxRange = scanner.maxRange;
      sensor.frame = mountFrame(scanner.id + " mount", scanner.mount);
    }
    if (control != null) for (wire in session.signals)
      if (wire.to.controller == control.id)
        converted.model.addSensor(new robotkit.model.Sensor(wire.name, "digital_input", 0.0, wire.name));
    var missionFrames = new Map<String, robotkit.model.Frame>();
    if (session.mission != null && (control == null || session.mission.controllerId == control.id))
      for (step in session.mission.steps) if (step.kind == "moveJoints" || step.kind == "moveVia") {
      var at:SceneArtifactPlace = cast step.at;
      var name = missionFrameName(at);
      if (!missionFrames.exists(name)) missionFrames.set(name, mountFrame(name, at));
    }
    var machine = session.machineMotion;
    var device:Null<robotkit.runtime.VirtualDeviceOptions> = null;
    var extruder:Null<robotkit.runtime.VirtualHeaters.HeaterExtruder> = null;
    if (virtualWelder) device = robotkit.runtime.VirtualServoOptions.fromModel(converted.model);
    else if (printJob != null) {
      // The printer's heaters run on the device. Its board has no switch inputs for a CoreXY (a switch joint driven by
      // two motors), so the machine is referenced where its initial pose has it, and its home switches are not wired.
      converted.model.switches.resize(0);
      var binding = robotkit.device.DeviceBinding.bind(converted.model,
        robotkit.device.DeviceLayout.forActuators(converted.model), printJob.controller.stepTickHz);
      converted.model = binding.model;
      device = new robotkit.runtime.VirtualDeviceOptions();
      device.actuators = binding.virtualActuators();
      device.baud = PrintProgramPlayer.LINK_BAUD;
      extruder = PrintProgramPlayer.extruderOnDevice(binding, printJob.extruder);
    } else if (machine != null && machine.virtualDevice == true) {
      var binding = robotkit.device.DeviceBinding.bind(converted.model,
        robotkit.device.DeviceLayout.forActuators(converted.model), 40000);
      converted.model = binding.model;
      device = new robotkit.runtime.VirtualDeviceOptions();
      device.actuators = binding.virtualActuators();
    }
    if (machine != null && (machine.program != null || machine.pressBrake != null)) converted.model.materializeLimits(true);
    if (virtualWelder) {
      // The device's braking bound is the same arm-program policy, rather than an invented motor acceleration.
      for (joint in converted.model.joints) {
        var acceleration = joint.limits.maxAcceleration;
        joint.limits.maxAcceleration = acceleration == null ? MissionPlayer.ARM_ACCELERATION
          : Math.min(acceleration, MissionPlayer.ARM_ACCELERATION);
      }
    }
    var blueprint = RobotRuntimeCompiler.compile(converted.model, converted.profile, revision);
    blueprint.linkCollisionShapes.resize(0);
    for (source in simulationShapes) blueprint.linkCollisionShapes.push(source);
    if (control != null) for (name in control.channels) {
      var found = false;
      for (entry in blueprint.channels) if (entry.id == name) found = true;
      if (!found) blueprint.addChannel(new ProcessChannelDeclaration(name, ProcessEventValue.Digital(false)));
    }
    // Process channels (a machine's spindle and coolant, a tool's vacuum) must be declared before the
    // robot is added.
    if (channels != null) for (channel in channels) blueprint.addChannel(channel);
    // Each robot tool brings its own channels with the stop policy it needs (RobotKit's `ToolChannels`): a suction tool
    // keeps holding through a commanded stop, as when its base arrives somewhere carrying a part; a torch's arc and wire
    // go off on any stop. A declaration made above that disagrees with the tool is refused.
    for (tool in session.robotTools) {
      if (control != null && converted.partLinks.get(tool.contact.occurrence) == null) continue;
      var driver = tool.screwdriver;
      if (driver != null) {
        for (name in [driver.feedJoint, driver.spindleJoint]) {
          var index = -1;
          for (i in 0...converted.model.joints.length) if (converted.model.joints[i].id == name) index = i;
          if (index < 0) throw 'Screwdriver process joint "$name" is absent';
          blueprint.processModelJoints.push(index);
        }
        blueprint.addTool(new robotkit.tool.ScrewdriverChannels(tool.channel, driver.directionChannel,
          driver.programChannel, driver.vacuumChannel));
        continue;
      }
      var welder = tool.torch;
      blueprint.addTool(welder != null
        ? new processkit.tool.WeldChannels(tool.channel, welder.wireSpeedChannel, welder.voltageChannel)
        : tool.kind == "gripper" ? new robotkit.tool.GripperChannels(tool.channel)
        : new robotkit.tool.SuctionChannels(tool.channel));
    }
    var heaterSetups:Array<robotkit.runtime.VirtualHeaters.HeaterSetup> = [];
    if (printJob != null && device != null) {
      heaterSetups = PrintProgramPlayer.heaterSetups(printJob, extruder);
      for (heater in printJob.heaters) blueprint.addTool(new robotkit.tool.HeaterChannels(heater.target, heater.fan));
      robotkit.runtime.VirtualHeaters.configure(device, blueprint, heaterSetups);
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
    var runtime = candidate.addRobotAtPose(blueprint, position, rotation, device, null, closures, null, null, null, null,
      true, simulationExcludes(converted.model, simulationShapes), restContactGroup);
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
    var robot = new SimulatedRobot(idFor(assembly,
      control == null || session.controllers.length == 1 ? null : control.id), runtime, converted.model.name,
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
      if (unowned.exists(occurrence.id)) continue;
      var placed = converted.partLinks.get(occurrence.id);
      if (control != null && placed == null) continue;
      var center = session.assemblyPreviewCenter(occurrence.definition);
      if (center == null) throw 'Assembly part "${occurrence.definition}" has no preview center';
      if (placed == null) throw 'Assembly occurrence "${occurrence.id}" has no simulated link ' +
        '(controller=${control == null ? "all" : control.id}, controllers=${session.controllers.length}, ' +
        'robot=${converted.model.name}, owned joints=${ownedJoints == null ? "all" : ownedJoints.join(",")})';
      parts.push({id: "project:" + occurrence.id, robotIndex: robotIndex, linkIndex: placed.link,
        offset: placed.offset, center: [for (coordinate in center) coordinate * physical.metresPerUnit]});
    }
    var heaters = printJob == null ? null : new robotkit.runtime.VirtualHeaters(candidate, runtime, robotIndex, heaterSetups);
    return new AssemblyRobot(robot, runtime, converted.model, blueprint, parts, cadbridge.AssemblySimulationBridge.shapes(converted.model),
      converted.jointInterfaces, warnings, mobile,
      toolFrames, missionFrames, heaters);
  }
}
