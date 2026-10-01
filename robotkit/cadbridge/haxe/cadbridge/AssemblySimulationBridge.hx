package cadbridge;

import materia.assembly.AssemblyDefinition;
import materia.assembly.AssemblyDefinition.AssemblyJointRole;
import materia.assembly.AssemblyDefinition.AssemblyJointType;
import materia.assembly.AssemblyDefinition.AssemblyStateRecord;
import materia.assembly.AssemblyDefinitionCodec;
import materia.assembly.AssemblyDefinitionFlattener;
import materia.assembly.AssemblyFrames;
import materia.assembly.AssemblyRecord.AssemblyFrame;
import robotkit.model.RobotModel;
import robotkit.model.Link;
import robotkit.model.Joint;
import robotkit.model.JointType;
import robotkit.model.JointLimits;
import robotkit.model.JointCoupling;
import cadkit.modeling.AssemblyState;

typedef AssemblySimulationModel = {
  var model:RobotModel;
  /** Link-order hulls in SI units for Simulation.addRobotAtPose. */
  var linkCollisionHulls:Array<Null<Array<Float>>>;
  var closureIds:Array<String>;
  var closures:Array<AssemblySimulationClosure>;
}

typedef AssemblySimulationClosure = {
  var id:String;
  var parent:String;
  var child:String;
  var type:AssemblyJointType;
  var anchorParent:Array<Float>;
  var axisParent:Array<Float>;
}

typedef AssemblyPhysicalPart = {
  var id:String;
  var materialId:String;
  var volume:Float;
  var centerOfMass:Array<Float>;
  var inertia:Array<Float>;
  var density:Float;
  /** Bounded convex support hull, flattened XYZ triples in CAD units. */
  @:optional var collisionHull:Array<Float>;
  @:optional var collisionWarning:String;
  @:optional var collisionErrorRatio:Float;
}

typedef AssemblyPhysicalData = {
  var metresPerUnit:Float;
  var parts:Array<AssemblyPhysicalPart>;
}

/** Converts an assembly's tree joints and physical parts to a RobotKit model. */
class AssemblySimulationBridge {
  /**
   * `freeOccurrences` are parts the simulation moves on its own instead of bolting them to the
   * assembly, such as a workpiece; they get no link, and no joint may touch them.
   */
  public static function toRobotModel(definition:AssemblyDefinition,
      artifact:AssemblyPhysicalData, ?savedState:AssemblyStateRecord,
      ?freeOccurrences:Array<String>):AssemblySimulationModel {
    var free = new Map<String, Bool>();
    if (freeOccurrences != null) for (id in freeOccurrences) free.set(id, true);
    AssemblyDefinitionCodec.validate(definition);
    var sourceDefinition = definition;
    definition = AssemblyDefinitionFlattener.flatten(definition);
    if (artifact == null || artifact.parts == null || artifact.metresPerUnit <= 0)
      throw "Assembly simulation needs a valid scene artifact";
    var scale = artifact.metresPerUnit;
    var parts = new Map<String, AssemblyPhysicalPart>();
    for (part in artifact.parts) parts.set(part.id, part);
    var definitions = new Map<String, materia.assembly.AssemblyDefinition.AssemblyComponentDefinition>();
    for (component in definition.definitions) definitions.set(component.id, component);
    var model = new RobotModel(definition.id);
    var root = model.addLink(new Link("assembly-root"));
    root.mass = 0.001;
    root.inertiaTensor = [1e-6, 0, 0, 0, 1e-6, 0, 0, 0, 1e-6];
    var links = new Map<String, Link>();
    var linkCollisionHulls:Array<Null<Array<Float>>> = [null];
    for (occurrence in definition.occurrences) {
      if (free.exists(occurrence.id)) continue;
      var part = parts.get(occurrence.definition);
      if (part == null) throw 'Assembly occurrence "${occurrence.id}" has no physical part';
      if (!Math.isFinite(part.volume) || part.volume <= 0 || !Math.isFinite(part.density) ||
          part.density <= 0 || part.centerOfMass == null || part.centerOfMass.length != 3 ||
          part.inertia == null || part.inertia.length != 9)
        throw 'Assembly occurrence "${occurrence.id}" has invalid mass properties';
      for (coordinate in part.centerOfMass) if (!Math.isFinite(coordinate))
        throw 'Assembly occurrence "${occurrence.id}" has a non-finite centre of mass';
      for (component in part.inertia) if (!Math.isFinite(component))
        throw 'Assembly occurrence "${occurrence.id}" has a non-finite inertia';
      var link = model.addLink(new Link(occurrence.id));
      link.mass = part.volume * part.density * scale * scale * scale;
      link.centerOfMass = [for (coordinate in part.centerOfMass) coordinate * scale];
      link.inertiaTensor = [for (component in part.inertia) component * part.density * Math.pow(scale, 5)];
      links.set(occurrence.id, link);
      var hull = part.collisionHull;
      if (hull != null) {
        if (hull.length < 12 || hull.length > 64 * 3 || hull.length % 3 != 0)
          throw 'Assembly occurrence "${occurrence.id}" has an invalid collision hull';
        for (coordinate in hull) if (!Math.isFinite(coordinate))
          throw 'Assembly occurrence "${occurrence.id}" has a non-finite collision hull';
        linkCollisionHulls.push([for (coordinate in hull) coordinate * scale]);
      } else linkCollisionHulls.push(null);
    }
    var roots = AssemblyDefinitionCodec.rootOccurrences(definition);
    var placement = new AssemblyState(sourceDefinition, savedState);
    for (occurrence in definition.occurrences) if (roots.exists(occurrence.id) && !free.exists(occurrence.id)) {
      var joint = model.addJoint(new Joint("root-" + occurrence.id, JointType.Fixed,
        root, links.get(occurrence.id)));
      setFrame(joint, placement.worldPose(occurrence.id), true, scale);
    }
    var closures:Array<String> = [];
    var closureGeometry:Array<AssemblySimulationClosure> = [];
    for (edge in definition.joints) {
      if (free.exists(edge.parent) || free.exists(edge.child))
        throw 'Free part is joined by "${edge.id}"; a part the simulation moves on its own cannot be joined';
      if (edge.role == AssemblyJointRole.Closure) {
        closures.push(edge.id);
        var frame = connector(definitions.get(occurrenceDefinition(definition, edge.parent)),
          edge.parentConnector);
        var endpoint = AssemblyFrames.transformPoint(frame, edge.axis.x, edge.axis.y, edge.axis.z);
        closureGeometry.push({id: edge.id, parent: edge.parent, child: edge.child,
          type: edge.type,
          anchorParent: [frame.x * scale, frame.y * scale, frame.z * scale],
          axisParent: [endpoint.x - frame.x, endpoint.y - frame.y, endpoint.z - frame.z]});
        continue;
      }
      var kind:JointType = switch (edge.type) {
        case AssemblyJointType.Fixed: JointType.Fixed;
        case AssemblyJointType.Revolute: JointType.Revolute;
        case AssemblyJointType.Continuous: JointType.Continuous;
        case AssemblyJointType.Prismatic: JointType.Prismatic;
        default: throw 'Unsupported assembly joint type "${edge.type}"';
      };
      var joint = model.addJoint(new Joint(edge.id, kind, links.get(edge.parent), links.get(edge.child)));
      var parentFrame = connector(definitions.get(occurrenceDefinition(definition, edge.parent)), edge.parentConnector);
      var initial = edge.type == AssemblyJointType.Fixed ? 0.0 : placement.joint(edge.id);
      setFrame(joint, AssemblyFrames.compose(parentFrame,
        AssemblyFrames.axisMotion(edge.type, edge.axis, initial)), true, scale);
      setFrame(joint, connector(definitions.get(occurrenceDefinition(definition, edge.child)), edge.childConnector), false, scale);
      joint.axis = [edge.axis.x, edge.axis.y, edge.axis.z];
      var factor = edge.type == AssemblyJointType.Prismatic ? scale : 1.0;
      joint.limits = new JointLimits(edge.limits.lower == null ? -1e9 : (edge.limits.lower - initial) * factor,
        edge.limits.upper == null ? 1e9 : (edge.limits.upper - initial) * factor,
        edge.limits.velocity == null ? 0 : edge.limits.velocity * factor,
        edge.limits.effort == null ? 0 : edge.limits.effort);
    }
    if (definition.couplings != null) for (coupling in definition.couplings) {
      var leader:Null<materia.assembly.AssemblyDefinition.KinematicJoint> = null;
      var follower:Null<materia.assembly.AssemblyDefinition.KinematicJoint> = null;
      for (edge in definition.joints) {
        if (edge.id == coupling.source) leader = edge;
        if (edge.id == coupling.target) follower = edge;
      }
      if (leader == null || follower == null || leader.role == AssemblyJointRole.Closure ||
          follower.role == AssemblyJointRole.Closure)
        throw 'Assembly coupling "${coupling.id}" requires tree joints';
      var leaderScale = leader.type == AssemblyJointType.Prismatic ? scale : 1.0;
      var followerScale = follower.type == AssemblyJointType.Prismatic ? scale : 1.0;
      var ratio = coupling.ratio * followerScale / leaderScale;
      var offset = (coupling.ratio * placement.joint(coupling.source) + coupling.offset -
        placement.joint(coupling.target)) * followerScale;
      model.addCoupling(new JointCoupling(coupling.id, coupling.source,
        coupling.target, ratio, offset));
    }
    return {model: model, linkCollisionHulls: linkCollisionHulls,
      closureIds: closures, closures: closureGeometry};
  }

  static function occurrenceDefinition(definition:AssemblyDefinition, id:String):String {
    for (occurrence in definition.occurrences) if (occurrence.id == id) return occurrence.definition;
    throw 'Unknown assembly occurrence "$id"';
  }

  static function connector(component:materia.assembly.AssemblyDefinition.AssemblyComponentDefinition,
      name:String):AssemblyFrame {
    if (component == null) throw "Missing assembly component definition";
    for (item in component.connectors) if (item.name == name) return item.frame;
    throw 'Missing assembly connector "$name"';
  }

  static function setFrame(joint:Joint, frame:AssemblyFrame, parent:Bool, scale:Float):Void {
    var position = [frame.x * scale, frame.y * scale, frame.z * scale];
    var rotation = [frame.qx, frame.qy, frame.qz, frame.qw];
    if (parent) {
      joint.parentFramePosition = position;
      joint.parentFrameRotation = rotation;
    } else {
      joint.childFramePosition = position;
      joint.childFrameRotation = rotation;
    }
  }
}
