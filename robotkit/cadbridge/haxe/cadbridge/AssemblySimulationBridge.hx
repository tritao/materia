package cadbridge;

import materia.project.AssemblyDefinition;
import materia.project.AssemblyDefinition.AssemblyJointRole;
import materia.project.AssemblyDefinition.AssemblyJointType;
import materia.project.AssemblyDefinition.AssemblyJointCoupling;
import materia.project.AssemblyDefinition.AssemblyStateRecord;
import materia.project.AssemblyDefinitionCodec;
import materia.project.AssemblyFrames;
import materia.project.AssemblyRecord.AssemblyFrame;
import robotkit.model.RobotModel;
import robotkit.model.Link;
import robotkit.model.Joint;
import robotkit.model.JointType;
import robotkit.model.JointLimits;
import cadkit.modeling.AssemblyState;

typedef AssemblySimulationModel = {
  var model:RobotModel;
  var couplings:Array<AssemblyJointCoupling>;
  var closureIds:Array<String>;
}

typedef AssemblyPhysicalPart = {
  var id:String;
  var materialId:String;
  var volume:Float;
  var centerOfMass:Array<Float>;
  var inertia:Array<Float>;
  var density:Float;
}

typedef AssemblyPhysicalData = {
  var metresPerUnit:Float;
  var parts:Array<AssemblyPhysicalPart>;
}

/** Converts an assembly's tree joints and physical parts to a RobotKit model. */
class AssemblySimulationBridge {
  public static function toRobotModel(definition:AssemblyDefinition,
      artifact:AssemblyPhysicalData, ?savedState:AssemblyStateRecord):AssemblySimulationModel {
    AssemblyDefinitionCodec.validate(definition);
    if (artifact == null || artifact.parts == null || artifact.metresPerUnit <= 0)
      throw "Assembly simulation needs a valid scene artifact";
    var scale = artifact.metresPerUnit;
    var parts = new Map<String, AssemblyPhysicalPart>();
    for (part in artifact.parts) parts.set(part.id, part);
    var definitions = new Map<String, materia.project.AssemblyDefinition.AssemblyComponentDefinition>();
    for (component in definition.definitions) definitions.set(component.id, component);
    var model = new RobotModel(definition.id);
    var root = model.addLink(new Link("assembly-root"));
    root.mass = 0.001;
    root.inertiaTensor = [1e-6, 0, 0, 0, 1e-6, 0, 0, 0, 1e-6];
    var links = new Map<String, Link>();
    for (occurrence in definition.occurrences) {
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
    }
    var roots = AssemblyDefinitionCodec.rootOccurrences(definition);
    var placement = new AssemblyState(definition, savedState);
    for (occurrence in definition.occurrences) if (roots.exists(occurrence.id)) {
      var joint = model.addJoint(new Joint("root-" + occurrence.id, JointType.Fixed,
        root, links.get(occurrence.id)));
      setFrame(joint, placement.worldPose(occurrence.id), true, scale);
    }
    var closures:Array<String> = [];
    for (edge in definition.joints) {
      if (edge.role == AssemblyJointRole.Closure) { closures.push(edge.id); continue; }
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
    return {model: model, couplings: definition.couplings == null ? [] : definition.couplings.copy(),
      closureIds: closures};
  }

  static function occurrenceDefinition(definition:AssemblyDefinition, id:String):String {
    for (occurrence in definition.occurrences) if (occurrence.id == id) return occurrence.definition;
    throw 'Unknown assembly occurrence "$id"';
  }

  static function connector(component:materia.project.AssemblyDefinition.AssemblyComponentDefinition,
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
