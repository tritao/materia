package cadkit.modeling;

import kinematicskit.ClosureKind;
import kinematicskit.JointKind;
import kinematicskit.KinematicModel;
import kinematicskit.KinematicModelBuilder;
import kinematicskit.Transform;
import kinematicskit.Vector3;
import materia.assembly.AssemblyDefinition;
import materia.assembly.AssemblyDefinition.AssemblyComponentDefinition;
import materia.assembly.AssemblyDefinition.AssemblyJointRole;
import materia.assembly.AssemblyDefinition.AssemblyJointType;
import materia.assembly.AssemblyDefinition.AssemblyMateKind;
import materia.assembly.AssemblyRecord.AssemblyFrame;

/**
 * A flattened `AssemblyDefinition` compiled into a `kinematicskit` model.
 * Occurrences become bodies (roots placed at their initial pose), tree
 * joints become joints (`parent · parentConnector · motion · childConnector⁻¹`),
 * couplings become shared DOFs, every occurrence connector becomes a frame,
 * and closure joints become closures between their two connector frames.
 * Lengths stay in the definition's unit.
 */
class AssemblyKinematics {
  public final model:KinematicModel;

  function new(model:KinematicModel) {
    this.model = model;
  }

  /**
   * `definition` must already be flattened (see `AssemblyDefinitionFlattener`). With `withMates`, its mates
   * become closures too (only the mate solver wants them: they place parts, so loop solves ignore them).
   */
  public static function compile(definition:AssemblyDefinition, withMates:Bool = false):AssemblyKinematics {
    var builder = new KinematicModelBuilder();
    var components = new Map<String, AssemblyComponentDefinition>();
    for (component in definition.definitions) components.set(component.id, component);
    var bodies = new Map<String, Int>();
    for (occurrence in definition.occurrences) bodies.set(occurrence.id, builder.addBody(occurrence.id));

    var connectorFrames = new Map<String, AssemblyFrame>();
    for (occurrence in definition.occurrences) {
      var component = components.get(occurrence.definition);
      if (component == null) throw 'Missing assembly component "${occurrence.definition}"';
      var body = bodyOf(bodies, occurrence.id);
      for (connector in component.connectors) {
        builder.addFrame(connectorKey(occurrence.id, connector.name), body, fromFrame(connector.frame));
        connectorFrames.set(connectorKey(occurrence.id, connector.name), connector.frame);
      }
    }

    var hasParent = new Map<String, Bool>();
    for (joint in definition.joints) if (joint.role == AssemblyJointRole.Tree) {
      hasParent.set(joint.child, true);
      var parentFrame = connectorFrames.get(connectorKey(joint.parent, joint.parentConnector));
      var childFrame = connectorFrames.get(connectorKey(joint.child, joint.childConnector));
      if (parentFrame == null) throw 'Missing assembly connector "${joint.parent}/${joint.parentConnector}"';
      if (childFrame == null) throw 'Missing assembly connector "${joint.child}/${joint.childConnector}"';
      var kind = switch joint.type {
        case AssemblyJointType.Fixed: JointKind.Fixed;
        case AssemblyJointType.Prismatic: JointKind.Prismatic;
        case AssemblyJointType.Revolute | AssemblyJointType.Continuous: JointKind.Revolute;
        default: throw 'Unsupported assembly joint type "${joint.type}"';
      };
      var axis = joint.axis == null ? null : new Vector3(joint.axis.x, joint.axis.y, joint.axis.z);
      var lower:Null<Float> = null, upper:Null<Float> = null;
      if (joint.limits != null) { lower = joint.limits.lower; upper = joint.limits.upper; }
      builder.addJoint(joint.id, kind, bodyOf(bodies, joint.parent), bodyOf(bodies, joint.child),
        fromFrame(parentFrame), fromFrame(childFrame).inverse(), axis, lower, upper,
        kind == JointKind.Fixed ? 0.0 : joint.defaultValue, joint.type == AssemblyJointType.Continuous);
    }
    for (occurrence in definition.occurrences) if (!hasParent.exists(occurrence.id))
      builder.setRootPose(bodyOf(bodies, occurrence.id), fromFrame(occurrence.initialPose));
    if (definition.couplings != null) for (coupling in definition.couplings)
      builder.couple(coupling.target, coupling.source, coupling.ratio, coupling.offset);

    for (joint in definition.joints) if (joint.role == AssemblyJointRole.Closure) {
      var kind = switch joint.type {
        case AssemblyJointType.Fixed: ClosureKind.Fixed;
        case AssemblyJointType.Prismatic: ClosureKind.Prismatic;
        case AssemblyJointType.Revolute | AssemblyJointType.Continuous: ClosureKind.Revolute;
        case AssemblyJointType.Spherical: ClosureKind.Spherical;
        case AssemblyJointType.Cylindrical: ClosureKind.Cylindrical;
        case AssemblyJointType.Planar: ClosureKind.Planar;
        default: throw 'Unsupported assembly closure type "${joint.type}"';
      };
      var axis = joint.axis == null ? null : new Vector3(joint.axis.x, joint.axis.y, joint.axis.z);
      builder.addClosure(joint.id, kind, frameIndexIn(builder, joint.parent, joint.parentConnector),
        frameIndexIn(builder, joint.child, joint.childConnector), axis, joint.closureTolerance);
    }
    if (withMates && definition.mates != null) for (mate in definition.mates) {
      var kind = switch mate.kind {
        case AssemblyMateKind.Coincident: ClosureKind.Spherical;
        case AssemblyMateKind.Coaxial: ClosureKind.Cylindrical;
        case AssemblyMateKind.Planar: ClosureKind.Planar;
        case AssemblyMateKind.Parallel: ClosureKind.Parallel;
        case AssemblyMateKind.Perpendicular: ClosureKind.Perpendicular;
        case AssemblyMateKind.Distance: ClosureKind.Distance;
        case AssemblyMateKind.Angle: ClosureKind.Angle;
        case AssemblyMateKind.Lock: ClosureKind.Fixed;
        default: throw 'Unsupported assembly mate kind "${mate.kind}"';
      };
      builder.addClosure(mate.id, kind, frameIndexIn(builder, mate.first, mate.firstConnector),
        frameIndexIn(builder, mate.second, mate.secondConnector), new Vector3(mate.axis.x, mate.axis.y, mate.axis.z), null,
        mate.value == null ? 0.0 : mate.value);
    }
    return new AssemblyKinematics(builder.build());
  }

  public function body(occurrenceId:String):Int {
    var index = model.bodyIndex(occurrenceId);
    if (index < 0) throw 'Missing assembly occurrence "$occurrenceId"';
    return index;
  }

  public function connectorFrame(occurrenceId:String, connector:String):Int {
    var index = model.frameIndex(connectorKey(occurrenceId, connector));
    if (index < 0) throw 'Missing assembly connector "$occurrenceId/$connector"';
    return index;
  }

  public static function fromFrame(frame:AssemblyFrame):Transform
    return new Transform(frame.x, frame.y, frame.z, frame.qx, frame.qy, frame.qz, frame.qw);

  public static function toFrame(value:Transform):AssemblyFrame
    return {x: value.x, y: value.y, z: value.z, qx: value.qx, qy: value.qy, qz: value.qz, qw: value.qw};

  /** Frame ID of an occurrence connector; the unit separator cannot appear in either authored name. */
  static function connectorKey(occurrence:String, connector:String):String
    return occurrence + String.fromCharCode(31) + connector;

  static function frameIndexIn(builder:KinematicModelBuilder, occurrence:String, connector:String):Int {
    var index = builder.frameIndex(connectorKey(occurrence, connector));
    if (index < 0) throw 'Missing assembly connector "$occurrence/$connector"';
    return index;
  }

  static function bodyOf(bodies:Map<String, Int>, id:String):Int {
    var body = bodies.get(id);
    if (body == null) throw 'Missing assembly occurrence "$id"';
    return body;
  }
}
