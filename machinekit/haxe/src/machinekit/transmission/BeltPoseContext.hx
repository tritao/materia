package machinekit.transmission;

import cadkit.modeling.AssemblyState;
import materia.assembly.AssemblyDefinition;
import materia.assembly.AssemblyDefinition.AssemblyJointLimits;
import materia.assembly.AssemblyDefinition.AssemblyJointType;
import materia.assembly.AssemblyDefinition.KinematicJoint;
import materia.assembly.AssemblyDefinition.AssemblyJointRole;

/** One uncoupled, unbounded pose copy shared by pose-dependent transmissions in a rebuild. */
class BeltPoseContext {
  public final definition:AssemblyDefinition;
  public final state:AssemblyState;

  public function new(mechanical:AssemblyDefinition) {
    // Only the fields set below are changed, so only they are copied: the record, its joints and their limits. Everything else is
    // shared read-only with `mechanical`, and the state below takes its own full snapshot of this definition.
    definition = Reflect.copy(mechanical);
    definition.joints = [for (joint in mechanical.joints) probeJoint(joint)];
    definition.couplings = [];
    definition.actuators = [];
    definition.encoders = [];
    definition.elasticNetworks = null;
    state = new AssemblyState(definition);
  }

  /** `joint` without limits and without a mechanical stop (stops require bounded travel and belong to the source machine). */
  static function probeJoint(joint:KinematicJoint):KinematicJoint {
    var copy:KinematicJoint = Reflect.copy(joint);
    var limits:AssemblyJointLimits = Reflect.copy(joint.limits);
    limits.lower = null;
    limits.upper = null;
    copy.limits = limits;
    copy.stop = null;
    return copy;
  }

  /** Every source starts at the same authored zero pose. */
  public function reset():Void {
    for (joint in definition.joints) if (joint.role == AssemblyJointRole.Tree
      && joint.type != AssemblyJointType.Fixed) state.setJoint(
        joint.id,
        joint.defaultValue
      );
  }
}
