package machinekit.transmission;

import cadkit.modeling.AssemblyState;
import haxeon.wire.JsonWire;
import materia.assembly.AssemblyDefinition;
import materia.assembly.AssemblyDefinition.AssemblyJointType;
import materia.assembly.AssemblyDefinition.AssemblyJointRole;

/** One uncoupled, unbounded pose copy shared by every belt resolved in a rebuild. */
class BeltPoseContext {
	public final definition:AssemblyDefinition;
	public final state:AssemblyState;

	public function new(mechanical:AssemblyDefinition) {
		definition = JsonWire.decode(JsonWire.encode(mechanical));
		definition.couplings = [];
		definition.actuators = [];
		definition.encoders = [];
		definition.elasticNetworks = null;
		for (joint in definition.joints) {
			joint.limits.lower = null;
			joint.limits.upper = null;
		}
		state = new AssemblyState(definition);
	}

	/** Every source starts at the same authored zero pose. */
	public function reset():Void {
		for (joint in definition.joints) if (joint.role == AssemblyJointRole.Tree && joint.type != AssemblyJointType.Fixed)
			state.setJoint(joint.id, joint.defaultValue);
	}
}
