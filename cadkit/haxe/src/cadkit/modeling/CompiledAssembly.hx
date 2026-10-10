package cadkit.modeling;

import materia.assembly.AssemblyDefinition;
import materia.assembly.AssemblyDefinition.AssemblyComponentDefinition;
import materia.assembly.AssemblyDefinition.AssemblyComponentOccurrence;
import materia.assembly.AssemblyDefinition.KinematicJoint;
import materia.assembly.AssemblyDefinitionCodec;
import materia.assembly.AssemblyDefinitionFlattener;

/** Immutable assembly topology shared by independent configurations. Edit a separate definition and compile a new model. */
class CompiledAssembly {
	/** Read-only after publication, including all nested records and arrays. */
	public final definition:AssemblyDefinition;
	public final kinematics:AssemblyKinematics;
	@:allow(cadkit.modeling.AssemblyState) final occurrences:Map<String, AssemblyComponentOccurrence> = [];
	@:allow(cadkit.modeling.AssemblyState) final components:Map<String, AssemblyComponentDefinition> = [];
	@:allow(cadkit.modeling.AssemblyState) final joints:Map<String, KinematicJoint> = [];

	function new(definition:AssemblyDefinition) {
		this.definition = definition;
		for (component in definition.definitions) components.set(component.id, component);
		for (occurrence in definition.occurrences) occurrences.set(occurrence.id, occurrence);
		for (joint in definition.joints) joints.set(joint.id, joint);
		kinematics = AssemblyKinematics.compile(definition);
	}

	/** Snapshot caller-owned editable data; later edits cannot affect this model. */
	public static function snapshot(source:AssemblyDefinition):CompiledAssembly {
		AssemblyDefinitionCodec.validate(source);
		return new CompiledAssembly(AssemblyDefinitionFlattener.flatten(source));
	}

	/** Transfer an exclusively owned definition, such as a freshly decoded artifact. The caller must stop editing it. */
	public static function takeOwnership(source:AssemblyDefinition):CompiledAssembly {
		AssemblyDefinitionCodec.validate(source);
		return new CompiledAssembly(AssemblyDefinitionFlattener.flattenView(source));
	}
}
