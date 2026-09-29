package machinekit.assembly;

import materia.assembly.AssemblyDefinition;
import materia.assembly.AssemblyDefinition.AssemblySubdefinition;
import machinekit.assembly.MachineAssemblyDescription.FrozenAssemblyDefinition;
import machinekit.assembly.MachineAssemblyDescription.FrozenAssemblySubdefinition;
import materia.assembly.AssemblyDefinitionCodec;

/** Detach mutable builder definitions at the description boundary. */
class FrozenAssemblyDefinitions {
	public static function freeze(value:AssemblyDefinition):FrozenAssemblyDefinition {
		var copy = AssemblyDefinitionCodec.decode(AssemblyDefinitionCodec.encode(value));
		var nested:Array<FrozenAssemblySubdefinition> = copy.assemblies == null ? null :
			[for (item in copy.assemblies) {id: item.id, definitions: item.definitions,
				occurrences: item.occurrences, joints: item.joints, couplings: item.couplings,
				exposedConnectors: item.exposedConnectors}];
		return {schemaVersion: copy.schemaVersion, id: copy.id, lengthUnit: copy.lengthUnit,
			definitions: copy.definitions, occurrences: copy.occurrences, joints: copy.joints,
			couplings: copy.couplings, assemblies: nested, exposedConnectors: copy.exposedConnectors};
	}

	public static function thaw(value:FrozenAssemblyDefinition):AssemblyDefinition {
		var nested:Array<AssemblySubdefinition> = value.assemblies == null ? null :
			[for (item in value.assemblies) {id: item.id, definitions: item.definitions.copy(),
				occurrences: item.occurrences.copy(), joints: item.joints.copy(),
				couplings: item.couplings == null ? null : item.couplings.copy(),
				exposedConnectors: item.exposedConnectors == null ? null : item.exposedConnectors.copy()}];
		var copy:AssemblyDefinition = {schemaVersion: value.schemaVersion, id: value.id,
			lengthUnit: value.lengthUnit, definitions: value.definitions.copy(),
			occurrences: value.occurrences.copy(), joints: value.joints.copy(),
			couplings: value.couplings == null ? null : value.couplings.copy(),
			assemblies: nested, exposedConnectors: value.exposedConnectors == null ? null : value.exposedConnectors.copy()};
		return AssemblyDefinitionCodec.decode(AssemblyDefinitionCodec.encode(copy));
	}
}
