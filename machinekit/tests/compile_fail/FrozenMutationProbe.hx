import machinekit.assembly.MachineAssemblyDescription.FrozenAssemblyDefinition;

class FrozenMutationProbe {
	static function main():Void {
		var description:FrozenAssemblyDefinition = null;
		description.occurrences[0].initialPose.x = 42;
	}
}
