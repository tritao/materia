import machinekit.assembly.MachineAssemblyDescription.FrozenAssemblyDefinition;

class FrozenJointMutationProbe {
	static function main():Void {
		var description:FrozenAssemblyDefinition = null;
		description.joints[0].limits.upper = 10;
	}
}
